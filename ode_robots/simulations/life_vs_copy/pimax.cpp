/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 ***************************************************************************/

#include "pimax.h"
using namespace matrix;
using namespace std;

PiMax::PiMax(const PiMaxConf& conf_)
  : AbstractController("PiMax", "0.2"), 
    conf(conf_), t(0) 
{

  addParameterDef("metrics", &useMetric, true, "whether to use metrics in gradient space");
  addParameterDef("epsC", &epsC, 0.1,    0,5, "learning rate of the controller");
  addParameterDef("epsA", &epsA, 0.05,    0,5, "learning rate of the model");
  addParameterDef("sense",  &sense,    1, 0.2,5,      "sensibility");
  addParameterDef("creativity", &creativity, 0, 0, 1, "creativity term (0: disabled) ");
  addParameterDef("damping",   &damping,     0.00001, 0,0.01, "forgetting term for model");
  addParameterDef("causeaware", &causeaware, conf.useExtendedModel ? 0.0 : 0 , 0,0.1, 
                  "awarness of controller influences");
  addParameterDef("harmony",    &harmony,    0, 0,0.1,
                  "dynamical harmony between internal and external world");

  addParameterDef("tau",   &tau   , 2  , 2, buffersize-1,
                  "length of time window");
  epsSigma=0.01;
  factorH=1;
  if(!conf.onlyMainParameters){
    addParameter("s4avg", &conf.steps4Averaging, 1, buffersize-1, 
                    "smoothing (number of steps)");
    addParameter("s4delay", &conf.steps4Delay,   1, buffersize-1, 
                    "delay  (number of steps)");
    addParameter("epsSigma", &epsSigma,  0,  1, "update rate for cov matrix");
    addParameter("factorH", &factorH,  0,  10, "learning rate factor for h");
  }

  gamma=0;
  if(conf.useTeaching){
    addParameterDef("gamma",  &gamma,    0.01, 0, 1, "guidance factor (teaching)");
    addInspectableMatrix("a_G", &a_teaching, "teaching signal at motor neurons");
  }

  addInspectableMatrix("A", &A, conf.someInternalParams, "model matrix");
  if(conf.useExtendedModel)
    addInspectableMatrix("S", &S, conf.someInternalParams, "model matrix (sensor branch)");
  addInspectableMatrix("C", &C, conf.someInternalParams, "controller matrix");
  addInspectableMatrix("L", &L, conf.someInternalParams, "Jacobi matrix");
  addInspectableMatrix("h", &h, conf.someInternalParams, "controller bias");
  addInspectableMatrix("b", &b, conf.someInternalParams, "model bias");
  addInspectableMatrix("ds0", &ds0, conf.someInternalParams, "ds0");

  addInspectableMatrix("Sigma", &Sigma, conf.someInternalParams, "covariance matrix of noise");


  intern_isTeaching = false;

};

PiMax::~PiMax(){
}


void PiMax::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
 
  number_sensors= sensornumber;
  number_motors = motornumber;
  A.set(number_sensors, number_motors);
  S.set(number_sensors, number_sensors);
  C.set(number_motors, number_sensors);
  b.set(number_sensors, 1);
  h.set(number_motors, 1);
  diago.set(number_motors, 1);//TEST
  L.set(number_sensors, number_sensors);

  A_native.set(number_sensors, number_motors);
  C_native.set(number_motors, number_sensors);

  ds0.set(number_sensors, 1);

  Sigma.set(number_sensors, number_sensors);
  Sigma.toId();
  Sigma*=0.01;

  A.toId(); // set a to identity matrix;
  C.toId(); // set a to identity matrix;
  C*=conf.initFeedbackStrength;

  S.toId();
  S*=0.05;

  // if motor babbling is used then this is overwritten
  A_native.toId();
  C_native.toId();
  C_native*=1.2;   
   
  a_teaching.set(number_motors, 1);

  s.set(number_sensors,1);
  s_smooth.set(number_sensors,1);
  for (unsigned int k = 0; k < buffersize; k++) {
    s_buffer[k].set( number_sensors,1);   
    a_buffer[k].set( number_motors,1);   
    xi_buffer[k].set(number_sensors,1);   
    gs_buffer[k].set( number_motors,1);
    gs_buffer[k].toMapP(1.0,constant);// should never be 0;
    L_buffer[k].set( number_sensors,number_sensors);       
  }
}

matrix::Matrix PiMax::getA(){
  return A;
}

void PiMax::setA(const matrix::Matrix& _A){
  assert(A.getM() == _A.getM() && A.getN() == _A.getN());
  A=_A;
}

matrix::Matrix PiMax::getC(){
  return C;
}

void PiMax::setC(const matrix::Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}

matrix::Matrix PiMax::geth(){
  return h;
}

void PiMax::seth(const matrix::Matrix& _h){
  assert(h.getM() == _h.getM() && h.getN() == _h.getN());
  h=_h;
}

// performs one step (includes learning). Calculates motor commands from sensor inputs.
void PiMax::step(const sensor* s_, int number_sensors, 
                       motor* a_, int number_motors){
  stepNoLearning(s_, number_sensors, a_, number_motors);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // learn controller and model
  if(epsC!=0 || epsA!=0) 
    learn();

  // update step counter
  t++;
};


// performs one step without learning. Calulates motor commands from sensor inputs.
void PiMax::stepNoLearning(const sensor* s_, int number_sensors, 
                                 motor* a_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors 
         && (unsigned)number_motors <= this->number_motors);

  s.set(number_sensors,1,s_); // store sensor values
  
  // averaging over the last s4avg values of s_buffer
  conf.steps4Averaging = ::clip(conf.steps4Averaging,1,buffersize-1);
  if(conf.steps4Averaging > 1)
    s_smooth += (s - s_smooth)*(1.0/conf.steps4Averaging);
  else
    s_smooth = s;
  
  s_buffer[t%buffersize] = s_smooth; // we store the smoothed sensor value
  
  // calculate controller values based on current input values (smoothed)  
  Matrix a =   (C*(s_smooth /*+ (v_avg*creativity)*/) + h).map(g);
  
  // Put new output vector in ring buffer a_buffer
  a_buffer[t%buffersize] = a;

  // convert a to motor* 
  a.convertToBuffer(a_, number_motors);

  // update step counter
  t++;
};
  

void PiMax::motorBabblingStep(const sensor* s_, int number_sensors,
                            const motor* a_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors 
         && (unsigned)number_motors <= this->number_motors);
  s.set(number_sensors,1,s_);
  s_buffer[t%buffersize] = s;
  Matrix a(number_motors,1,a_);
  a_buffer[t%buffersize] = a;

  double factor = .1; // we learn slower here
  // learn model:
  const Matrix& s_tm1 = s_buffer[(t - 1 + buffersize) % buffersize];
  const Matrix& a_tm1 = a_buffer[(t - 1 + buffersize) % buffersize];
  const Matrix& sp    = (A * a_tm1+ b + S * s_tm1);
  const Matrix& xi   = s - sp;
  
  A += (xi * (a_tm1^T) * (epsA * factor) + (A *  -damping) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);
  b += (xi           * (epsA * factor) + (b *  -damping) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);
  if(conf.useExtendedModel)
    S += (xi * (s_tm1^T) * (epsA*factor) + (S *  -damping*10 ) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);

  // learn controller
  const Matrix& z       = (C * (s_tm1) + h); // here no creativity 
  const Matrix& ap      = z.map(g);
  const Matrix& g_prime = z.map(g_s);  
  const Matrix& delta   = (a_tm1 - ap) & g_prime;
  C += ((delta * (s_tm1^T)) * (epsC *factor)).mapP(0.1, clip) + (C *  -damping);
  h += (delta * (epsC *factor*factorH)).mapP(0.1, clip);
  C_native = C;
  A_native = A;
  t++;
}

      
// learn values h,C,A,b,S
void PiMax::learn(){

  const Matrix& s       =  s_buffer[(t - 1) % buffersize];
  const Matrix& a_creat =  a_buffer[(t - 1) % buffersize];
  const Matrix& s_fut   =  s_buffer[t% buffersize]; // future sensor (with respect to s,a)

  const Matrix& xi      = s_fut  - (A * a_creat + b + S * s); // here we use creativity
  xi_buffer[t % buffersize] = xi;
    
  const Matrix& z       = (C * (s) + h); // here no creativity 
  const Matrix& a       = z.map(g);
  const Matrix& g_prime = z.map(g_s);
  gs_buffer[(t-1) % buffersize] = g_prime; 

  L                     = A * (C & g_prime) + S;
  L_buffer[(t-1) % buffersize] = L; 

  if(tau<2) tau=2;

  /// calc all delta s_{t-l};
  vector<Matrix> ds;
  ds.resize(tau+1);  
  //semantic: ds[l] == \delta s_{t-l} 
  // this means the array of ds is expands backwards in time
  ds[tau].set(number_sensors,1); // vector of zeros;
  for(int l=tau-1; l>=0; l--){
    ds[l] = (L_buffer[(t-(l+1))%buffersize]*ds[l+1] + xi_buffer[(t-l)%buffersize]
             );//.mapP(0.2,clip); //TEST clipping outside the loop 
  }
  ds[0] = ds[0].mapP(0.1,clip);
  Sigma                += (ds[0].multMT()-Sigma)*epsSigma;
  ds0 = ds[0];

  vector<Matrix> du;
  double lambda = 0.000001;
  const Matrix& sigma_ds_t = ((Sigma + (Sigma^0)*lambda)^(-1))*ds[0];
  du.resize(tau+1);  
  //semantic: du[l] == \delta u_{t-l+1}  
  // \delta u_{t-l+1} = L^{l-1}(t-l)Sigma^{-1} \delta s_{t}
  du[1] = sigma_ds_t;
  for(int l=2; l<tau; l++){    
    du[l] = (L_buffer[(t-l)%buffersize]^T) * du[l-1];
  }
  if(epsC > 0){
    //TEST EE 
    //double EE = ((sigma_ds_t^T)*sigma_ds_t).val(0,0)/1000.0; 
    // cout << "EE" << EE <<endl;
    double EE = 1.0;
    double epsCN = epsC/(EE*100.0*(tau-1));
    // l means: time point t-l
    // x,y, L and g' are to be taken at t-l
    for(int l=1; l<tau; l++){

      const Matrix& al      = a_buffer[(t-l)%buffersize];
      const Matrix& sl      = s_buffer[(t-l)%buffersize];
      const Matrix& gs      = gs_buffer[(t-l)%buffersize];
      const Matrix& dmu     = ((A^T)*du[l]) & gs;
      //%%%%TEST: xxxx
      for (int i=0; i<number_motors; i++) diago.val(i,0)= (C*A).val(i,i);
      diago = diago&gs; 
      const Matrix& epsrel  = (C*ds[l]) & dmu * 2 * sense +  ((((C*ds[l]) & dmu)&diago) * 2 * sense);
      
      const Matrix& metric = useMetric ? gs.map(one_over).map(sqr) : gs.mapP(1, constant);

      //cout << l << "  ds:"  <<(ds[l]^T) <<  " du:" << (du[l]^T) << endl;

      C += ((( dmu * (ds[l]^T) - (epsrel & al) * (sl^T)) & metric) * epsCN
            ).mapP(.015, clip); //TEST clip 
      h += ((((epsrel & a)) & metric) * (-epsCN*factorH) ).mapP(.05, clip);//TEST clip 
    
    }
    if(damping)
      C += (((C_native-C).map(power3))*damping           ).mapP(.05, clip);
  }


  if(epsA > 0){
    const Matrix& eta     = A.pseudoInverse() * xi; 
    const Matrix& a_hat   = a + eta*causeaware;
    A   += (xi * (a_hat^T) * epsA                      ).mapP(0.1, clip);
    if(damping)
      A += (((A_native-A).map(power3))*damping         ).mapP(0.1, clip);
    if(conf.useExtendedModel)
      S += (xi * (s^T)     * epsA + (S *  -damping*10) ).mapP(0.1, clip);
    b   += (xi             * epsA + (b *  -damping)    ).mapP(0.1, clip);
  }

};


void PiMax::setMotorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_motors && teaching.getN() == 1);  
  // Note: through the clipping the otherwise effectless 
  //  teaching with old motor value has now an effect, 
  //  namely to drive out of the saturation region. 
  a_teaching= teaching.mapP(0.95,clip);
  intern_isTeaching=true;
}

void PiMax::setSensorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_sensors && teaching.getN() == 1);  
  // calculate the a_teaching, 
  // that belongs to the distal teaching value by the inverse model.
  a_teaching = (A.pseudoInverse() * (teaching-b)).mapP(0.95, clip); 
  intern_isTeaching=true;  
}

matrix::Matrix PiMax::getLastMotorValues(){
  return a_buffer[(t-1+buffersize)%buffersize];
}

matrix::Matrix PiMax::getLastSensorValues(){
  return s_buffer[(t-1+buffersize)%buffersize];
}
  
/* stores the controller values to a given file. */
bool PiMax::store(FILE* f) const{  
  // save matrix values
  C.store(f);
  h.store(f);
  A.store(f);
  b.store(f);
  S.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool PiMax::restore(FILE* f){
  // save matrix values
  C.restore(f);
  h.restore(f);
  A.restore(f);
  b.restore(f);
  S.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

