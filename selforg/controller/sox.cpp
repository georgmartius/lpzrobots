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

#include "sox.h"
using namespace matrix;
using namespace std;

Sox::Sox(double init_feedback_strength, bool useExtendedModel, bool useTeaching )  
  : AbstractController("Sox", "0.8"), 
    init_feedback_strength(init_feedback_strength),
    useExtendedModel(useExtendedModel), useTeaching(useTeaching) {
  t=0;


  addParameterDef("Logarithmic", &loga, false, "whether to use logarithmic error");
  addParameterDef("epsC", &epsC, 0.1,     0,5, "learning rate of the controller");
  addParameterDef("epsA", &epsA, 0.1,     0,5, "learning rate of the model");
  addParameterDef("s4avg", &s4avg, 1,     1, buffersize-1, "smoothing (number of steps)");
  addParameterDef("s4delay", &s4delay, 1, 1, buffersize-1, "delay  (number of steps)");
  addParameterDef("sense",  &sense,    1, 0.2,5,           "sensibility");
  addParameterDef("creativity", &creativity, 0, 0, 1,      "creativity term (0: disabled) ");
  addParameterDef("damping",   &damping,     0.00001, 0,0.01, "forgetting term for model");
  addParameterDef("causeaware", &causeaware, useExtendedModel ? 0.01 : 0 , 0,0.1, 
                  "awarness of controller influences");
  addParameterDef("harmony",    &harmony,    0, 0,0.1,
                  "dynamical harmony between internal and external world");
  addParameterDef("pseudo",   &pseudo   , 0  , 
       "type of pseudo inverse: 0 moore penrose, 1 sensor space, 2 motor space, 3 special");

  if(useTeaching){
    addParameterDef("gamma",  &gamma,    0.01, 0, 1,           "guidance factor (teaching)");
    addInspectableMatrix("y_G", &y_teaching, "teaching signal at motor neurons");
  }

  addInspectableMatrix("A", &A, false, "model matrix");
  if(useExtendedModel)
    addInspectableMatrix("S", &S, false, "model matrix (sensor branch)");
  addInspectableMatrix("C", &C, false, "controller matrix");
  addInspectableMatrix("L", &L, false, "Jacobi matrix");
  addInspectableMatrix("h", &h, false, "controller bias");
  addInspectableMatrix("b", &b, false, "model bias");
  addInspectableMatrix("R", &R, false, "linear response matrix");

  addInspectableMatrix("v_avg", &v_avg, "input shift (averaged)");

  intern_isTeaching = false;


};

Sox::~Sox(){
}


void Sox::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
 
  number_sensors= sensornumber;
  number_motors = motornumber;
  A.set(number_sensors, number_motors);
  S.set(number_sensors, number_sensors);
  C.set(number_motors, number_sensors);
  b.set(number_sensors, 1);
  h.set(number_motors, 1);
  L.set(number_sensors, number_sensors);
  v_avg.set(number_sensors, 1);
  A_native.set(number_sensors, number_motors);
  C_native.set(number_motors, number_sensors);

  R.set(number_sensors, number_sensors);

  A.toId(); // set a to identity matrix;
  C.toId(); // set a to identity matrix;
  C*=init_feedback_strength;

  S.toId();
  S*=0.05;

  // if motor babbling is used then this is overwritten
  A_native.toId();
  C_native.toId();
  C_native*=1.2;   
   
  y_teaching.set(number_motors, 1);

  x.set(number_sensors,1);
  x_smooth.set(number_sensors,1);
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);   
    y_buffer[k].set(number_motors,1);   

  }
}

matrix::Matrix Sox::getA(){
  return A;
}

void Sox::setA(const matrix::Matrix& _A){
  assert(A.getM() == _A.getM() && A.getN() == _A.getN());
  A=_A;
}

matrix::Matrix Sox::getC(){
  return C;
}

void Sox::setC(const matrix::Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}

matrix::Matrix Sox::geth(){
  return h;
}

void Sox::seth(const matrix::Matrix& _h){
  assert(h.getM() == _h.getM() && h.getN() == _h.getN());
  h=_h;
}

// performs one step (includes learning). Calculates motor commands from sensor inputs.
void Sox::step(const sensor* x_, int number_sensors, 
                       motor* y_, int number_motors){
  stepNoLearning(x_, number_sensors, y_, number_motors);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // learn controller and model
  if(epsC!=0 || epsA!=0) 
    learn();

  // update step counter
  t++;
};


// performs one step without learning. Calulates motor commands from sensor inputs.
void Sox::stepNoLearning(const sensor* x_, int number_sensors, 
                                 motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors 
         && (unsigned)number_motors <= this->number_motors);

  x.set(number_sensors,1,x_); // store sensor values
  
  // averaging over the last s4avg values of x_buffer
  s4avg = ::clip(s4avg,1,buffersize-1);
  if(s4avg > 1)
    x_smooth += (x - x_smooth)*(1.0/s4avg);
  else
    x_smooth = x;
  
  x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value
  
  // calculate controller values based on current input values (smoothed)  
  Matrix y =   (C*(x_smooth + (v_avg*creativity)) + h).map(g);
  
  // Put new output vector in ring buffer y_buffer
  y_buffer[t%buffersize] = y;

  // convert y to motor* 
  y.convertToBuffer(y_, number_motors);

  // update step counter
  t++;
};
  

void Sox::motorBabblingStep(const sensor* x_, int number_sensors,
                            const motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors 
         && (unsigned)number_motors <= this->number_motors);
  x.set(number_sensors,1,x_);
  x_buffer[t%buffersize] = x;
  Matrix y(number_motors,1,y_);
  y_buffer[t%buffersize] = y;

  double factor = .1; // we learn slower here
  // learn model:
  const Matrix& x_tm1 = x_buffer[(t - 1 + buffersize) % buffersize];
  const Matrix& y_tm1 = y_buffer[(t - 1 + buffersize) % buffersize];
  const Matrix& xp    = (A * y_tm1+ b + S * x_tm1);
  const Matrix& xi   = x - xp;
  
  A += (xi * (y_tm1^T) * (epsA * factor) + (A *  -damping) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);
  b += (xi           * (epsA * factor) + (b *  -damping) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);
  if(useExtendedModel)
    S += (xi * (x_tm1^T) * (epsA*factor) + (S *  -damping*10 ) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);

  // learn controller
  const Matrix& z       = (C * (x_tm1) + h); // here no creativity 
  const Matrix& yp      = z.map(g);
  const Matrix& g_prime = z.map(g_s);  
  const Matrix& delta   = (y_tm1 - yp) & g_prime;
  C += ((delta * (x_tm1^T)) * (epsC *factor)).mapP(0.1, clip) + (C *  -damping);
  h += (delta * (epsC *factor)).mapP(0.1, clip);
  C_native = C;
  A_native = A;
  t++;
}


Matrix Sox::pseudoInvL(const Matrix& L, const Matrix& A, const Matrix& C){
  if(pseudo == 0){
    return L.pseudoInverse();
  }else{
    const Matrix& P = pseudo==1 || pseudo==2 ? A^T : C;
    const Matrix& Q = pseudo==1              ? C^T : A;
    return Q *((P * L * Q)^(-1)) * P;
  }
}

      
// learn values h,C,A,b,S
void Sox::learn(){


  // the effective x/y is (actual-steps4delay) element of buffer  
  s4delay = ::clip(s4delay,1,buffersize-1);
  const Matrix& x       = x_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& y_creat = y_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& x_fut   = x_buffer[t% buffersize]; // future sensor (with respect to x,y)

  const Matrix& xi      = x_fut  - (A * y_creat + b + S * x); // here we use creativity
    
  const Matrix& z       = (C * (x) + h); // here no creativity 
  const Matrix& y       = z.map(g);
  const Matrix& g_prime = z.map(g_s);

  L = A * (C & g_prime) + S;
  R = A * C+S; // this is only used for visualization

  const Matrix& eta    = A.pseudoInverse() * xi; 
  const Matrix& y_hat  = y + eta*causeaware;

  const Matrix& Lplus  = pseudoInvL(L,A,C);
  const Matrix& v      = Lplus * xi; 
  const Matrix& chi    = (Lplus^T) * v;

  const Matrix& mu     = ((A^T) & g_prime) * chi;
  const Matrix& epsrel = (mu & (C * v)) * (sense * 2);
  
  const Matrix& v_hat = v + x * harmony;

  v_avg += ( v  - v_avg ) *.1; 

  double EE = 1.0; 
  if(loga){
    EE = .1/(v.norm_sqr() + .001); // logarithmic error (E = log(v^T v))
  }
  if(epsA > 0){
    A   += (xi * (y_hat^T) * epsA                      ).mapP(0.1, clip);
    if(damping)
      A += (((A_native-A).map(power3))*damping         ).mapP(0.1, clip);
    if(useExtendedModel)
      S += (xi * (x^T)     * epsA + (S *  -damping*10) ).mapP(0.1, clip);
    b   += (xi             * epsA + (b *  -damping)    ).mapP(0.1, clip);
  }
  if(epsC > 0){
    C += (( mu * (v_hat^T) 
            - (epsrel & y) * (x^T))   * (EE * epsC) ).mapP(.05, clip); 
    if(damping)
      C += (((C_native-C).map(power3))*damping      ).mapP(.05, clip);
    h += ((mu*harmony - (epsrel & y)) * (EE * epsC) ).mapP(.05, clip);
    
    if(intern_isTeaching && gamma > 0){    
      // scale of the additional terms
      const Matrix& metric = (A^T) * Lplus.multTM() * A;
      
      const Matrix& y      = y_buffer[(t-1)% buffersize];
      const Matrix& xsi    = y_teaching - y;
      const Matrix& delta  = xsi.multrowwise(g_prime);
      C += ((metric * delta*(x^T) ) * (gamma * epsC)).mapP(.05, clip);
      h += ((metric * delta)        * (gamma * epsC)).mapP(.05, clip);
      // after we applied teaching signal it is switched off until new signal is given
      intern_isTeaching    = false; 
    }  
  }

};


void Sox::setMotorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_motors && teaching.getN() == 1);  
  // Note: through the clipping the otherwise effectless 
  //  teaching with old motor value has now an effect, 
  //  namely to drive out of the saturation region. 
  y_teaching= teaching.mapP(0.95,clip);
  intern_isTeaching=true;
}

void Sox::setSensorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_sensors && teaching.getN() == 1);  
  // calculate the y_teaching, 
  // that belongs to the distal teaching value by the inverse model.
  y_teaching = (A.pseudoInverse() * (teaching-b)).mapP(0.95, clip); 
  intern_isTeaching=true;  
}

matrix::Matrix Sox::getLastMotorValues(){
  return y_buffer[(t-1+buffersize)%buffersize];
}

matrix::Matrix Sox::getLastSensorValues(){
  return x_buffer[(t-1+buffersize)%buffersize];
}
  
/* stores the controller values to a given file. */
bool Sox::store(FILE* f) const{  
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
bool Sox::restore(FILE* f){
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

