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

#include "soxexpand.h"
using namespace matrix;
using namespace std;

SoxExpand::SoxExpand(const SoxExpandConf& conf)  
  : AbstractController("SoxExpand", "0.6"), conf(conf) 
{
  t=0;

  addParameterDef("creativity",&creativity,0  , "creativity term (0: disabled) ");

  addParameterDef("Logarithmic", &loga, false, "whether to use logarithmic error");
  addParameterDef("epsC", &epsC, 0.1,  "learning rate of the controller");
  addParameterDef("epsA", &epsA, 0.1,  "learning rate of the model");
  addParameterDef("s4avg", &s4avg, 1,  "smoothing (number of steps)");
  addParameterDef("s4delay", &s4delay, 1, "delay  (number of steps)");
  addParameterDef("causeaware", &causeaware, 0, "awarness of controller influences");
  addParameterDef("harmony",    &harmony,    0, 
                  "dynamical harmony between internal and external world");
  addParameterDef("creativity", &creativity, 0.1, "use virtual sensor values");
  addParameterDef("sense",        &sense   , 1  , "sensibility");

  addInspectableMatrix("A", &A, false, "model matrix");
  addInspectableMatrix("S", &S, false, "model matrix (sensor branch)");
  addInspectableMatrix("C", &C, false, "controller matrix");
  addInspectableMatrix("L", &L, false, "Jacobi matrix");
  addInspectableMatrix("h", &h, false, "controller bias");
  addInspectableMatrix("b", &b, false, "model bias");
  addInspectableMatrix("AC", &AC, false, "A times C");
  addInspectableMatrix("R", &R, false, "Jacobi matrix");

  addInspectableMatrix("v_avg", &v_avg, false, "input shift (averaged)");
  addInspectableMatrix("x_c", &x_c, false, "context sensor values");

  addInspectableMatrix("B", &this->conf.contextCoupling, false, "Coupling of context sensors");
};

SoxExpand::~SoxExpand(){
}


void SoxExpand::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
 
  number_sensors= sensornumber - conf.numberContextSensors;
  assert(number_sensors>0);
  number_motors = motornumber;

  if(conf.numberContextSensors){
    assert((signed)conf.contextCoupling.getM()==number_motors);
    assert(conf.contextCoupling.getN()==conf.numberContextSensors);
  }    

  A.set(number_sensors, number_motors);
  S.set(number_sensors, number_sensors);
  C.set(number_motors, number_sensors);
  b.set(number_sensors, 1);
  h.set(number_motors, 1);
  L.set(number_sensors, number_sensors);
  v_avg.set(number_sensors, 1);

  AC.set(number_sensors, number_sensors);
  R.set(number_sensors, number_sensors);

  A.toId(); // set a to identity matrix;
  C.toId(); // set a to identity matrix;
  C*=conf.initFeedbackStrength;
   
  x.set(number_sensors,1);
  x_c.set(conf.numberContextSensors,1);
  x_smooth.set(number_sensors,1);
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);   
    y_buffer[k].set(number_motors,1);   

  }
}

matrix::Matrix SoxExpand::getA(){
  return A;
}

void SoxExpand::setA(const matrix::Matrix& _A){
  assert(A.getM() == _A.getM() && A.getN() == _A.getN());
  A=_A;
}

matrix::Matrix SoxExpand::getC(){
  return C;
}

void SoxExpand::setC(const matrix::Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}

matrix::Matrix SoxExpand::getContextC(){
  return conf.contextCoupling;
}
void SoxExpand::setContextC(const matrix::Matrix& CC){
  assert(conf.contextCoupling.getM() == CC.getM() && conf.contextCoupling.getN() == CC.getN());
  conf.contextCoupling=CC;
}


matrix::Matrix SoxExpand::geth(){
  return h;
}

void SoxExpand::seth(const matrix::Matrix& _h){
  assert(h.getM() == _h.getM() && h.getN() == _h.getN());
  h=_h;
}

// performs one step (includes learning). Calculates motor commands from sensor inputs.
void SoxExpand::step(const sensor* x_, int number_sensors, 
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
void SoxExpand::stepNoLearning(const sensor* x_, int number_sensors, 
                                 motor* y_, int number_motors){
  assert(number_sensors <= this->number_sensors + (signed)conf.numberContextSensors 
         && number_motors <= this->number_motors);

  x.set(this->number_sensors,1,x_); // store sensor values
  if(conf.numberContextSensors)
    x_c.set(conf.numberContextSensors,1,x_+this->number_sensors); // store context sensor values
  
  // averaging over the last s4avg values of x_buffer
  s4avg = ::clip(s4avg,1,buffersize-1);
  if(s4avg > 1)
    x_smooth += (x - x_smooth)*(1.0/s4avg);
  else
    x_smooth = x;
  
  x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value
  
  Matrix y;
  if(conf.numberContextSensors){
    // calculate controller values based on current input values (smoothed)  
    y =   (C*(x_smooth + (v_avg*creativity)) + h + conf.contextCoupling * x_c).map(g);
  } else {
    y =   (C*(x_smooth + (v_avg*creativity)) + h).map(g);
  }

  
  // Put new output vector in ring buffer y_buffer
  y_buffer[t%buffersize] = y;

  // convert y to motor* 
  y.convertToBuffer(y_, number_motors);

  // update step counter
  t++;
};
  
      
// learn values h,C,A,b,S
void SoxExpand::learn(){


  // the effective x/y is (actual-steps4delay) element of buffer  
  s4delay = ::clip(s4delay,1,buffersize-1);
  const Matrix& x = x_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& y_creat = y_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& x_fut   = x_buffer[t% buffersize]; // future sensor (with respect to x,y)

  const Matrix& xi = x_fut  - (A * y_creat + b + S * x); // here we use creativity
  
  
  const Matrix& z    = (C * (x) + h); // here no creativity 
  const Matrix& y    = z.map(g);
  const Matrix& g_prime = z.map(g_s);

  L = A * (C & g_prime) + S;
  AC = A * (C);
  R = AC+S;

  const Matrix& eta    = A.pseudoInverse() * xi; 
  const Matrix& y_hat  = y + eta*causeaware;

  const Matrix& chi    =  L.multMT().pseudoInverse()*xi;
  const Matrix& v      = (L^T) * chi;
  const Matrix& mu     = ((A^T) & g_prime) * chi;
  const Matrix& epsrel = (mu & (C * v)) * (sense * 2);
  
  const Matrix& v_hat = v + x * harmony;

  v_avg += ( v  - v_avg ) *.1; 

  double EE = 1.0; 
  if(loga){
    EE = .1/(v.norm_sqr() + .001); // logarithmic error (E = log(v^T v))
  }

  A += (xi * (y_hat^T) * epsA + (A *  -0.0001) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);
  S += (xi * (x^T)     * epsA + (S *  -0.001 ) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);
  b += (xi             * epsA + (b *  -0.0001) * ( epsA > 0 ? 1 : 0)).mapP(0.1, clip);
    
  C += (( mu * (v_hat^T) 
          - (epsrel & y) * (x^T)) * (EE *epsC) ).mapP(.05, clip); 
  // C += (( mu * (v_hat^T) 
  //         - (epsrel & z) * (x^T)) * (EE *epsC)
  //       + (C *  -0.0001) * ( epsC > 0 ? 1 : 0)).mapP(.05, clip); 
  h += ((epsrel & y) * (-EE * epsC)).mapP(.05, clip); 
  
};

  
/* stores the controller values to a given file. */
bool SoxExpand::store(FILE* f) const{  
  // save matrix values
  C.store(f);
  h.store(f);
  A.store(f);
  b.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool SoxExpand::restore(FILE* f){
  // save matrix values
  C.restore(f);
  h.restore(f);
  A.restore(f);
  b.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

