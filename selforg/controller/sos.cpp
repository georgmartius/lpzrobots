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

#include "sos.h"
using namespace matrix;
using namespace std;

Sos::Sos(double init_feedback_strength)
  : AbstractController("Sos", "0.6"), init_feedback_strength(init_feedback_strength){

  t=0;
  addParameterDef("creativity",&creativity,0  ,0 , 1, "creativity term (0: disabled) ");

  addParameterDef("TLE",   &TLE, true, "whether to use the time loop error");
  addParameterDef("Logarithmic", &loga, false, "whether to use logarithmic error");
  addParameterDef("epsC", &epsC, 0.1, 0,2, "learning rate of the controller");
  addParameterDef("epsA", &epsA, 0.1, 0,2, "learning rate of the model");
  addParameterDef("s4avg", &s4avg, 1, 1, buffersize, "smoothing (number of steps)");
  addParameterDef("s4delay", &s4delay, 1, 1, buffersize, "delay  (number of steps)");

  addInspectableMatrix("A", &A, false, "model matrix");
  addInspectableMatrix("C", &C, false, "controller matrix");
  addInspectableMatrix("h", &h, false, "controller bias");
  addInspectableMatrix("b", &b, false, "model bias");

  addInspectableMatrix("v_avg", &v_avg, "input shift (averaged)");

};

Sos::~Sos(){
}


void Sos::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
 
  number_sensors= sensornumber;
  number_motors = motornumber;
  A.set(number_sensors, number_motors);
  C.set(number_motors, number_sensors);
  b.set(number_sensors, 1);
  h.set(number_motors, 1);
  L.set(number_sensors, number_sensors);
  v_avg.set(number_sensors, 1);

  A.toId(); // set a to identity matrix;
  C.toId(); // set a to identity matrix;
  C*=init_feedback_strength;
  
  x.set(number_sensors,1);
  x_smooth.set(number_sensors,1);
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);   
    y_buffer[k].set(number_motors,1);   

  }
}

matrix::Matrix Sos::getA(){
  return A;
}

void Sos::setA(const matrix::Matrix& _A){
  assert(A.getM() == _A.getM() && A.getN() == _A.getN());
  A=_A;
}

matrix::Matrix Sos::getC(){
  return C;
}

void Sos::setC(const matrix::Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}

matrix::Matrix Sos::geth(){
  return h;
}

void Sos::seth(const matrix::Matrix& _h){
  assert(h.getM() == _h.getM() && h.getN() == _h.getN());
  h=_h;
}

// performs one step (includes learning). Calculates motor commands from sensor inputs.
void Sos::step(const sensor* x_, int number_sensors, 
                       motor* y_, int number_motors){
  stepNoLearning(x_, number_sensors, y_, number_motors);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // learn controller and model
  learn();

  // update step counter
  t++;
};


// performs one step without learning. Calulates motor commands from sensor inputs.
void Sos::stepNoLearning(const sensor* x_, int number_sensors, 
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
  
      
// learn values h,C,A
void Sos::learn(){

  Matrix C_update(number_motors,number_sensors);
  Matrix h_update(number_motors,1);

  // the effective x/y is (actual-steps4delay) element of buffer  
  s4delay = ::clip(s4delay,1,buffersize-1);
  const Matrix& x = x_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& y = y_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& x_fut = x_buffer[t% buffersize]; // future sensor (with respect to x,y)
  
  const Matrix& z    = (C * (x + v_avg * creativity) + h);

  const Matrix& g_prime = z.map(g_s);
  const Matrix& g_prime_inv = g_prime.map(one_over);

  const Matrix& xsi = x_fut  - (A* y + b);

  A += (xsi * (y^T) * epsA + (A *  -0.003) * ( epsA > 0 ? 1 : 0)).mapP(0.1,clip);
  b += (xsi *  epsA        + (b *  -0.001) * ( epsA > 0 ? 1 : 0)).mapP(0.1,clip);

  //************ TLE (homeokinetic) learning **********
  // Matrix  eta = (A^-1) * xsi; 
  const Matrix&  eta = A.pseudoInverse() * xsi; 
  
  const Matrix& zeta  = (eta & g_prime_inv).mapP(1.0, clip);
  
  // C.multMT = (C*(C^T))
  const Matrix& mue   = (((C.multMT().pluslambdaI())^-1)*zeta);
  
  const Matrix& v =  ( (C^T) * mue ).mapP(1.0, clip);
  
  v_avg += ( v  - v_avg ) *.1; 
  
  double EE = 1.0; 
  if(loga){
    EE = .1/(v.norm_sqr() + .001); // logarithmic error (E = log(v^T v))
  }
  
  C_update =  (( mue * (v^T) )
               + (( mue & y & zeta) *(-2) * (x^T))) * (EE *epsC); 
  h_update =  ( mue & y & zeta) * (-2 *EE * epsC); 
  
  
  // apply updates to h,C (clipped to [-.01,0.01])
  h += h_update.mapP(.1, clip);
  C += C_update.mapP(.05, clip);
};

  
/* stores the controller values to a given file. */
bool Sos::store(FILE* f) const{  
  // save matrix values
  C.store(f);
  h.store(f);
  A.store(f);
  b.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool Sos::restore(FILE* f){
  // save matrix values
  C.restore(f);
  h.restore(f);
  A.restore(f);
  b.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

