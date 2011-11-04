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

#include "homeokinesis.h"
using namespace matrix;
using namespace std;

Homeokinesis::Homeokinesis()
  : AbstractController("Homeokinesis", "0.9"){
  t=0;

  addParameterDef("creativity",&creativity,0, 0, 100, "creativity term (0: disabled) ");
  addParameterDef("epsC", &epsC, 0.1,  0, 1, "learning rate of the controller");
  addParameterDef("epsA", &epsA, 0.1, 0, 1, "learning rate of the model");
  addParameterDef("s4avg", &s4avg, 1, 0, 1000, "number of steps to smooth the sensor values");
  addParameterDef("dampA", &dampA, 0.00001, 0, 1, "damping factor for model learning");
  addParameterDef("s4delay", &s4delay, 1,  0, 1000,
                  "number of steps to delay motor values (delay in the loop)");

  addInspectableMatrix("A", &A, false, "model matrix");
  addInspectableMatrix("C", &C, false, "controller matrix");
  addInspectableMatrix("h", &h, false, "controller bias");
  addInspectableMatrix("b", &b, false, "model bias");

  addInspectableMatrix("v_avg", &v_avg, "input shift (averaged)");
};

Homeokinesis::~Homeokinesis(){
}


void Homeokinesis::init(int sensornumber, int motornumber, RandGen* randGen){
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
  //   A*=0.1;
  C.toId(); // set a to identity matrix;
  C*=.4;
  
  x.set(number_sensors,1);
  x_smooth.set(number_sensors,1);
  for (unsigned int k = 0; k < buffersize; k++) {
    y_buffer[k].set(number_motors,1);   

  }
}


/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void Homeokinesis::step(const sensor* x_, int number_sensors, 
                       motor* y_, int number_motors){
  stepNoLearning(x_, number_sensors, y_, number_motors);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // calculate effective input/output, which is (actual-steps4delay) element of buffer
  const Matrix& y_effective = y_buffer[(t - max(s4delay,1) + buffersize) % buffersize];

  // learn controller with effective input/output
  learn(x, y_effective);

  // update step counter
  t++;
};


/// performs one step without learning. Calulates motor commands from sensor inputs.
void Homeokinesis::stepNoLearning(const sensor* x_, int number_sensors, 
                                 motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors 
         && (unsigned)number_motors <= this->number_motors);

  x.set(number_sensors,1,x_); // store sensor values
  
  // averaging over the last s4avg values of x_buffer
  x_smooth += (x - x_smooth)*(1.0/max(s4avg,1));

  // calculate controller values based on smoothed input values
  //   Matrix y = (C*(x_smooth) + h).map(g);
  const Matrix& y =   (C*(x + v_avg*creativity) + h).map(g);

  // Put new output vector in ring buffer y_buffer
  y_buffer[t%buffersize] = y;

  // convert y to motor* 
  y.convertToBuffer(y_, number_motors);

  // update step counter
  t++;
};
  
      

/// learn values h,C,A
void Homeokinesis::learn(const Matrix& x, const Matrix& y){

  Matrix C_update(number_motors,number_sensors);
  Matrix h_update(number_motors,1);
 
  // Todo: test both versions!
  //   const Matrix& z    = (C * x_delay/*_smooth*/ + h);    
  const Matrix& z    = (C * (x + v_avg * creativity)/*_smooth*/ + h);//TEST

  const Matrix& g_prime = z.map(g_s);
  const Matrix& g_prime_inv = g_prime.map(one_over);

  const Matrix& xsi = x  - (A* y + b);

  A += (xsi * (y^T) * epsA - (A * dampA) * ( epsA > 0 ? 1 : 0)).mapP(0.1,clip);
  b += (xsi *  epsA        - (b * dampA) * ( epsA > 0 ? 1 : 0)).mapP(0.1,clip);
  //   b += (x-b)*.03 + ( b&b&b) *-.0001;


  /************ TLE (homeokinetic) learning **********/
  // Matrix  eta = (A^-1) * xsi; 
  const Matrix&  eta = A.pseudoInverse() * xsi; 
  
  const Matrix& zeta  = (eta & g_prime_inv);//.mapP(1.0, clip);
  
  // C.multMT = (C*(C^T))
  const Matrix& mue   = (((C.multMT().pluslambdaI())^-1)*zeta);//.map(g); 
  
  const Matrix& v =  ( (C^T) * mue );//.mapP(1.0, clip);
  
  v_avg += ( v  - v_avg ) *.1; 
  
  double  EE = /*sqrt*/(v.norm_sqr() + .001); 
  //       double EE; EE = /*sqrt*/(xsi.norm_sqr*() + .0001); 
  //  EE = .1/EE; // logarithmic error (E = log(v^T v))
  EE = 1; // normal error
  
  C_update =  (( mue * (v^T) )
               + (( mue & y & zeta) *(-2) * (x^T))) * (EE *epsC); 
  
  h_update =  ( mue & y & zeta) * (-2 *EE * epsC); 
  
  // if (epsC > 0 ) h_update += ( h & h & h )*-.001;  
  /********** end of homeokinetic learning  ***********/

  // apply updates to h,C
  h += h_update.mapP(1.0, clip);
  C += C_update.mapP(1.0, clip);
};

  
/** stores the controller values to a given file. */
bool Homeokinesis::store(FILE* f) const{  
  // save matrix values
  C.store(f);
  h.store(f);
  A.store(f);
  b.store(f);
  Configurable::print(f,0);
  return true;
}

/** loads the controller values from a given file. */
bool Homeokinesis::restore(FILE* f){
  // save matrix values
  C.restore(f);
  h.restore(f);
  A.restore(f);
  b.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}


Matrix Homeokinesis::getA(){
  return A;
}

void Homeokinesis::setA(const Matrix& _A){
  assert(A.getM() == _A.getM() && A.getN() == _A.getN());
  A=_A;
}

Matrix Homeokinesis::getC(){
  return C;
}

void Homeokinesis::setC(const Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}
