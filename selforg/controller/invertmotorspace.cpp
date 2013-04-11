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

#include "invertmotorspace.h"
#include "regularisation.h"
using namespace matrix;
using namespace std;

InvertMotorSpace::InvertMotorSpace( int buffersize, double cInit /* = 0.1 */, bool someInternalParams /* = true*/)
  : InvertMotorController(buffersize, "InvertMotorSpace", "$Id$") {

  BNoiseGen = 0;
  x_buffer=0;
  y_buffer=0;

  this->cInit=cInit;
  this->someInternalParams=someInternalParams;

  addInspectableMatrix("A", &A, someInternalParams, "model matrix");
  addInspectableMatrix("C", &C, someInternalParams, "controller matrix");
  addInspectableMatrix("R", &R, someInternalParams, "linear Response matrix");

  addInspectableMatrix("H", &H, false, "controller bias");
  addInspectableMatrix("B", &B, false, "model bias");

};

InvertMotorSpace::~InvertMotorSpace() {
  if(BNoiseGen) delete BNoiseGen;
  if(x_buffer) delete[] x_buffer;
  if(y_buffer) delete[] y_buffer;
}

//double somevalue(double){ return 0.3;}

void InvertMotorSpace::init(int sensornumber, int motornumber, RandGen* randGen){
  assert(sensornumber>=motornumber);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak

  number_motors  = motornumber;
  number_sensors = sensornumber;
  A.set(number_sensors, number_motors);
  C.set(number_motors,  number_sensors);
  R.set(number_motors, number_motors);
  H.set(number_motors,  1);
  B.set(number_sensors, 1);

  BNoiseGen = new WhiteUniformNoise();
  BNoiseGen->init(number_sensors);

  A.toId(); // set a to identity matrix;
  // Matrix Init(number_sensors, number_motors);
  // Init.map(somevalue); // set to constant 0.3;
  //  A = A*0.5; // + Init; //

  C.toId(); // set a to identity matrix;
  C*= cInit;
  x_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
  }

  initialised = true;
}


/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void InvertMotorSpace::step(const sensor* x_, int number_sensors,
                            motor* y_, int number_motors){
  fillBuffersAndControl(x_, number_sensors, y_, number_motors);
  if(t>buffersize){
    // get effective motor values which caused present sensor values
    const Matrix& y_effective = y_buffer[(t-int(s4delay))%buffersize];
    const Matrix& x = x_buffer[t%buffersize];

    // learn controller with effective input/output
    learnController(x, x_smooth, max(int(s4delay)-1,0));

    // learn Model with actual sensors and with effective motors;
    learnModel(x, y_effective);
  }
  // update step counter
  t++;
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void InvertMotorSpace::stepNoLearning(const sensor* x, int number_sensors,
                                            motor*  y, int number_motors ){
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
};

void InvertMotorSpace::fillBuffersAndControl(const sensor* x_, int number_sensors,
                                              motor* y_, int number_motors){
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);

  Matrix x(number_sensors,1,x_);

  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);

  // averaging over the last s4avg values of x_buffer
  x_smooth = calculateSmoothValues(x_buffer, t < s4avg ? 1 : int(s4avg));

  // calculate controller values based on smoothed input values
  Matrix y = calculateControllerValues(x_smooth);

  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);

  // convert y to motor*
  y.convertToBuffer(y_, number_motors);
}

/// learn values H,C
// @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
void InvertMotorSpace::learnController(const Matrix& x, const Matrix& x_smooth, int delay){
  const Matrix& y = y_buffer[t%buffersize];     // current motors
  const Matrix& y_tm1 = y_buffer[(t-1-delay)%buffersize];     // motors, which correspond to x
  //  const Matrix& x_tm1 = x_buffer[(t-1)%buffersize];
  const Matrix xsi    = x -  (A * y_tm1 + B); // Modelling error in sensor-space
  // eta = A^-1 xsi (first shift in motor-space at current time)
  //  we use pseudoinverse U=A^T A -> eta = U^-1 A^T xsi
  const Matrix eta = (A.multTM()^-1) * ( (A^T) * xsi );

  // zeta = G' omega
  // RR^T zeta = mu = G'^{-1} eta

  const Matrix z = C * x_smooth + H;  // Todo: check whether x_smooth should be x
  const Matrix g_          = z.map(g);
  const Matrix g_prime     = Matrix::map2(g_s, z, eta); // new regularised version of g'
  const Matrix g_prime_inv = g_prime.map(one_over);
  R = C * A;
  const Matrix mu = eta.multrowwise(g_prime_inv); // G'^{-1} eta
  const Matrix zeta = (R.multMT()^-1) * mu; // (RR^T)^{-1} * mu

  const Matrix v = (R^T) * zeta;
  const Matrix omega = zeta.multrowwise(g_prime_inv);
  // correction of negled terms (LL^T)^-1 from learning rule, which drive the system out of saturation.
  // double f=1;
  //   for(int i=0; i< g_prime_inv.getM(); i++){
  //     f *= g_prime_inv.val(i,0);
  //   }
  //   const Matrix omega = eta * f;
  const Matrix alpha = A * v;
  const Matrix beta = omega.multrowwise(eta).multrowwise(y) * -2;

  const Matrix zmodel = R * y + H;
  Matrix C_update;
  Matrix H_update;

  // apply updates to H,C
  if(zetaupdate==0){
    // delta C = eps * (zeta * alpha^T + beta x^T)
    C_update = (zeta * (alpha^T) + beta * (x_smooth^T)) * epsC;
    H_update = (beta * epsC); // delta H = beta
  }else{
    // delta C = eps * (zeta * alpha^T + beta x^T + zeta x^T)
    C_update = (zeta * (alpha^T) + (beta + zeta*zetaupdate) * (x_smooth^T)) * epsC;
    H_update = (beta + zeta*zetaupdate) * epsC;     // delta H = beta + zeta
  }

  double error_factor = calcErrorFactor(v, (logaE & 1) !=0, (rootE & 1) !=0);
  C_update *= error_factor;
  H_update *= error_factor;

  C += C_update.map(squash);
  H += H_update.map(squash);
};

// normal delta rule
void InvertMotorSpace::learnModel(const Matrix& x, const Matrix& y){
  Matrix xsi = x -  (A * y + B);

  Matrix A_update;
  Matrix B_update;
  A_update=(( xsi*(y^T) ) * epsA );
  Matrix B_noise  = noiseMatrix(B.getM(),B.getN(), *BNoiseGen, -noiseB, noiseB); // noise for bias
  B_update=(  xsi * epsA + B_noise) * factorB;

  double error_factor = calcErrorFactor(xsi, (logaE & 2) != 0, (rootE & 2) != 0); //
  A_update *= error_factor;
  B_update *= error_factor;

  A += A_update.map(squash);
  B += B_update.map(squash);

};

/// calculate controller outputs
/// @param x_smooth smoothed sensors Matrix(number_channels,1)
Matrix InvertMotorSpace::calculateControllerValues(const Matrix& x_smooth){
  return (C*x_smooth+H).map(g);
};

/** stores the controller values to a given file. */
bool InvertMotorSpace::store(FILE* f) const {
  // save matrix values
  C.store(f);
  H.store(f);
  A.store(f);
  B.store(f);
  Configurable::print(f,"");
  return true;
}

/** loads the controller values from a given file. */
bool InvertMotorSpace::restore(FILE* f){
  // save matrix values
  C.restore(f);
  H.restore(f);
  A.restore(f);
  B.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}


list<Inspectable::ILayer> InvertMotorSpace::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  l+=ILayer("x","", number_sensors, 0, "Sensors");
  l+=ILayer("y","H", number_motors, 1,"Motors");
  l+=ILayer("xP","B", number_sensors, 2,"Prediction");
  return l;
}

list<Inspectable::IConnection> InvertMotorSpace::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  return l;
}

