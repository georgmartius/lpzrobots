/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.1  2007-04-20 12:30:42  martius
 *   multiple sat networks test
 *
 *
 ***************************************************************************/

#include "multisat.h"

using namespace matrix;
using namespace std;


Sat::Sat(MultiLayerFFNN* _net, double _eps){
  net=_net;
  eps=_eps;
  sigma=1;
  error=0;
  avg_error=1;
}


MultiSat::MultiSat( const MultiSatConf& _conf) 
  : AbstractController("MultiSat", "$Id: "), buffersize(_conf.buffersize), conf(_conf)
{
  initialised = false;
};


MultiSat::~MultiSat()
{
  if(x_buffer && y_buffer && xp_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] xp_buffer;
  }
  FOREACH(vector<Sat>, sats, s){
    if(s->net) delete s->net;
  }
}


void MultiSat::init(int sensornumber, int motornumber){

  number_motors  = motornumber;
  number_sensors = sensornumber;  

  x_buffer = new Matrix[buffersize];
  xp_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    xp_buffer[k].set(2*number_sensors,1);
    y_buffer[k].set(number_motors,1);
  }

  for(int i=0; i<2; i++){
    vector<Layer> layers;
    layers.push_back(Layer(conf.numberHidden, 0.5 , FeedForwardNN::tanh,FeedForwardNN::dtanh));
    layers.push_back(Layer(1,1));
    MultiLayerFFNN* net = new MultiLayerFFNN(1, layers); // learning rate is set to 1 and modulates each step  
    Sat sat(net, conf.eps0);
    sats.push_back(sat);
  }
  t=0;
  initialised = true;
}

// put new value in ring buffer
void MultiSat::putInBuffer(matrix::Matrix* buffer, const matrix::Matrix& vec, int delay){
  buffer[(t-delay)%buffersize] = vec;
}


/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void MultiSat::step(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{

  fillSensorBuffer(x_, number_sensors);
  conf.controller->step(x_, number_sensors, y_, number_motors);
  fillMotorBuffer(y_, number_motors);
  if(t>buffersize) {

    winner = compete();    
    FOREACH(vector<Sat>, sats, s){
      s->sigma=conf.lambda_comp;
    }
    sats[winner].sigma=1;
    sats[winner].eps *= conf.lambda_time;
    FOREACH(vector<Sat>, sats, s){
      s->net->learn(satInput, nomSatOutput, s->eps*s->sigma);
    }
    // winner should somehow influence control! e.g. control himself.
  }
  // update step counter
  t++;
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void MultiSat::stepNoLearning(const sensor* x, int number_sensors, motor*  y, int number_motors )
{
  fillSensorBuffer(x, number_sensors);
  conf.controller->stepNoLearning(x,number_sensors,y,number_motors);
  fillMotorBuffer(y, number_motors);
  // update step counter
  t++;
};


void MultiSat::fillSensorBuffer(const sensor* x_, int number_sensors)
{
  assert((unsigned)number_sensors == this->number_sensors);
  Matrix x(number_sensors,1,x_);
  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);  
  const Matrix& xp = calcDerivatives(x_buffer,0);
  putInBuffer(xp_buffer, xp);    
}

void MultiSat::fillMotorBuffer(const motor* y_, int number_motors)
{
  assert((unsigned)number_motors == this->number_motors);
  Matrix y(number_motors,1,y_);
  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);  
}

int MultiSat::compete()
{
  int winner=0;  
  const Matrix& x = x_buffer[t%buffersize];
  const Matrix& y = y_buffer[t%buffersize];

  const Matrix& x_tm1 = x_buffer[(t-1)%buffersize];
  const Matrix& xp_tm1 = xp_buffer[(t-1)%buffersize];
  const Matrix& y_tm1 = y_buffer[(t-1)%buffersize];

  nomSatOutput = x.above(y);
  satInput   = x_tm1.above(xp_tm1.above(y_tm1));
  // ask all networks to make there predictions on last timestep an compare with real world
  // and select network with minimum average error
  double minerror=10000;
  unsigned int i=0;
  FOREACH(vector<Sat>, sats, s){
    const Matrix& out = s->net->process(satInput);
    s->error = (nomSatOutput-out).multTM().val(0,0);
    s->avg_error = (1-1/conf.tau) * s->avg_error + (1/conf.tau) * s->error;    
    if(s->avg_error < minerror){
      minerror=s->avg_error;
      winner = i;
    }
    i++;
  }
    
  return winner;
}

  
Matrix MultiSat::calcDerivatives(const matrix::Matrix* buffer,int delay){  
  const Matrix& xt    = buffer[(t-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

void MultiSat::management(){
}

bool MultiSat::store(FILE* f) const {  
  // TODO: store!
  return true;
}

bool MultiSat::restore(FILE* f){
  // TODO: restore!
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

list<Inspectable::iparamkey> MultiSat::getInternalParamNames() const {
  list<iparamkey> keylist;
//   if(conf.someInternalParams){
//     keylist += store4x4AndDiagonalFieldNames(A, "A");
//     if(conf.useS) keylist += store4x4AndDiagonalFieldNames(S, "S");
//     keylist += store4x4AndDiagonalFieldNames(C, "C");
//     keylist += store4x4AndDiagonalFieldNames(R, "R");    
//   }else{
//     keylist += storeMatrixFieldNames(A, "A");
//     if(conf.useS) keylist += storeMatrixFieldNames(S, "S");
//     keylist += storeMatrixFieldNames(C, "C");
//     keylist += storeMatrixFieldNames(R, "R");
//     keylist += storeMatrixFieldNames(y_teaching, "yteach");
//   }
//   keylist += storeVectorFieldNames(H, "H");
//   keylist += storeVectorFieldNames(B, "B");
//   keylist += string("teacher");
//   //keylist += string("useTeaching");
//   //keylist += storeVectorFieldNames(y_teaching, "y_teaching");
//   keylist += string("epsA");
//   keylist += string("epsC");
//   keylist += string("xsi");
//   keylist += string("xsi_norm");
//   keylist += string("xsi_norm_avg");
//   keylist += string("pain");
  return keylist; 
}

list<Inspectable::iparamval> MultiSat::getInternalParams() const {
  list<iparamval> l;
//   if(conf.someInternalParams){
//     l += store4x4AndDiagonal(A);
//     if(conf.useS) l += store4x4AndDiagonal(S);
//     l += store4x4AndDiagonal(C);
//     l += store4x4AndDiagonal(R);    
//   }else{
//     l += A.convertToList();
//     if(conf.useS) l += S.convertToList();
//     l += C.convertToList();
//     l += R.convertToList();
//     l += y_teaching.convertToList();
//   }
//   l += H.convertToList();
//   l += B.convertToList();
//   l += teacher;
// 	//l += (useTeaching ? 1 : 0);
// //  l += x_teaching.convertToList();
//   l += epsA;
//   l += epsC;
//   l += xsi.elementSum();
//   l += xsi_norm;
//   l += xsi_norm_avg;
//   l += pain;
  return l;
}

list<Inspectable::ILayer> MultiSat::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
//   l+=ILayer("x","", number_sensors, 0, "Sensors");
//   l+=ILayer("y","H", number_motors, 1, "Motors");
//   l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> MultiSat::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
//   l+=IConnection("C", "x", "y");
//   l+=IConnection("A", "y", "xP");
//   if(conf.useS) l+=IConnection("S", "x", "xP"); // this is not quite true! it is x' x'' -> xp
  return l;
}
