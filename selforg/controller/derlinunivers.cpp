/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <math.h>
#include "derlinunivers.h"
#include "regularisation.h"

using namespace std;
using namespace matrix;

DerLinUnivers::DerLinUnivers(const DerLinUniversConf& conf)
  : AbstractController("derlinunivers", "$Id$"), conf(conf) {
  t=0;
  addParameterDef("eps",    &eps,    1);
  addParameterDef("epsM",   &epsM,   0.1);
  addParameterDef("epsV",   &epsV,   0.1);
  addParameterDef("lambda", &lambda, 0.1);
  addParameterDef("s4avg",  &s4avg,  1);
  addParameterDef("s4del",  &s4del,  1);
  addParameterDef("Enorm",  &Enorm,  0);
  addParameterDef("epsDyn", &epsDyn, 0.001);

  number_sensors=0;
  number_motors=0;
  initialised=false;
};


DerLinUnivers::~DerLinUnivers(){
  if(x_buffer && y_buffer && v_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] v_buffer;
  }
}

void DerLinUnivers::init(int sensornumber, int motornumber, RandGen* randGen){
  number_sensors=sensornumber;
  number_motors=motornumber;
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  assert(conf.net);
  assert(conf.net->getLayerNum()>1);
  // if no motorlayer is given, we use the second last (n-1)
  if(conf.motorlayer == -1)
    conf.motorlayer = conf.net->getLayerNum()-2;
  assert(conf.motorlayer < (int)conf.net->getLayerNum());

  // the number of neurons in the motorlayer at initialisation time
  //  is considered to be the number of additional neurons in this layer.
  conf.net->getLayer(conf.motorlayer).size += number_motors;
  conf.net->setSomeInternalParams(conf.someInternalParams);
  conf.net->init(number_sensors, number_sensors, conf.init);

  v.set(number_sensors,1);
  xsi_smooth.set(number_sensors,1);
  J.set(number_sensors,number_sensors);

  x_buffer = new Matrix[conf.buffersize];
  y_buffer = new Matrix[conf.buffersize];
  v_buffer = new Matrix[conf.buffersize];
  for (unsigned int k = 0; k < conf.buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
    v_buffer[k].set(number_sensors,1);
  }

  initialised = true;
};



void DerLinUnivers::step(const sensor* x_, int sensornumber,
                               motor* y_, int motornumber) {
  fillBuffersAndControl(x_, number_sensors, y_, number_motors);
  if(t > conf.buffersize){
    // int delay = max(int(s4del)-1,0);    // TODO
    const Matrix& x     = x_buffer[t%conf.buffersize];
    const Matrix& x_tm1 = x_buffer[(t-1)%conf.buffersize];

    // Updating the model parameters
    Matrix        xsi;

    xsi = x  - conf.net->process(x_tm1); // calc xsi with original input
    // calculate model update
    NetUpdate modelupdate = conf.net->weightIncrementBlocked(xsi, conf.motorlayer,
                                                             0, number_motors);

// Now we calculate the  update of the controller prameters with liner activation functions of the neural network.
//As a first step we replace  the activation functions with linear ones:
    std::vector<ActivationFunction> oldactfunlist = conf.net-> setActivationFunction(FeedForwardNN::linear);
    // calculation step:



    const Matrix& xp  = conf.net->process(x_tm1); // for direct v
    xsi = x  - xp;
    J   = conf.net->response(Matrix());
    v = (J^T)*((J.multMT()+(J^0)*lambda)^(-1))*xsi; // direct v calculation
    v.toMapP(5,clip);


    // calculate controller update
    xsi = x  - conf.net->process(x_tm1 + v); // calc xsi with new v
    // xsi_smooth = xsi_smooth*(1-0.1) + xsi*0.1;
    NetUpdate cupdate2 = conf.net->weightIncrementBlocked(xsi, conf.motorlayer,
                                                          number_motors, -1);
    conf.net->process(x_tm1); //  activate again with original input!
    NetUpdate cupdate1 = conf.net->weightIncrementBlocked(xsi, conf.motorlayer,
                                                          number_motors, -1);
 // set actfun back
    conf.net->setActivationFunctions(oldactfunlist);

    //Now we have to do the gradient descent step on the error caused by the nonlinearities of the true activeation funcitons:



    NetUpdate update(modelupdate.weights.size(), modelupdate.bias.size(),
                     modelupdate.other.size());

    double error_factor = calcErrorFactor(v, (int)Enorm);
    for(unsigned int i=0; i<update.weights.size(); i++){
      update.weights[i] = cupdate2.weights[i] - cupdate1.weights[i];
      update.weights[i] *= eps * error_factor;
      if((int)i>conf.motorlayer) update.weights[i] *= 0;
      update.weights[i] += modelupdate.weights[i]*epsM;
      double n = matrixNorm1(update.weights[i]);
      update.weights[i].toMapP(&conf.squashsize, squash);
      n -= matrixNorm1(update.weights[i]);
      if(n > 0.001 && epsDyn>0){
        eps *= 0.95;
        update.weights[i] *= 0;
      }
    }
    for(unsigned int i=0; i<update.bias.size(); i++){
      update.bias[i] = cupdate2.bias[i] - cupdate1.bias[i];
      update.bias[i] *= eps * error_factor;
      if((int)i>conf.motorlayer) update.bias[i] *= 0;
      update.bias[i] += modelupdate.bias[i]*epsM;
      update.bias[i].toMapP(&conf.squashsize, squash);
    }
    for(unsigned int i=0; i<update.other.size(); i++){
      update.other[i] = cupdate2.other[i] - cupdate1.other[i];
      update.other[i] *= eps * error_factor;
      update.other[i] += modelupdate.other[i]*epsM;
      update.other[i].toMapP(&conf.squashsize, squash);
    }
    conf.net->updateWeights(update);
    // conf.net->getWeights(1).toId();

    eps += epsDyn;
  }
  // update step counter
  t++;
};

void DerLinUnivers::stepNoLearning(const sensor* x, int number_sensors,
                                         motor* y, int number_motors) {
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
};

void DerLinUnivers::fillBuffersAndControl(const sensor* x_, int number_sensors,
                                                motor* y_, int number_motors){
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);

  Matrix x(number_sensors,1,x_);

  putInBuffer(x_buffer, x);  // put new input vector in ring buffer x_buffer
  // averaging over the last s4avg values of x_buffer
  const Matrix& x_smooth = calculateSmoothValues(x_buffer, t < s4avg ? 1 : int(max(1.0,s4avg)));

  // calculate controller values based on smoothed input values
  Matrix y = calculateControllerValues(x_smooth);
  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);
  // convert y to motor*
  y.convertToBuffer(y_, number_motors);
}


// calculate controller outputs
Matrix DerLinUnivers::calculateControllerValues(const matrix::Matrix& x){
  /*const Matrix& xp_tp1 = */
  conf.net->process(x);
  return conf.net->getLayerOutput(conf.motorlayer).rows(0, number_motors-1);
}

/// calculate time-smoothed values
Matrix DerLinUnivers::calculateSmoothValues(const Matrix* buffer,
                                                  int number_steps_for_averaging_){
  // number_steps_for_averaging_ must not be larger than buffersize
  assert ((unsigned)number_steps_for_averaging_ <= conf.buffersize);
  matrix::Matrix result(buffer[t % conf.buffersize]);
  for (int k = 1; k < number_steps_for_averaging_; k++) {
    result += buffer[(t - k + conf.buffersize) % conf.buffersize];
  }
  if(number_steps_for_averaging_ == 1) return result;
  else return result * (1/((double) (number_steps_for_averaging_)));
};


double DerLinUnivers::calcErrorFactor(const matrix::Matrix& e, int Enorm){
  double error_factor = 1;
  if (Enorm==2){   // using logarithmic error E=ln(v^T*v)
    error_factor= 1/(e.multTM().val(0,0)+0.000001)*0.01; // factor 1/100 for normalising (empirically)
  }else if (Enorm==1){  // using root error E=(v^T*v)^(1/2)
    error_factor= 1/sqrt(e.multTM().val(0,0)+0.000001)*0.1; // factor 1/10 for normalising (empirically)
  }
  return error_factor;
}


bool DerLinUnivers::store(FILE* f) const {
  if(conf.net->store(f)){
    Configurable::print(f,"");
    return true;
  } else return false;
}

bool DerLinUnivers::restore(FILE* f) {
  if(conf.net->restore(f)){
    Configurable::parse(f);
    t=0; // set time to zero to ensure proper filling of buffers
    return true;
  } else return false;
}



std::list<Inspectable::iparamkey> DerLinUnivers::getInternalParamNames() const  {
  list<Inspectable::iparamkey> l;
  if(conf.someInternalParams)
    l+= store4x4AndDiagonalFieldNames(J, "J");
  else
    l+= storeMatrixFieldNames(J, "J");
  l+= storeVectorFieldNames(v, "v");
  l+= string("eps");
  l+= conf.net->getInternalParamNames();
  return l;
}

std::list<Inspectable::iparamval> DerLinUnivers::getInternalParams() const {
  list<Inspectable::iparamval> l;
  if(conf.someInternalParams)
    l += store4x4AndDiagonal(J);
  else
    l += J.convertToList();
  l += v.convertToList();
  l += eps;
  l += conf.net->getInternalParams();
  return l;
}

Inspectable::ilayerlist DerLinUnivers::getStructuralLayers() const{
  return conf.net->getStructuralLayers();
}

Inspectable::iconnectionlist DerLinUnivers::getStructuralConnections() const{
  return conf.net->getStructuralConnections();
}
