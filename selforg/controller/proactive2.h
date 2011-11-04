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
#ifndef __PROACTIVE2_H
#define __PROACTIVE2_H

#include "invertmotornstep.h"
#include "onelayerffnn.h"

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>

/**
 * robot controller for self-organized behaviour using pro-active elements
 * and it is based in InvertMotorNStep. 
 */
class ProActive2 : public InvertMotorNStep {

public:
  /** @param numberNonContext number of input channels that are not considered as context inputs 
      (e.g.\ infrared)
      @param tau time window for temporal correlation
      @param conf configuration \ref InvertMotorNStepConf
   */
  ProActive2( unsigned int numberNonContext, unsigned int tau, const InvertMotorNStepConf& conf = getDefaultConf())
    : InvertMotorNStep(conf), synDyn(0.1) {
    // prepare name;
    Configurable::insertCVSInfo(name, "$RCSfile$", "$Revision$");
    addConfigurable(synDyn);
    addParameterDef("epsH", &epsH, 0.5,0,2,"learning rate for bias modulation");
    addParameterDef("tau", &this->tau, tau, 0,1,"weigting term");
    addParameterDef("dampH", &dampH, 0.001, 0,0.1,"damping for bias");

    assert(tau < buffersize);
    this->tau = tau;
    this->numberNonContext = numberNonContext;
  }

  virtual ~ProActive2(){
    if (syndyn_buffer){
      delete[] syndyn_buffer;
    } 
  }

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){ 
    assert( numberNonContext <= (unsigned int) sensornumber);
    number_all_sensors = sensornumber;
    InvertMotorNStep::init(numberNonContext, motornumber);

    int synDynInputNumber = (sensornumber - numberNonContext); // + motornumber
    synDyn.init(synDynInputNumber, motornumber);
    syndyn_buffer = new matrix::Matrix[buffersize];

    for (unsigned int k = 0; k < buffersize; k++) {
      syndyn_buffer[k].set(synDynInputNumber,1);
    }    
    
    xsi_pred.set(numberNonContext,1);
    xsi_orig.set(numberNonContext,1);

  }

  virtual void step(const sensor* x_, int number_sensors, motor* y_, int number_motors){
    // call InvertMotorNStep just with the non-context sensors    
    // column vector with context sensor values
    x_context.set(number_all_sensors - numberNonContext, 1, x_ + numberNonContext); 
    InvertMotorNStep::step(x_, numberNonContext, y_, number_motors);    
  }

  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* x_, int number_sensors, 
			      motor* y_, int number_motors){
    x_context.set(number_all_sensors - numberNonContext, 1, x_ + numberNonContext); // column vector with context sensor values
    // time is increased 
    InvertMotorNStep::stepNoLearning(x_, numberNonContext, y_, number_motors); 
    t--;
    bufferSynDynInput();    
    t++;
  }

  virtual list<iparamkey> getInternalParamNames() const {
    list<iparamkey> keylist = InvertMotorNStep::getInternalParamNames();
    if(conf.someInternalParams){
      keylist += store4x4AndDiagonalFieldNames(synDyn.getWeights(), "HDW");
    }else{
      keylist += storematrix::MatrixFieldNames(synDyn.getWeights(), "HDW");
    }
    keylist += storematrix::MatrixFieldNames(synDyn.getBias(), "HDB");
    keylist += storeVectorFieldNames(xsi_pred, "xsi_orig");
    keylist += storeVectorFieldNames(xsi_pred, "xsi_pred");
    return keylist;
  }

  virtual list<iparamval> getInternalParams() const {
    list<iparamval> l = InvertMotorNStep::getInternalParams();
    if(conf.someInternalParams){
      l += store4x4AndDiagonal(synDyn.getWeights());
    }else{
      l += synDyn.getWeights().convertToList();
    }
    l += synDyn.getBias().convertToList();
    l += xsi_orig.convertToList();
    l += xsi_pred.convertToList();
    return l;
  }
      
  static InvertMotorNStepConf getDefaultConf(){
    InvertMotorNStepConf c;
    c.buffersize = 10;
    c.cInit = 0.1;
    c.useS  = true;
    c.someInternalParams = true;
    return c;
  }

protected:

  virtual void bufferSynDynInput(){
    //const matrix::Matrix z = C * x_smooth + H;
    //    const matrix::Matrix& y = y_buffer[t% buffersize];
    // first the postsynaptic potential and then the context sensors and then the non-context sensors
    //    matrix::Matrix hinput = z.above(x_context*y.val(0,0)); // column vector with z above x_context
    //    const matrix::Matrix& hinput = z.above(x_context); // column vector with z above x_context
    const matrix::Matrix& hinput = x_context; 
    // put new sensor vector in ring buffer sensor_buffer (all sensors)
    putInBuffer(syndyn_buffer, hinput);
  }

  /// calculates xsi for the current time step using the delayed y values
  //  overloaded version which incorporates time smoothing
  virtual void calcXsi(int delay){ 
    InvertMotorNStep::calcXsi(delay);
    xsi_orig = xsi;
    const matrix::Matrix& y = y_buffer[(t - 1 - delay) % buffersize];
    if(x_context.val(0,0) *  y.val(0,0) > 0.1){
      xsi_pred.val(0,0) = x_context.val(0,0) *  y.val(0,0)  * 0.2 ;
    }else{
      xsi_pred.val(0,0)=0;      
    }    
    xsi += xsi_pred;
  }
  
protected:
  OneLayerFFNN synDyn;

  matrix::Matrix* syndyn_buffer;
  matrix::Matrix x_context;
  matrix::Matrix xsi_pred;
  matrix::Matrix xsi_orig;
  paramkey* internkeylist;
  
  unsigned int tau;
  unsigned int numberNonContext;
  unsigned int number_all_sensors;
  double dampH; 
  double epsH; 

};

#endif
