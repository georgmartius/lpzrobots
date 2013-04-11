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
#ifndef __PROACTIVE_H
#define __PROACTIVE_H

#include "invertmotornstep.h"
#include "onelayerffnn.h"

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>

/**
 * robot controller for self-organized behaviour using pro-active elements
 * and it is based in InvertMotorNStep.
 */
class ProActive : public InvertMotorNStep {

public:
  /** @param numberNonContext number of input channels that are not considered as context inputs
      (e.g.\ infrared).
      @param tau time window for temporal correlation
      @param conf configuration \ref InvertMotorNStepConf
   */
  ProActive( unsigned int numberNonContext, unsigned int tau, const InvertMotorNStepConf& conf = getDefaultConf())
    : InvertMotorNStep(conf), synDyn(0.1, 0.0) {
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

  virtual ~ProActive(){
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
    H_delta.set(motornumber,1);
    H_delta_net.set(motornumber,1);

    H_orig.set(motornumber,1);
    H_context.set(motornumber,1);
    h_context_norm_avg=0.01;

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
      keylist += storeMatrixFieldNames(synDyn.getWeights(), "HDW");
    }
    keylist += storeMatrixFieldNames(synDyn.getBias(), "HDB");
    keylist += storeMatrixFieldNames(H_delta, "HD");
    keylist += storeMatrixFieldNames(H_delta_net, "HDN");
    keylist += storeVectorFieldNames(syndyn_buffer[0], "hinput");
    keylist += storeVectorFieldNames(H_orig, "H_orig");
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
    l += H_delta.convertToList();
    l += H_delta_net.convertToList();
    l += syndyn_buffer[(t+buffersize-tau)%buffersize].convertToList();
    l += H_orig.convertToList();

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
    const matrix::Matrix& y = y_buffer[t% buffersize];
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
  }

  /// overloaded version which includes the synaptic dynamics network
  /// updates the matrix C and H
  virtual void updateCandH(const matrix::Matrix& C_update, const matrix::Matrix& H_update, double squashSize){
    bufferSynDynInput();

    // the C-update stays as it is.
    C += C_update.mapP(&squashSize, squashP);
//    // the H-update is also applied, but there is another term see below
//    H += H_update.mapP(&squashSize, squashP);

    // for H-update we use an additional network, which learns the update
    // in case of paint.
    //  We use delayed y and delayed x values so that the current
    //   error can be assoziated with previous motor commands.
    matrix::Matrix C_update_delay(C.getM(), C.getN());
    matrix::Matrix H_update_delay(H.getM(), H.getN());
    matrix::Matrix hinput_delay = syndyn_buffer[(t - tau) % buffersize];
    if(pain!=0){
      // pain learning
      calcCandHUpdates(C_update_delay, H_update_delay, tau);
      double squashS=1;
      H_delta = H_update_delay.mapP(&squashS, squashP);
      synDyn.learn(hinput_delay, H_delta);
    }

    // apply the output of the network to H (use current inputs)
    const matrix::Matrix& hinput = syndyn_buffer[t % buffersize];
    H_delta_net = synDyn.process(hinput);
    synDyn.damp(dampH);

    H_context = H_delta_net * epsH;
    double h_context_norm = calcMatrixNorm(H_context);
    double decay=0.0001;
    h_context_norm_avg = h_context_norm_avg*(1-decay) + h_context_norm*decay;


    if(h_context_norm < h_context_norm_avg * 5){
      // normal H dynamic
      H_orig += H_update.mapP(&squashSize, squashP);
    }else{
      cerr << "pla";
    }

    H = H_orig + H_context;



  }

protected:
  OneLayerFFNN synDyn;

  matrix::Matrix* syndyn_buffer;
  matrix::Matrix x_context;
  matrix::Matrix H_context;
  double h_context_norm_avg;
  matrix::Matrix H_orig;

  unsigned int tau;
  unsigned int numberNonContext;
  unsigned int number_all_sensors;
  double dampH;
  double epsH;

  matrix::Matrix H_delta;
  matrix::Matrix H_delta_net;

};

#endif
