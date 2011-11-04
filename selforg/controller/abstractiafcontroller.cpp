/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
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
#include "abstractiafcontroller.h"

#include <selforg/matrix.h>
#include "controller_misc.h"

AbstractIAFController::AbstractIAFController(const AbstractIAFControllerConf& conf)
    : AbstractController("AbstractIAFController", "$Id$"), conf(conf), range(1.0) {
      addParameter("leaki", conf.leakI);
      addParameter("leako", conf.leakO);
      addParameter("thresholdi", conf.thresholdI);
      addParameter("thresholdo", conf.thresholdO);
      addParameter("wiinitscale", conf.wIInitScale);
      addParameter("wiinitscale", conf.wOInitScale);
      addParameter("niafperinput", conf.numberIAFNeuronsPerInput);
      addParameter("niafperoutput", conf.numberIAFNeuronsPerOutput);
//       addInspectableValue("[I]leak", conf.leakI);
//       addInspectableValue("[O]leak", conf.leakO);
//       addInspectableValue("[I]t",conf.thresholdI);
//       addInspectableValue("[O]t",conf.thresholdO);
      addInspectableMatrix("[I]sum",&sumI);
      addInspectableMatrix("[O]sum",&sumO);
//       addInspectableMatrix("[I]W",&wI);
//       addInspectableMatrix("[O]W",&wO);
      addInspectableMatrix("[I]x",&xI);
      addInspectableMatrix("[O]x",&xO);
      initialised = false;
    }


  /// ABSTRACTCONTROLLER INTERFACE

  void AbstractIAFController::init(int sensornumber, int motornumber, RandGen* randGen) {
    if(!randGen) randGen = new RandGen(); // this gives a small memory leak
    this->randG=randGen;
    // set dimensions
    sensorNumber=sensornumber;
    motorNumber=motornumber;
    initMatrices();
    initialised=true;
  }


  void AbstractIAFController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
    forwardStep(sensors,sensornumber,motors,motornumber);
  }

  void AbstractIAFController::stepNoLearning(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
    forwardStep(sensors,sensornumber,motors,motornumber);
  }

  /// CONFIGURABLE INTERFACE
  void AbstractIAFController::notifyOnChange(const paramkey& key){
    if(key=="thresholdi") {
      tI.toZero().toSum((*conf.thresholdI)*sqrt(*conf.numberIAFNeuronsPerInput));
    } else if (key=="thresholdo") {
      tO.toZero().toSum((*conf.thresholdO)*sqrt(*conf.numberIAFNeuronsPerOutput));
    } else if(key=="wiinitscale") {
        wI.toId().toMult(*conf.wIInitScale); // scale
    } else if(key=="woinitscale") {
      wI.toId().toMult(*conf.wOInitScale); // scale
    } else if(key=="niafperinput") {
      initMatrices();
    } else if(key=="niafperoutput") {
      initMatrices();
    }
  }


  void AbstractIAFController::forwardStep(const sensor* sensors, int number_sensors, motor* motors, int number_motors) {
    assert ((sensorNumber==number_sensors) && (motorNumber==number_motors));

    // construct input vector for input layer neurons
    matrix::Matrix input(1,sensorNumber,sensors);
    matrix::Matrix negInput(1,sensorNumber,sensors);
    input.toSum(0.1);
    negInput.toSum(0.1);
    negInput.toMult(-1.);
    xI.set(1,sensorNumber,sensors);
    xI.toSum(0.1);
    // generate discrete fire events (0 or 1)
    xI.toMapP(randG,toDualStateWithProbability); // positive neurons
    xI.addColumns(sensorNumber,negInput.mapP(randG,toDualStateWithProbability)); // neg
    for (int i=1;i<*conf.numberIAFNeuronsPerInput;i++) {
      xI.addColumns(sensorNumber,input.mapP(randG,toDualStateWithProbability)); // positive
      xI.addColumns(sensorNumber,negInput.mapP(randG,toDualStateWithProbability)); // neg
    }

    // calculate input layer
    xI*=wI; // calculate x * w
    sumI+=xI; // add the result to existing sum
    sumI.toSum((*conf.leakI)*-1.); // add leak
    sumI.toMapP(conf.restingPotential,lowercutof);

    // if above threshold, fire (=1.0), otherwise not (=0)
    xO.copy(sumI);
    xO.toMap2(toDualStateWithThreshold,tI);

    // set all sums to zero of the neurons who have fired
    sumI.toMap2P(conf.restingPotential,toValueIfFired,xO);

    // calculate output layer
    xO*=wO; // calculate x * w
    sumO+=xO; // add the result to existing sum
    sumO.toSum((*conf.leakO)*-1.); // add leak
    sumO.toMapP(conf.restingPotential,lowercutof);

    // if above threshold, fire (=1.0), otherwise not (=0)
    matrix::Matrix y = matrix::Matrix::map2(toDualStateWithThreshold,sumO,tO);

    // set all sums to zero of the neurons who have fired
    sumO.toMap2P(conf.restingPotential,toValueIfFired,y);

    // calculate a summed rate of the output of the output population neurons
    matrix::Matrix m(1,motorNumber);
    for (int i=0;i<*conf.numberIAFNeuronsPerOutput;i++) {
      for (int j=0; j<motorNumber;j++) {
        m.val(0,j)+=y.val(0,motorNumber*i+j); // positive neurons
        m.val(0,j)-=y.val(0,(matrix::I)(motorNumber*i+j+motorNumber*(*conf.numberIAFNeuronsPerOutput))); // neg
      }
    }
    m*=(1./(*conf.numberIAFNeuronsPerOutput));
    // assign calculated output to new motor values
    m.convertToBuffer(motors, motorNumber); // convert y to motor*
  }

void AbstractIAFController::initMatrices() {
  wI.set((matrix::I)(sensorNumber*(*conf.numberIAFNeuronsPerInput)*2),
         (matrix::I)(sensorNumber*(*conf.numberIAFNeuronsPerInput)*2));
  wO.set((matrix::I)(sensorNumber*(*conf.numberIAFNeuronsPerInput)*2),
         (matrix::I)(motorNumber*(*conf.numberIAFNeuronsPerOutput)*2));
  sumI.set(1,(matrix::I)(sensorNumber*(*conf.numberIAFNeuronsPerInput)*2));
  sumO.set(1,(matrix::I)(motorNumber*(*conf.numberIAFNeuronsPerOutput)*2));
  tI.set(1,(matrix::I)(sensorNumber*(*conf.numberIAFNeuronsPerInput)*2));
  tO.set(1,(matrix::I)(motorNumber*(*conf.numberIAFNeuronsPerOutput)*2));
  xI.set(1,(matrix::I)(sensorNumber*(*conf.numberIAFNeuronsPerInput)*2));
  xO.set(1,(matrix::I)(sensorNumber*(*conf.numberIAFNeuronsPerInput)*2));

    // set init values
  wI.toId().toMult(*conf.wIInitScale); // scale
  wO.toId().toMult(*conf.wOInitScale); // scale
    // set threshold values
  tI.toSum((*conf.thresholdI)*sqrt(*conf.numberIAFNeuronsPerInput));
  tO.toSum((*conf.thresholdO)*sqrt(*conf.numberIAFNeuronsPerOutput));
}
