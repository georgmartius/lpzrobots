/***************************************************************************
 *   Copyright (C) 2008 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *   Revision 1.2  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.1  2008/05/05 06:26:47  guettler
 *   tristateiafcontroller moved to ode_robots/simulations/iafsim
 *   for test purposes
 *
 *   Revision 1.1  2008/04/30 14:56:27  guettler
 *   neuron population support, other improvements
 *
 *   Revision 1.1  2008/04/25 10:31:50  guettler
 *   first version
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#include "tristateiafcontroller.h"

#include <selforg/matrix.h>
#include <selforg/controller_misc.h>

TristateIAFController::TristateIAFController(const TristateIAFControllerConf& conf)
    : AbstractController("TristateIAFController", "$Id$"), conf(conf), range(1.0) {
      addParameter("leakI", conf.leakI);
      addParameter("leakO", conf.leakO);
      addParameter("thresholdI", conf.thresholdI);
      addParameter("thresholdO", conf.thresholdO);
      addInspectableValue("[I]leak", conf.leakI);
      addInspectableValue("[O]leak", conf.leakO);
      addInspectableValue("[I]t",conf.thresholdI);
      addInspectableValue("[O]t",conf.thresholdO);
      addInspectableMatrix("[I]sum",&sumI);
      addInspectableMatrix("[O]sum",&sumO);
/*      addInspectableMatrix("[I]W",&wI);
      addInspectableMatrix("[O]W",&wO);*/
      addInspectableMatrix("[I]x",&xI);
      addInspectableMatrix("[O]x",&xO);
      initialised = false;
    }

  /// ABSTRACTCONTROLLER INTERFACE

  void TristateIAFController::init(int sensornumber, int motornumber, RandGen* randGen) {
    if(!randGen) randGen = new RandGen(); // this gives a small memory leak
    this->randG=randGen;
    // set dimensions
    sensorNumber=sensornumber;
    motorNumber=motornumber;
    wI.set(sensorNumber*conf.numberIAFNeuronsPerInput,sensorNumber*conf.numberIAFNeuronsPerInput);
    wO.set(sensorNumber*conf.numberIAFNeuronsPerInput,motorNumber*conf.numberIAFNeuronsPerOutput);
    sumI.set(1,sensorNumber*conf.numberIAFNeuronsPerInput);
    sumO.set(1,motorNumber*conf.numberIAFNeuronsPerOutput);
    tI.set(1,sensorNumber*conf.numberIAFNeuronsPerInput);
    tO.set(1,motornumber*conf.numberIAFNeuronsPerOutput);
    xI.set(1,sensorNumber*conf.numberIAFNeuronsPerInput);
    xO.set(1,sensorNumber*conf.numberIAFNeuronsPerInput);

    // set init values
    wI.toMapP(randGen,random_minusone_to_one) * conf.wIInitScale; // scale random
    wO.toMapP(randGen,random_minusone_to_one) * conf.wOInitScale; // scale random
    // set threshold values
    tI.toSum((*conf.thresholdI)*sqrt((double)conf.numberIAFNeuronsPerInput));
    tO.toSum((*conf.thresholdO)*sqrt((double)conf.numberIAFNeuronsPerOutput));
    initialised=true;
  }


  void TristateIAFController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
    forwardStep(sensors,sensornumber,motors,motornumber);
  }

  void TristateIAFController::stepNoLearning(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
    forwardStep(sensors,sensornumber,motors,motornumber);
  }

  /// CONFIGURABLE INTERFACE
  bool TristateIAFController::setParam(const paramkey& key, paramval val, bool traverseChildren){
    if(key=="thresholdI") {
      (*conf.thresholdI)=val;
      tI.toZero().toSum(val*sqrt((double)conf.numberIAFNeuronsPerInput));
    } else if (key=="thresholdO") {
      (*conf.thresholdO)=val;
      tO.toZero().toSum(val*sqrt((double)conf.numberIAFNeuronsPerOutput));
    } else
      return Configurable::setParam(key,val);
    return true;
  }


  void TristateIAFController::forwardStep(const sensor* sensors, int number_sensors, motor* motors, int number_motors) {
    assert ((sensorNumber==number_sensors) && (motorNumber==number_motors));

    // construct input vector for input layer neurons
    matrix::Matrix input(1,sensorNumber,sensors);
    xI.set(1,sensorNumber,sensors);
    // generate discrete fire events (-1 or 1)
    xI.toMapP(randG,toTristateWithProbability);
    for (int i=1;i<conf.numberIAFNeuronsPerInput;i++) {
      xI.addColumns(sensorNumber,input.mapP(randG,toTristateWithProbability));
    }

    // calculate input layer
    xI*=wI; // calculate x * w
    sumI+=xI; // add the result to existing sum
    sumI.toMapP(conf.leakI,dampToZero); // add leak

    // if above threshold, fire (=1.0), otherwise not (=-1)
    xO.copy(sumI);
    xO.toMap2(toTristateWithThreshold,tI);

    // set all sums to zero of the neurons who have fired
    sumI.toMap2(toZeroIfFired,xO);

    // calculate output layer
    xO*=wO; // calculate x * w
    sumO+=xO; // add the result to existing sum
    sumO.toMapP(conf.leakI,dampToZero); // add leak

    // if above threshold, fire (=1.0), otherwise not (=0)
    matrix::Matrix y = matrix::Matrix::map2(toTristateWithThreshold,sumO,tO);

    // set all sums to zero of the neurons who have fired
    sumO.toMap2(toZeroIfFired,y);

    // calculate a summed rate of the output of the output population neurons
    matrix::Matrix m(1,motorNumber);
    for (int i=0;i<conf.numberIAFNeuronsPerOutput;i++) {
      m.val(0,0)+=y.val(0,2*i);
      m.val(0,1)+=y.val(0,2*i+1);
    }
    m*=(1/(double)conf.numberIAFNeuronsPerOutput);
    // assign calculated output to new motor values
    m.convertToBuffer(motors, motorNumber); // convert y to motor*
  }

