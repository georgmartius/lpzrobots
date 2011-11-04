/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
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
#ifndef __ABSTRACTIAFCONTROLLER_H
#define __ABSTRACTIAFCONTROLLER_H

#include "abstractcontroller.h"

#include <selforg/matrix.h>
#include "controller_misc.h"
#include <selforg/configurable.h>

typedef struct AbstractIAFControllerConf {
  AbstractIAFControllerConf() {
    thresholdI=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    thresholdO=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    leakI=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    leakO=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    restingPotential=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    wIInitScale=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    wOInitScale=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    numberIAFNeuronsPerInput=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    numberIAFNeuronsPerOutput=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
  }
  ~AbstractIAFControllerConf() {
    /*  if(thresholdI) free(thresholdI);
    if(thresholdO) free(thresholdO);
    if(leakI) free(leakI);
    if(leakO) free(leakO);*/
  }
  Configurable::paramval* numberIAFNeuronsPerInput;  ///< simulate a population if >1
  Configurable::paramval* numberIAFNeuronsPerOutput; ///< simulate a population if >1
  Configurable::paramval* wIInitScale;            ///< scaling factor of weights, initialized random
  Configurable::paramval* wOInitScale;            ///< between [-1*wInitScale,1*wInitScale]
  Configurable::paramval* thresholdI;
  Configurable::paramval* thresholdO;
  Configurable::paramval* leakI;
  Configurable::paramval* leakO;
  Configurable::paramval* restingPotential;
  //Configurable::paramval* test;
} AbstractIAFControllerConf;


/**
 * Abstract class (interface) for robot controller that uses an integrate
 * and fire neuron model
 *
 * Implements (assumes) that the sensor values are only 0 or 1.
 * Implements standard configureable interface for some useful parameters
 * like leakI and leakO (input and output layer)
 */
class AbstractIAFController : public AbstractController {

public:
  AbstractIAFController(const AbstractIAFControllerConf& conf = getDefaultConf());

  virtual ~AbstractIAFController() {}


  static AbstractIAFControllerConf getDefaultConf(){
    AbstractIAFControllerConf c;
    *c.numberIAFNeuronsPerInput  = 10;
    *c.numberIAFNeuronsPerOutput = 10;
    *c.wIInitScale= 0.5;
    *c.wOInitScale= 0.5;
    *c.thresholdI=0.5;
    *c.thresholdO=0.5;
    *c.leakI=0.01;
    *c.leakO=0.01;
    *c.restingPotential=0.0;

    return c;
  }


  /// ABSTRACTCONTROLLER INTERFACE

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual int getSensorNumber() const { return sensorNumber; }

  virtual int getMotorNumber() const { return motorNumber; }

  virtual void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber);

  virtual void stepNoLearning(const sensor* sensors, int sensornumber, motor* motors, int motornumber);

  /// STORABLE INTERFACE

  virtual bool store(FILE* f) const { return true; }

  virtual bool restore(FILE* f) { return true; }

  /// CONFIGURABLE INTERFACE
  virtual void notifyOnChange(const paramkey& key);

protected:
  AbstractIAFControllerConf conf;
  RandGen* randG;
  bool initialised;
  int sensorNumber;
  int motorNumber;
  double range;
  matrix::Matrix xI; // matrix with input for the input layer neurons
  matrix::Matrix xO; // matrix with input for the output layer neurons
  matrix::Matrix wI; // matrix with weights of the input layer neurons, incl.leak
  matrix::Matrix wO; // matrix with weights of the output layer neurons, incl.leak
  matrix::Matrix sumI; // matrix with current sums of the input layer neurons
  matrix::Matrix sumO; // matrix with current sums of the output layer neurons
  matrix::Matrix tI; // matrix with threshold value of the input layer neurons
  matrix::Matrix tO; // matrix with threshold value of the output layer neurons

  /**
   * makes a forward step (without any learning)
   */
  virtual void forwardStep(const sensor* sensors, int number_sensors, motor* motors, int number_motors);

  /**
   * inits the internal used matrices
   * If you change something on dimensions, you should call
   * this function.
   * @note that all learned weights are lost.
   */
  void initMatrices();




  /// returns -1 if probability is to low, otherwise 1 for mapP
  static double toTristateWithProbability(void* r,double x) {
    RandGen* g = (RandGen*) r;
    if (!g) return 0.;
    double rand = g->rand();
    return x < -rand ? -1. : (x < rand ? 0. : 1.);
  }

  /// returns -1 if below -threshold, 0 if above -threshold
  /// and threshold, otherwise 1, for map2
  static double toTristateWithThreshold(double x, double threshold){
    return x < -threshold ? -1. : (x < threshold ? 0. : 1.);
  }

  /// damps the value, if <0, damp value is added
  /// if >0, damp value is subtracted
  /// and threshold, otherwise 1, for map2
  static double dampToZero(void* r, double x){
    double damp = *(double*)r;
    return x < -damp ? x+damp : (x > damp ? x-damp : 0.);
  }

  // returns 0 if fired==1 (-1 or 1), otherwise x
  static double toZeroIfFired(double x, double fired) {
    return (fired==1 || fired==-1) ? 0 : x ;
  }

    // returns value if fired==1 (-1 or 1), otherwise x
  static double toValueIfFired(void* r,double x, double fired) {
    double value = *(double*)r;
    return (fired==1 || fired==-1) ? value : x ;
  }


  /// returns 0 if probability is to low, otherwise 1 for mapP
  static double toDualStateWithProbability(void* r,double x) {
    RandGen* g = (RandGen*) r;
    if (!g) return 0.;
    double rand = g->rand();
    return x < rand ? 0. : 1.;
  }

    /// returns 0 if below threshold, otherwise 1, for map2
  static double toDualStateWithThreshold(double x, double threshold){
    return x < threshold ? 0. : 1.;
  }




};


#endif
