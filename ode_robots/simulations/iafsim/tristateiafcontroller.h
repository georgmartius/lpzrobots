/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
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
 *   Revision 1.3  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.2  2008/05/05 06:31:59  guettler
 *   header defines corrected
 *
 *   Revision 1.1  2008/05/05 06:26:47  guettler
 *   tristateiafcontroller moved to ode_robots/simulations/iafsim
 *   for test purposes
 *
 *   Revision 1.1  2008/04/30 14:56:27  guettler
 *   neuron population support, other improvements
 *
 *   Revision 1.7  2008/04/29 15:31:57  guettler
 *   tristate try
 *
 *   Revision 1.6  2008/04/29 11:06:48  guettler
 *   adjusted to changes of matrix.h
 *
 *   Revision 1.5  2008/04/29 07:31:41  guettler
 *   improvements
 *
 *   Revision 1.4  2008/04/28 10:31:11  guettler
 *   first working version, still incomplete
 *
 *   Revision 1.3  2008/04/28 06:55:20  guettler
 *   some advancements
 *
 *   Revision 1.2  2008/04/25 10:36:48  guettler
 *   bugfix
 *
 *   Revision 1.1  2008/04/25 10:31:50  guettler
 *   first version
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __TRISTATEIAFCONTROLLER_H
#define __TRISTATEIAFCONTROLLER_H

#include <selforg/abstractcontroller.h>

#include <selforg/matrix.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

typedef struct TristateIAFControllerConf {
  TristateIAFControllerConf() {
    thresholdI=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    thresholdO=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    leakI=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    leakO=(Configurable::paramval*) malloc(sizeof(Configurable::paramval));
    std::cout << "constructing conf" << std::endl;
  }
  ~TristateIAFControllerConf() {
    /*  if(thresholdI) free(thresholdI);
    if(thresholdO) free(thresholdO);
    if(leakI) free(leakI);
    if(leakO) free(leakO);*/
  }
  int numberIAFNeuronsPerInput;  ///< simulate a population if >1
  int numberIAFNeuronsPerOutput; ///< simulate a population if >1
  double wIInitScale;            ///< scaling factor of weights, initialized random
  double wOInitScale;            ///< between [-1*wInitScale,1*wInitScale]
  Configurable::paramval* thresholdI;
  Configurable::paramval* thresholdO;
  Configurable::paramval* leakI;
  Configurable::paramval* leakO;
} TristateIAFControllerConf;


/**
 * Abstract class (interface) for robot controller that uses an integrate
 * and fire neuron model
 *
 * Implements (assumes) that the sensor values are only 0 or 1.
 * Implements standard configureable interface for some useful parameters
 * like leakI and leakO (input and output layer)
 */
class TristateIAFController : public AbstractController {

public:
  TristateIAFController(const TristateIAFControllerConf& conf = getDefaultConf());

  virtual ~TristateIAFController() {}


  static TristateIAFControllerConf getDefaultConf(){
    TristateIAFControllerConf c;
    c.numberIAFNeuronsPerInput  = 10;
    c.numberIAFNeuronsPerOutput = 10;
    c.wIInitScale= 0.5;
    c.wOInitScale= 0.5;
    *c.thresholdI=0.75;
    *c.thresholdO=0.75;
    *c.leakI=0.05;
    *c.leakO=0.05;
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
  virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren=true);

protected:
  TristateIAFControllerConf conf;
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


};


#endif
