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
 *   This version of multisat has the following features                   *
 *    n agents, competition by gating network                              *
 *    error predictions are modulated by penalty term which depends        *
 *    on suboptimality (away from minimum) for each agent                  *
 *    only winner is allowed to learn                                      *
 *    individual leaning rate is modulated by NPOSA                        *
 *    every agent has a lifetime and he will learn every timestep as       *
 *    with decreasing learningrate                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.1  2007/06/22 14:25:26  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2007/06/21 16:31:54  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2007/06/18 08:11:22  martius
 *   nice version with many agents
 *
 *   Revision 1.3  2007/06/14 08:01:45  martius
 *   Pred error modulation by distance to minimum works
 *
 *   Revision 1.2  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.1  2007/04/20 12:30:43  martius
 *   multiple sat networks test
 *
 *
 ***************************************************************************/
#ifndef __MULTISAT_H
#define __MULTISAT_H

#include <selforg/abstractcontroller.h>
#include <selforg/multilayerffnn.h>

#include <assert.h>
#include <math.h>

#include <selforg/matrix.h>
#include <selforg/noisegenerator.h>
#include <selforg/som.h>
#include <selforg/multilayerffnn.h>

typedef struct MultiSatConf {
  AbstractController* controller;
  unsigned short buffersize; ///< size of the ringbuffers for sensors, motors,...
  int numHidden;        ///< number of hidden units in the satelite networks
  double eps0;          ///< learning rate for satelite networks
  double tauE;          ///< time horizont for averaging error;
  double tauC;          ///< time horizont for annealing initial learning phase
  double lambda_comp;   ///< discount of learning rate for non-winners (competition) (modulated automatically)
  double deltaMin;      ///< additive decay the minimum term (in 1/1000) (should be very small)
  int    numSomPerDim;  ///< number of SOM neuronen per context dimension (attention raises exponential)
  int    numContext;    ///< number of context sensors
  int    numSats;       ///< number of satelite networks
  bool   useDerive;     ///< input to sat network includes derivatives
  double penalty;       ///< factor to multiply with square of difference of error and optimal error
} MultiSatConf;

/// Satelite network struct
typedef struct Sat {
  Sat(MultiLayerFFNN* _net, double _eps);
  MultiLayerFFNN* net;
  double eps;
  double lifetime;
} Sat;

/**
 * class for robot controller
 * using several feedforward networks (satelite) and one selforg controller
 */
class MultiSat : public AbstractController {

public:
  MultiSat(const MultiSatConf& conf = getDefaultConf());
  virtual void init(int sensornumber, int motornumber);

  virtual ~MultiSat();

  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; }

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);

  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  /// stores the sat networks into seperate files
  void storeSats(const char* filestem);


  /************** CONFIGURABLE ********************************/
  virtual paramval getParam(const paramkey& key, bool traverseChildren=true) const;
  virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren=true);
  virtual paramlist getParamList() const;


  /**** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  /**** INSPECTABLE ****/
  virtual std::list<iparamkey> getInternalParamNames() const;
  virtual std::list<iparamval> getInternalParams() const;
  virtual std::list<ILayer> getStructuralLayers() const;
  virtual std::list<IConnection> getStructuralConnections() const;

  static MultiSatConf getDefaultConf(){
    MultiSatConf c;
    c.buffersize=10;
    c.numHidden = 2;
    c.eps0=0.0005;
    c.lambda_comp = 0;
    c.deltaMin = 1.0/500.0;
    c.tauC = 10.0/c.eps0;
    c.tauE = 200;
    c.numContext=0;
    c.numSomPerDim=3;
    c.numSats=20;
    c.useDerive=false;
    c.penalty=5;
    return c;
  }


protected:
public:
  unsigned short number_sensors;
  unsigned short number_motors;

  // sensor, sensor-derivative and motor values storage
  unsigned short buffersize;
  matrix::Matrix* x_buffer;
  matrix::Matrix* xp_buffer;
  matrix::Matrix* y_buffer;
  matrix::Matrix* x_context_buffer;

  std::vector <Sat> sats; ///< satelite networks
  int winner; ///< index of winner network
  matrix::Matrix nomSatOutput; ///< norminal output of satelite networks (x_t,y_t)^T
  matrix::Matrix satInput;     ///< input to satelite networks (x_{t-1}, xp_{t-1}, y_{t-1})^T

  bool runcompetefirsttime;       ///< flag to initialise averaging with proper value
  matrix::Matrix satErrors;       ///< actual errors of the sats
  matrix::Matrix satPredErrors;   ///< predicted errors of sats
  matrix::Matrix satModPredErrors; ///< modulated predicted errors of sats
  matrix::Matrix satAvgErrors;    ///< average errors of sats
  matrix::Matrix satMinErrors;    ///< minimum errors of sats

  matrix::Matrix satLearnRateFactors;  ///< learning rate factors from 0.01-1.01

  SOM* gatingSom;
  MultiLayerFFNN* gatingNet;

  MultiSatConf conf;
  bool initialised;
  int t;
  int managementInterval;       ///< interval between subsequent management calls

  /// satelite networks competition, return vector of predicted errors of sat networks
  matrix::Matrix compete();

  // put new value in ring buffer
  void putInBuffer(matrix::Matrix* buffer, const matrix::Matrix& vec, int delay = 0);

  /// puts the sensors in the ringbuffer
  virtual void fillSensorBuffer(const sensor* x_, int number_sensors);
  /// puts the motors in the ringbuffer
  virtual void fillMotorBuffer(const motor* y_, int number_motors);


  /// handles inhibition damping etc.
  virtual void management();

  /** Calculates first and second derivative and returns both in on matrix (above).
      We use simple discrete approximations:
      \f[ f'(x) = (f(x) - f(x-1)) / 2 \f]
      \f[ f''(x) = f(x) - 2f(x-1) + f(x-2) \f]
      where we have to go into the past because we do not have f(x+1). The scaling can be neglegted.
  */
  matrix::Matrix calcDerivatives(const matrix::Matrix* buffer, int delay);

};

#endif
