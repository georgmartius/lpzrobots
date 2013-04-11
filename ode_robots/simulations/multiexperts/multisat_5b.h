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
 *    n experts, competition pairwise between winner and companion         *
 *    prediction error of companion is increased for competition (hysteris)*
 *    expert get mature, when winning and low error,                       *
 *     i.e. learning rate decreases                                        *
 *    always the agent with lowest maturation is selected  as new companion*
 *    only winner and companion are allowed to learn                        *
 *                                                                         *
 *    (NOT YET) sat networks have equal structure to selforg               *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.1  2008/11/14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.11  2008/04/22 15:22:55  martius
 *   removed test lib and inc paths from makefiles
 *
 *   Revision 1.10  2007/12/13 16:57:40  martius
 *   new variation of the multisat controller
 *
 *   Revision 1.9  2007/08/24 11:59:43  martius
 *   *** empty log message ***
 *
 *   Revision 1.8  2007/08/06 14:25:57  martius
 *   new version without gating network
 *
 *   Revision 1.7  2007/07/19 15:44:32  martius
 *   new multisat version without gating
 *
 *   Revision 1.6  2007/06/22 14:25:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2007/06/21 16:31:54  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2007/06/18 08:11:22  martius
 *   nice version with many experts
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
#include <selforg/multilayerffnn.h>

typedef struct MultiSatConf {
  AbstractController* controller;
  unsigned short buffersize; ///< size of the ringbuffers for sensors, motors,...
  int numHidden;        ///< number of hidden units in the satelite networks
  double eps0;          ///< learning rate for satelite networks
  double tauE1;         ///< time horizont for short averaging error
  double tauE2;         ///< time horizont for long averaging error
  double lambda_w;      ///< discount for winner prediction error (hysteresis)
  double tauW;          ///< time horizont for winner learning rate decay (maturation)
  int    numContext;    ///< number of context sensors (number of sensors that are ignored)
  double deltaMin;      ///< additive decay the minimum and eps term (in 1/1000) (should be very small)
  int    numSats;       ///< number of satelite networks
  bool   useDerive;     ///< input to sat network includes derivatives
  bool   useY;          ///< input to sat network includes y
  double satControlFactor; ///< factor of which the output of winner sat is used for control
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
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

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

  // !!!!!!!!!!!!!!!!!!! MISC STUFF !!!!!!!!

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
    c.eps0=0.005;
    //    c.lambda_comp = 0;
    c.deltaMin = 1.0/1000.0;
    // c.tauC = 10.0/c.eps0;
    c.tauE1 = 20;
    c.tauE2 = 200;
    c.tauW  = 1000;
    c.numContext=0;
    c.numSats=20;
    c.useDerive=false;
    c.useY=true;
    // c.penalty=5;
    c.satControlFactor = 0;
    return c;
  }


protected:
  unsigned short number_sensors;
  unsigned short number_motors;

  // sensor, sensor-derivative and motor values storage
  unsigned short buffersize;
  matrix::Matrix* x_buffer;
  matrix::Matrix* xp_buffer;
  matrix::Matrix* y_buffer;
  matrix::Matrix* x_context_buffer;

  std::vector <Sat> sats; ///< satelite networks
  int winner;    ///< index of winner network
  int companion; ///< index of companion network
  bool satControl; ///< True is sat networks take part in control
  matrix::Matrix nomSatOutput; ///< norminal output of satelite networks (x_t,y_t)^T
  matrix::Matrix satInput;     ///< input to satelite networks (x_{t-1}, xp_{t-1}, y_{t-1})^T

  bool runcompetefirsttime;       ///< flag to initialise averaging with proper value
  matrix::Matrix satErrors;       ///< actual errors of the sats
  matrix::Matrix satAvg1Errors;   ///< short averaged errors of sats
  matrix::Matrix satAvg2Errors;   ///< long averaged errors of sats
  matrix::Matrix satModErrors;    ///< modulated (avg1) errors of sats
  matrix::Matrix satMinErrors;    ///< minimum errors of sats (calculated from avg2)
  matrix::Matrix satEpsMod;       ///< modulated eps of sats
  /// modulats the importance of the prediction, e.g. sensors are less important
  matrix::Matrix satPredictWeight;

  MultiSatConf conf;
  bool initialised;
  int t;
  int managementInterval;       ///< interval between subsequent management calls

  /// satelite networks competition, return vector of predicted errors of sat networks
  matrix::Matrix compete();

  /// control of the robot through satelite network(s), and returns suggested control (or 0 matrix if none)
  matrix::Matrix controlBySat(int winner);

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
