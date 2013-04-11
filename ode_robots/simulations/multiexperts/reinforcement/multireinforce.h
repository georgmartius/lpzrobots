/***************************************************************************
 *   Copyright (C) 2008 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
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
 *   This version of multireinforce has the following features             *
 *    n agents can control robot                                           *
 *    external (e.g. keyboard) control possible                            *
 *    Q-learning used for action selection                                 *
 *     states: x, defined by subclass                                       *
 *     action: y, where y is an agent                                      *
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.2  2008/04/22 15:22:55  martius
 *   removed test lib and inc paths from makefiles
 *
 *   Revision 1.1  2007/08/29 15:32:52  martius
 *   reinforcement learning with 4 wheeled
 *
 *
 ***************************************************************************/
#ifndef __MULTIREINFORCE_H
#define __MULTIREINFORCE_H

#include <selforg/abstractcontroller.h>
#include <selforg/multilayerffnn.h>

#include <assert.h>
#include <math.h>

#include <selforg/matrix.h>
#include <selforg/noisegenerator.h>
#include <selforg/multilayerffnn.h>
#include <selforg/qlearning.h>

typedef struct MultiReinforceConf {
  unsigned short buffersize; ///< size of the ringbuffers for sensors, motors,...
  int    numContext;    ///< number of context sensors (ignored)
  std::list<std::string> satFiles; /// filenames for sat networks
  int    numSats;       ///< number of satelite networks (derived from length of files
  bool   useDerive;     ///< input to sat network includes derivatives
  bool   useY;          ///< input to sat network includes y (motor values)
  double tauE1;         ///< time horizont for short averaging error
  double tauH;          ///< hystersis time (time an state is kept even another one seams right)
  double tauI;          ///< maximal waiting time for state change if action was changed
  int reinforce_interval; ///<  time between consecutive reinforcement selections

  QLearning* qlearning;      ///< QLearning instance
  matrix::Matrix* actioncorrel; /// correlation matrix of actions
} MultiReinforceConf;

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
class MultiReinforce : public AbstractController {

public:
  MultiReinforce(const MultiReinforceConf& conf = getDefaultConf());
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~MultiReinforce();

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

  /// restores the sat networks from seperate files
  static std::list<std::string> createFileList(const char* filestem, int n);
  /// restores the sat networks from seperate files
  void restoreSats(const std::list<std::string>& files);
  /// stores the sats into the given files
  void storeSats(const std::list<std::string>& files);


  /** enables/disables manual control, action_ is the sat network number to be used
      if mControl is false, action is ignored
   */
  void setManualControl(bool mControl, int action_ = 0);


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

  static MultiReinforceConf getDefaultConf(){
    MultiReinforceConf c;
    c.buffersize=10;
    c.numContext=0;
    c.numSats=0; // has to be changed by user!
    c.useDerive=false;
    c.useY=true;
    c.qlearning=0;
    c.tauE1=25;
    c.tauH=10;
    c.tauI=50;
    c.reinforce_interval=10;
    c.actioncorrel=0;
    c.qlearning=0;
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

  std::vector <Sat> sats;      ///< satelite networks
  bool manualControl;          ///< True if actions (sats) are selected manually
  matrix::Matrix nomSatOutput; ///< norminal output of satelite networks (x_t,y_t)^T
  matrix::Matrix satInput;     ///< input to satelite networks (x_{t-1}, xp_{t-1}, y_{t-1})^T
  int action;                  ///< index of controlling network
  int newaction;               ///< index of new controlling network
  int oldaction;               ///< index of old controlling network
  int state;                   ///< current state
  double reward;               ///< current reward
  double oldreward;            ///< old reward (nicer for plotting)
  int phase;                   ///< current phase of the controller: 0: action just selected 1:state changed first time 2:state changed second time
  int phasecnt;               ///< counts number of steps in one phase.

  matrix::Matrix satErrors;       ///< actual errors of the sats
  matrix::Matrix satAvgErrors;    ///< averaged errors of the sats
  matrix::Matrix statesbins;      ///< bins with counts for each state

  MultiReinforceConf conf;
  bool initialised;
  int t;
  int managementInterval;       ///< interval between subsequent management calls

  /// returns number of state, to be overwritten
  virtual int getStateNumber() = 0;

  /// returns state, to be overwritten
  virtual int calcState() = 0;

  /// returns the reinforcement (reward), to be overwritten
  virtual double calcReinforcement() = 0;


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
