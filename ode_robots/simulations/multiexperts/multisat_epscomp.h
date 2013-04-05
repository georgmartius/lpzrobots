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
 *   $Log$
 *   Revision 1.1  2007-06-08 15:39:50  martius
 *   new multi expert (satellite networks) stuff
 *   works nice
 *
 *
 ***************************************************************************/
#ifndef __MULTISAT_H
#define __MULTISAT_H

#include <selforg/invertmotornstep.h>
#include <selforg/multilayerffnn.h>

#include <assert.h>
#include <math.h>

#include <selforg/matrix.h>
#include <selforg/noisegenerator.h>

typedef struct MultiSatConf {
  InvertMotorNStep* controller;
  unsigned short buffersize; ///< size of the ringbuffers for sensors, motors,...
  int numberHidden;   ///< number of hidden units in the satelite networks
  double eps0;        ///< initial learning rate for satelite networks
  double lambda_time; ///< decay of learning rate over time
  double lambda_comp; ///< discount of learning rate for non-winners
  double tau;            ///< time horizont for averaging error ;
} MultiSatConf;

/// Satelite network struct
typedef struct Sat {
  Sat(MultiLayerFFNN* _net, double _eps);
  MultiLayerFFNN* net;
  double eps;
  double sigma;
  double error;
  double avg_error;
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
    c.controller=0;
    c.buffersize=10;
    c.numberHidden = 10;
    c.eps0=0.4;
    c.lambda_time = 0.999;
    c.lambda_comp = 0.05;
    c.tau = 10;
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

  std::vector <Sat> sats; ///< satelite networks
  int winner; ///< index of winner network
  matrix::Matrix nomSatOutput; ///< norminal output of satelite networks (x_t,y_t)^T
  matrix::Matrix satInput;     ///< input to satelite networks (x_{t-1}, xp_{t-1}, y_{t-1})^T

  MultiSatConf conf;
  bool initialised;
  int t;

  /// satelite networks competition
  int compete();

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
