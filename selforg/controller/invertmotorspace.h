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
#ifndef __INVERTMOTORSPACE_H
#define __INVERTMOTORSPACE_H

#include "invertmotorcontroller.h"
#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/noisegenerator.h>

/**
 * class for robot controller that uses the georg's matrixlib for
 *  direct matrix inversion for n channels
 * (simple one layer networks)
 *
 * Implements standart parameters: eps, rho, mu, stepnumber4avg, stepnumber4delay
 */
class InvertMotorSpace : public InvertMotorController {

public:
  InvertMotorSpace(int buffersize, double cInit = 0.1 , bool someInternalParams = true);
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~InvertMotorSpace();

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
  /** stores the controller values to a given file (binary).  */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file (binary). */
  virtual bool restore(FILE* f);

  // inspectable interface
  virtual std::list<ILayer> getStructuralLayers() const;
  virtual std::list<IConnection> getStructuralConnections() const;


protected:
  unsigned short number_sensors;
  unsigned short number_motors;

  matrix::Matrix A; // Model Matrix
  matrix::Matrix C; // Controller Matrix
  matrix::Matrix R; // C*A
  matrix::Matrix H; // Controller Bias
  matrix::Matrix B; // Model Bias
  NoiseGenerator* BNoiseGen; // Noisegenerator for noisy bias
  matrix::Matrix* x_buffer;
  matrix::Matrix* y_buffer;
  matrix::Matrix x_smooth;

  bool someInternalParams;
  double cInit;

  /// puts the sensors in the ringbuffer, generate controller values and put them in the
  //  ringbuffer as well
  void fillBuffersAndControl(const sensor* x_, int number_sensors,
                             motor* y_, int number_motors);

  /// learn h,C, delayed motors y and corresponding sensors x
  virtual void learnController(const matrix::Matrix& x, const matrix::Matrix& x_smooth, int delay);

  /// learn A, using motors y and corresponding sensors x
  virtual void learnModel( const matrix::Matrix& x, const matrix::Matrix& y);

  /// returns controller output for given sensor values
  virtual matrix::Matrix calculateControllerValues(const matrix::Matrix& x_smooth);

};

#endif
