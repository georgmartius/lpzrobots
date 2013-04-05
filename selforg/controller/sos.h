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
#ifndef __SOS_H
#define __SOS_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>

/**
 * This controller implements the standard algorihm described the Chapter 5 (Homeokinesis)
 *  of book "The Playful Machine"
 */
class Sos : public AbstractController {

public:
  Sos(double init_feedback_strength = 1.0);
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~Sos();

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


  /***** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  /* some direct access functions (unsafe!) */
  virtual matrix::Matrix getA();
  virtual void setA(const matrix::Matrix& A);
  virtual matrix::Matrix getC();
  virtual void setC(const matrix::Matrix& C);
  virtual matrix::Matrix geth();
  virtual void seth(const matrix::Matrix& h);

protected:
  unsigned short number_sensors;
  unsigned short number_motors;
  static const unsigned short buffersize = 10;

  matrix::Matrix A; // Model Matrix
  matrix::Matrix C; // Controller Matrix
  matrix::Matrix h; // Controller Bias
  matrix::Matrix b; // Model Bias
  matrix::Matrix L; // Jacobi Matrix
  matrix::Matrix y_buffer[buffersize]; // buffer needed for delay
  matrix::Matrix x_buffer[buffersize]; // buffer of sensor values
  matrix::Matrix v_avg;
  matrix::Matrix x;        // current sensor value vector
  matrix::Matrix x_smooth; // time average of x values
  int t;
  bool TLE;
  bool loga;

  double init_feedback_strength;

  paramval creativity;
  paramval epsC;
  paramval epsA;
  paramint s4avg;          // # of steps the sensors are averaged (1 means no averaging)
  paramint s4delay;        // # of steps the motor values are delayed (1 means no delay)


  /// learn values model and controller (A,b,C,h)
  virtual void learn();

  /// neuron transfer function
  static double g(double z)
  {
    return tanh(z);
  };

  /// derivative of g
  static double g_s(double z)
  {
    double k=tanh(z);
    return 1.0 - k*k;
  };

  /// function that clips the second argument to the interval [-first,first]
  static double clip(double r, double x){
    return min(max(x,-r),r);
  }
  /// calculates the inverse the argument (useful for Matrix::map)
  static double one_over(double x){
    return 1/x;
  }


};

#endif


