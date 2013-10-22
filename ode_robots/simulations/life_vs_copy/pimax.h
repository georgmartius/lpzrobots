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
#ifndef __PIMAX_H
#define __PIMAX_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/teachable.h>


/// configuration object for PiMax controller. Use PiMax::getDefaultConf().
struct PiMaxConf {
  double initFeedbackStrength;  ///< initial strength of sensor to motor connection
  bool   useExtendedModel;      ///< if true, the extended model (S matrix) is used
  /// if true the controller can be taught see teachable interface
  bool   useTeaching;
  /// # of steps the sensors are averaged (1 means no averaging)
  int    steps4Averaging;
  /// # of steps the motor values are delayed (1 means no delay)
  int    steps4Delay;
  bool   someInternalParams;    ///< if true only some internal parameters are exported
  bool   onlyMainParameters;    ///< if true only some configurable parameters are exported
};


/**
 * This controller implements the predictive information maximization
   described in paper: to be published
   TODO: insert paper
 */
class PiMax : public AbstractController, public Teachable {

public:
  /// constructor
  PiMax(const PiMaxConf& conf = getDefaultConf());

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~PiMax();

  static PiMaxConf getDefaultConf(){
    PiMaxConf conf;
    conf.initFeedbackStrength = 1.0;
    conf.useExtendedModel     = false;
    conf.useTeaching          = false;
    conf.steps4Averaging      = 1;
    conf.steps4Delay          = 1;
    conf.someInternalParams   = false;
    conf.onlyMainParameters   = true;
    return conf;
  }

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

  /// called during babbling phase
  virtual void motorBabblingStep(const sensor* , int number_sensors,
                                 const motor* , int number_motors);

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

  /***** TEACHABLE ****/
  virtual void setMotorTeaching(const matrix::Matrix& teaching);
  virtual void setSensorTeaching(const matrix::Matrix& teaching);
  virtual matrix::Matrix getLastMotorValues();
  virtual matrix::Matrix getLastSensorValues();

protected:
  unsigned short number_sensors;
  unsigned short number_motors;
  static const unsigned short buffersize = 20;

  matrix::Matrix A; // Model Matrix
  matrix::Matrix C; // Controller Matrix
  matrix::Matrix S; // Model Matrix (sensor branch)
  matrix::Matrix h; // Controller Bias
  matrix::Matrix b; // Model Bias
  matrix::Matrix L; // Jacobi Matrix

  matrix::Matrix Sigma; // noise covariance matrix

  matrix::Matrix ds0; //
  matrix::Matrix C_native; // Controller Matrix obtained from motor babbling
  matrix::Matrix A_native; // Model Matrix obtained from motor babbling

  matrix::Matrix a_buffer[buffersize]; // buffer needed for delay
  matrix::Matrix s_buffer[buffersize]; // buffer of sensor values
  matrix::Matrix xi_buffer[buffersize]; // buffer of pred errors
  matrix::Matrix gs_buffer[buffersize]; // buffer of g'
  matrix::Matrix L_buffer[buffersize]; // buffer of Jacobians

  matrix::Matrix s;        // current sensor value vector
  matrix::Matrix s_smooth; // time average of s values
  matrix::Matrix diago;  //TEST
  PiMaxConf conf; ///< configuration objects

  int t;


  bool intern_isTeaching;    // teaching signal available?
  matrix::Matrix a_teaching; // motor teaching  signal

  bool useMetric;
  paramval creativity;
  paramval sense;
  paramval harmony;
  paramval causeaware;
  paramint pseudo;
  paramval epsC;
  paramval epsA;
  paramval epsSigma;
  paramval factorH;
  paramval damping;
  paramval gamma;          // teaching strength

  paramint tau;            // length of time window


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
    return 1.05 - k*k;
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


