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
#ifndef __SOXIGNORENULL_H
#define __SOXIGNORENULL_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/teachable.h>


/// configuration object for SoxIgnoreNull controller. Use SoxIgnoreNull::getDefaultConf().
struct SoxIgnoreNullConf {
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
 * This controller implements the standard algorihm described the Chapter 5 (Homeokinesis)
 *  with extensions of Chapter 15 of the book "The Playful Machine"
 * Here the model does not learn on sensor inputs that are zero.
 */
class SoxIgnoreNull : public AbstractController, public Teachable {

public:
  /// constructor
  SoxIgnoreNull(const SoxIgnoreNullConf& conf = getDefaultConf());

  /// constructor provided for convenience, use conf object to customize more
  SoxIgnoreNull(double init_feedback_strength, bool useExtendedModel = true,
      bool useTeaching = false );

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~SoxIgnoreNull();

  static SoxIgnoreNullConf getDefaultConf(){
    SoxIgnoreNullConf conf;
    conf.initFeedbackStrength = 1.0;
    conf.useExtendedModel     = true;
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
  static const unsigned short buffersize = 10;

  matrix::Matrix A; // Model Matrix
  matrix::Matrix C; // Controller Matrix
  matrix::Matrix S; // Model Matrix (sensor branch)
  matrix::Matrix h; // Controller Bias
  matrix::Matrix b; // Model Bias
  matrix::Matrix L; // Jacobi Matrix
  matrix::Matrix R; //
  matrix::Matrix C_native; // Controller Matrix obtained from motor babbling
  matrix::Matrix A_native; // Model Matrix obtained from motor babbling
  matrix::Matrix y_buffer[buffersize]; // buffer needed for delay
  matrix::Matrix x_buffer[buffersize]; // buffer of sensor values
  matrix::Matrix v_avg;
  matrix::Matrix x;        // current sensor value vector
  matrix::Matrix x_smooth; // time average of x values

  matrix::Matrix eta_G; // teaching error

  int t;

  bool loga;

  SoxIgnoreNullConf conf; ///< configuration objects

  bool intern_isTeaching;    // teaching signal available?
  matrix::Matrix y_teaching; // motor teaching  signal

  paramval creativity;
  paramval sense;
  paramval harmony;
  paramval causeaware;
  paramint pseudo;
  paramval epsC;
  paramval epsA;
  paramval damping;
  paramval gamma;          // teaching strength

  paramval factorB;        // learning speed factor for model bias
  paramval factorH;        // learning speed factor for controller bias
  paramval factorS;        // learning speed factor for sensor branch

  void constructor();

  // calculates the pseudo inverse of L in different ways, depending on pseudo
  matrix::Matrix pseudoInvL(const matrix::Matrix& L, const matrix::Matrix& A, const matrix::Matrix& C);

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

  // if x (sensor value) is zero then we do not learn -> xsi=0;
  static double checkZero(double xsi, double x){
    if(x==0) return 0;
    else return xsi;
  }

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


