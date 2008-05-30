/*
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
 */
 
#ifndef __DINVERT3CHANNELCONTROLLER_H
#define __DINVERT3CHANNELCONTROLLER_H

#include "invertcontroller.h"

#include <assert.h>
#include <cmath>


/**
 * class for robot controller that use naglaa's direct matrix inversion for n channels 
 * (simple one layer networks)
 * 
 * Implements standart parameters: eps, rho, mu, stepnumber4avg, stepnumber4delay
 */
class DInvert3ChannelController : public InvertController {

public:

  /*
    Invert3ChannelController();

    //virtual ~Invert3ChannelController(){}


    /// performs one step (includes learning). Calulates motor commands from sensor inputs.
    virtual void step(const sensor*, motor*);

    /// performs one step without learning. Calulates motor commands from sensor inputs.
    virtual void stepNoLearning(const sensor*, motor*);
  */


protected:
  int NUMBER_CHANNELS;
  int BUFFER_SIZE;

  double* x_smooth;
  double* x_effective;
  double* y_effective;
  double** Q_buf1;
  double** Q_buf2;
  double** L;
  double* z;
  
public:
  double** A;    ///< model matrix
  double** C;    ///< controller matrix
  double* h;       ///< bias vector
  
  double** x_buffer; ///< buffer for input values, x[t%buffersize]=actual value, x[(t-1+buffersize)%buffersize]=x(t-1)  
  double** y_buffer; ///< buffer for output values, y[t%buffersize]=actual value(if already calculated!), y[(t-1+buffersize)%buffersize]=y(t-1)   
  
  double* xsi4E;
  double* xsi4Model;

  int    t;       ///< number of steps, needed for ringbuffer x_buffer
  char name[50];


  /*
    virtual void inverseMatrix(double Q[NUMBER_CHANNELS][NUMBER_CHANNELS], double Q_1[NUMBER_CHANNELS][NUMBER_CHANNELS]);

    virtual double calculateE(double *x_delay, double *y_delay);
  
    /// learn values h,C   //,A
    virtual void learn(double *x_delay, double *y_delay);
  
    /// learn model parameter (matrix A) by gradient descent
    virtual void learnModel(double *x_actual, double *y_effective);


    /// calculate delayed values
    virtual void calculateDelayedValues(double source[BUFFER_SIZE][NUMBER_CHANNELS], int number_steps_of_delay_, double *target);

    virtual void calculateSmoothValues(double source[BUFFER_SIZE][NUMBER_CHANNELS], int number_steps_for_averaging_, double *target);

    /// calculate controller ouptus
    virtual void calculateControllerValues(double *x_smooth, double *y);
  
    // put new value in ring buffer
    virtual void putInBuffer(double buffer[BUFFER_SIZE][NUMBER_CHANNELS], double *values);  

  */
  /// neuron transfer function
  virtual double g(double z)
  {
    return tanh(z);
  };


  ///
  virtual double g_s(double z)
  {
   /*  double k=tanh(z); */
/*     return 1.0 - k*k; */
    //    return 1.0 - tanh(z)*tanh(z);
    cout << "do not use this function g_s" << endl; 
    return 0; //1/(1+z*z);
  };

  /// squashing function, to protect against to large weight updates
  virtual double squash(double z)
  {
    return z < -0.1 ? -0.1 : ( z > 0.1 ? 0.1 : z );
    //return 0.1 * tanh(10.0 * z);
  };


#include "dinvert3channelcontroller.hpp"  

};

#endif
