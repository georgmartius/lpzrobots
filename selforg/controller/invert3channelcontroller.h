/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
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

#ifndef __INVERT3CHANNELCONTROLLER_H
#define __INVERT3CHANNELCONTROLLER_H

#include "invertcontroller.h"

#include <cassert>
#include <cmath>


/**
 * class for robot controller that use naglaa's direct matrix inversion for n channels
 * (simple one layer networks)
 *
 * Implements standart parameters: eps, rho, mu, stepnumber4avg, stepnumber4delay
 */
template <int NUMBER_CHANNELS, int BUFFER_SIZE=2> class Invert3ChannelController : public InvertController {

public:

  /*
    Invert3ChannelController();

    //virtual ~Invert3ChannelController(){}


    /// performs one step (includes learning). Calulates motor commands from sensor inputs.
    virtual void step(const sensor*, motor*);

    /// performs one step without learning. Calulates motor commands from sensor inputs.
    virtual void stepNoLearning(const sensor*, motor*);
  */


  //protected:
public:
  double A[NUMBER_CHANNELS][NUMBER_CHANNELS];    ///< model matrix
  double C[NUMBER_CHANNELS][NUMBER_CHANNELS];    ///< controller matrix
  double h[NUMBER_CHANNELS];       ///< bias vector

  double x_buffer[BUFFER_SIZE][NUMBER_CHANNELS]; ///< buffer for input values, x[t%buffersize]=actual value, x[(t-1+buffersize)%buffersize]=x(t-1)
  double y_buffer[BUFFER_SIZE][NUMBER_CHANNELS]; ///< buffer for output values, y[t%buffersize]=actual value(if already calculated!), y[(t-1+buffersize)%buffersize]=y(t-1)

  double xsi4E[NUMBER_CHANNELS];
  double xsi4Model[NUMBER_CHANNELS];

  int    t;       ///< number of steps, needed for ringbuffer x_buffer





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
    return 1.0 - tanh(z) * tanh(z);
  };



  /// squashing function, to protect against to large weight updates
  virtual double squash(double z)
  {
    return 0.1 * tanh(10.0 * z);
  };


  //template <int NUMBER_CHANNELS, int BUFFER_SIZES>
  //        Invert3ChannelController<int NUMBER_CHANNELS, int BUFFER_SIZES>::
  Invert3ChannelController(){

    t=0;

    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        h[i] = 0.0;
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            if (i == j)
              {
                A[i][j] = 1.0;
                C[i][j] = 0.1;
              }
            else
              {
                A[i][j] = 0.0;
                C[i][j] = 0.0;
              }
          }
      }
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        for (int k = 0; k < BUFFER_SIZE; k++)
          {
            x_buffer[k][i] = 0;
            y_buffer[k][i] = 0;
          }
      }

    /*    // print initial values
          std::cout<<"Constructor of RobotLearnControl:"<<std::endl;
          std::cout<<"init: epsilon="<<eps<<std::endl;
          std::cout<<"init: rho="<<rho<<std::endl;
          std::cout<<"init: stepnumber4delay="<<stepnumber4delay<<std::endl;
          std::cout<<"init: stepnumber4avg="<<stepnumber4avg<<std::endl;
          std::cout<<"init: delta="<<delta<<std::endl;
          std::cout<<"init: m (for calculation of E)="<<m<<std::endl;
    */
  };

  /// performs one step (includes learning). Calulates motor commands from sensor inputs.
  virtual void step(const sensor* x_, int sensornumber,
                    motor* y_, int motornumber) {
    double x_smooth[NUMBER_CHANNELS];
    double x_effective[NUMBER_CHANNELS];
    double y_effective[NUMBER_CHANNELS];
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        x_smooth[i] = 0.0;
        x_effective[i] = 0.0;
        y_effective[i] = 0.0;
      }

    // put new input value in ring buffer x_buffer
    putInBuffer(x_buffer, x_);

    // averaging over the last stepnumber4avg values of x_buffer
    calculateSmoothValues(x_buffer, stepnumber4avg, x_smooth);

    // calculate controller values based on smoothed input values
    calculateControllerValues(x_smooth, y_);

    // put new output value in ring buffer y_buffer
    putInBuffer(y_buffer, y_);

    // calculate effective input/output, which is (actual-stepnumber4delay) element of buffer
    calculateDelayedValues(x_buffer, stepnumber4delay, x_effective);
    calculateDelayedValues(y_buffer, stepnumber4delay, y_effective);

    // learn controller with effective input/output
    learn(x_, x_effective, y_effective);

    // learn model with actual input and effective output (that produced the actual input)
    learnModel(x_, y_effective);

    // update step counter
    t++;
  };


  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* x_, int sensornumber,
                              motor* y_, int motornumber){
    double x_smooth[NUMBER_CHANNELS];
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        x_smooth[i] = 0.0;
      }


    // put new input value in ring buffer x_buffer
    putInBuffer(x_buffer, x_);

    // averaging over the last stepnumber4avg values of x_buffer
    calculateSmoothValues(x_buffer, stepnumber4avg, x_smooth);

    // calculate controller values based on smoothed input values
    calculateControllerValues(x_smooth, y_);

    // put new output value in ring buffer y_buffer
    putInBuffer(y_buffer, y_);

    // update step counter
    t++;
  };


protected:
  virtual void inverseMatrix(double Q[NUMBER_CHANNELS][NUMBER_CHANNELS], double Q_1[NUMBER_CHANNELS][NUMBER_CHANNELS]){
    // Berechne Inverse von Q

    // only if NUMBER_CHANNELS<4
    assert(NUMBER_CHANNELS<4);
    if (NUMBER_CHANNELS==2){

      double det = Q[0][0] * Q[1][1] - Q[0][1] * Q[1][0];
      Q_1[0][0] = Q[1][1] / det;
      Q_1[1][1] = Q[0][0] / det;
      Q_1[0][1] = -Q[0][1] / det;
      Q_1[1][0] = -Q[1][0] / det;

    }


    if (NUMBER_CHANNELS==3){

      double  Q_adjoint[NUMBER_CHANNELS][NUMBER_CHANNELS]  ;
      double  detQ=0  ;

      //calculate the inverse of Q
      Q_adjoint[0][0]=Q[1][1]*Q[2][2]-Q[1][2]*Q[2][1] ;
      Q_adjoint[0][1]=(Q[1][2]*Q[2][0]-Q[1][0]*Q[2][2]) ;
      Q_adjoint[0][2]=Q[1][0]*Q[2][1]-Q[1][1]*Q[2][0] ;
      Q_adjoint[1][0]=(Q[2][1]*Q[0][2]-Q[0][1]*Q[2][2]) ;
      Q_adjoint[1][1]=Q[0][0]*Q[2][2]-Q[0][2]*Q[2][0] ;
      Q_adjoint[1][2]=(Q[0][1]*Q[2][0]-Q[0][0]*Q[2][1]) ;
      Q_adjoint[2][0]=Q[0][1]*Q[1][2]-Q[1][1]*Q[0][2] ;
      Q_adjoint[2][1]=(Q[1][0]*Q[0][2]-Q[0][0]*Q[1][2]) ;
      Q_adjoint[2][2]=Q[0][0]*Q[1][1]-Q[0][1]*Q[1][0] ;
      detQ=Q[0][0]*Q_adjoint[0][0]+Q[0][1]*Q_adjoint[0][1]+Q[0][2]*Q_adjoint[0][2] ;
      for(int i=0; i<NUMBER_CHANNELS; i++){
        for(int j=0; j<NUMBER_CHANNELS; j++) {
          Q_1[i][j]=(Q_adjoint[j][i])/detQ  ;
        }
      }
    }
  };


  virtual double calculateE(const double *x_, const double *x_delay, const  double *y_delay) {
    double L[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double Q[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double Q_1[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double z[NUMBER_CHANNELS];


    // Calculate z based on the delayed inputs since the present input x is
    // produced by the outputs tau time steps before
    // which on their hand are y = K(x_D)
    // due to the delay in the feed back loop.

    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        z[i] = h[i];
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            z[i] += C[i][j] * x_delay[j];
          }
      }

    // Berechne Matrix L
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            L[i][j] = 0.0;
            for (int k = 0; k < NUMBER_CHANNELS; k++)
              {
                L[i][j] += A[i][k] * g_s(z[k]) * C[k][j];
              }
          }
      }

    // Berechne Q=LL^T
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            Q[i][j] = 0.0;
            for (int k = 0; k < NUMBER_CHANNELS; k++)
              {
                Q[i][j] += L[i][k] * L[j][k];
              }
            if (i == j)
              Q[i][j] += rho / NUMBER_CHANNELS; // Regularisation
          }
      }

    // Berechne Inverse von Q
    inverseMatrix(Q, Q_1);

    // Berechne xsi
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        xsi4E[i] = x_[i];
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            xsi4E[i] -= A[i][j] * y_delay[j];  // using old y value -> no influence of changes (adding delta) in controller
            // very very strange!!!!
            //xsi4E[i] -= A[i][j] * g(z[j]);     // using recalculating y -> influence of changes (adding delta) in controller
          }
      }
    double E = 0;
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            E += xsi4E[i] * Q_1[i][j] * xsi4E[j];
          }
      }

    double E_s=0;
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            E_s += (A[i][j]*g(z[j]) - x_[i]) * (A[i][j]*g(z[j]) - x_[i]);
          }
      }

    E=(1-m)*E+ m*E_s;
    return (E);

  };


  virtual void learn(const double *x_, const double *x_delay, const double *y_delay)
  {
    //    double A_update[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double C_update[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double h_update[NUMBER_CHANNELS];

    double E_0 = calculateE(x_,x_delay, y_delay);

    // calculate updates for h,C,A
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        h[i] += delta;
        h_update[i] = -eps * (calculateE(x_,x_delay, y_delay) - E_0) / delta;
        h[i] -= delta;
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            C[i][j] += delta;
            C_update[i][j] = -eps * (calculateE(x_,x_delay, y_delay) - E_0) / delta;
            C[i][j] -= delta;
            //A[i][j] += delta;
            //A_update[i][j] = -eps * a_factor * (calculateE(x_delay, y_delay) - E_0) / delta;
            //A[i][j] -= delta;
          }
      }

    // apply updates to h,C,A
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        h[i] += squash(h_update[i]);
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            C[i][j] += squash(C_update[i][j]);
            //A[i][j] += squash(A_update[i][j]);
          }
      }
  };


  virtual void learnModel(const double *x_actual, double *y_effective){
    /*         double z[N_output];
        for(int i=0; i<N_output; i++){
        z[i]=h[i];
        for(int j=0; j<N_input; j++) {
        z[i]+=C[i][j]*x_D[j];
        }
        }
    */
    // Berechne xsi
    for(int i=0; i<NUMBER_CHANNELS; i++){
      xsi4Model[i]=x_actual[i];
      for(int j=0; j<NUMBER_CHANNELS; j++){
        xsi4Model[i]-= A[i][j]*y_effective[j];
      }
    }

    for(int i=0; i<NUMBER_CHANNELS; i++){
      for (int j=0; j<NUMBER_CHANNELS; j++){
        A[i][j]+=squash( (factor_a*eps*0.2) *xsi4Model[i] * y_effective[j]) ;
      }
    }
  };





  /// calculate delayed values
  virtual void calculateDelayedValues(double source[BUFFER_SIZE][NUMBER_CHANNELS], paramval number_steps_of_delay_, double *target)
  {
    // number_steps_of_delay_ must not be larger than BUFFER_SIZE
    assert ((int)number_steps_of_delay_ < BUFFER_SIZE);

    // get delayed value from ring buffer

    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        target[i] = source[(t - (int)number_steps_of_delay_ + BUFFER_SIZE) % BUFFER_SIZE][i];
      }
  };

  virtual void calculateSmoothValues(double source[BUFFER_SIZE][NUMBER_CHANNELS], paramval number_steps_for_averaging_, double *target)
  {
    // number_steps_for_averaging_ must not be larger than BUFFER_SIZE
    assert ((int)number_steps_for_averaging_ <= BUFFER_SIZE);

    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        target[i] = 0.0;
        for (int k = 0; k < (int)number_steps_for_averaging_; k++)
          {
            target[i] += source[(t - k + BUFFER_SIZE) % BUFFER_SIZE][i]/ (double) (number_steps_for_averaging_);
          }
      }
  };



  /// calculate controller ouptus
  virtual void calculateControllerValues(double *x_smooth, double *y)
  {
    double z[NUMBER_CHANNELS];

    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        z[i] = h[i];
        for (int j = 0; j < NUMBER_CHANNELS; j++)
          {
            z[i] += C[i][j] * x_smooth[j];
          }
        y[i] = g(z[i]);
      }
  };


  // put new value in ring buffer
  virtual void putInBuffer(double buffer[BUFFER_SIZE][NUMBER_CHANNELS], const double *values){
    for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        buffer[(t+BUFFER_SIZE)% BUFFER_SIZE][i] = values[i];
      }
  };



  virtual int getInternalParamNames(paramkey*& keylist){
    keylist=(paramkey*)malloc(sizeof(paramkey)*10);
    keylist[0]="C00";
    keylist[1]="C01";
    keylist[2]="C10";
    keylist[3]="C11";
    keylist[4]="H0";
    keylist[5]="H1";
    keylist[6]="A00";
    keylist[7]="A01";
    keylist[8]="A10";
    keylist[9]="A11";
    return 10;
  }

  virtual int getInternalParams(paramval* vallist, int length) {
    if(length < 10) return 0;
    vallist[0]=C[0][0];
    vallist[1]=C[0][1];
    vallist[2]=C[1][0];
    vallist[3]=C[1][1];
    vallist[4]=h[0];
    vallist[5]=h[1];
    vallist[6]=A[0][0];
    vallist[7]=A[0][1];
    vallist[8]=A[1][0];
    vallist[9]=A[1][1];
    return 10;
  }


};

AbstractController* getController(int sensornumber, int motornumber,
                                  int param1/*=0*/, double param2/*=0*/)
{
  if(param1 < 2) param1 = 2;
  return new Invert3ChannelController<2, 4>;
}



#endif
