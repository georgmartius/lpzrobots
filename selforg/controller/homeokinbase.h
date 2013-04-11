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

#ifndef __HOMEOKINBASE_H
#define __HOMEOKINBASE_H

#include "abstractcontroller.h"
#include "controller_misc.h"
#include <stdlib.h>
#include <string.h>


/**
 * Abstract class (interface) for robot controller that use are based on the homeokinetic
 * prinziple
 *
 * Implements standard buffers and
 *  configureable interface for some useful parameters like  epsC, epsA, s4avg ...
 */
class HomeokinBase : public AbstractController {
public:
  HomeokinBase( unsigned short buffersize ,
                 const std::string& name, const std::string& revision)
    : AbstractController(name, revision){
    this->buffersize = buffersize;
    addParameterDef("epsC", &epsC, 0.1);
    addParameterDef("epsA",&epsA,  0.1);
    addParameterDef("s4delay",&s4delay,1);
    addParameterDef("s4avg",&s4avg,1);
    addParameterDef("factorB",&factorB,0.2);
    addParameterDef("squashsize",&squashSize,0.01);
    addParameterDef("rootE",&rootE,0);
    addParameterDef("logaE",&logaE,0);
    t=0;
    initialised = false;
  }


protected:
  paramval epsC; ///< learning rate factor for controller learning
  paramval epsA; ///< learning rate factor for model learning
  paramval factorB; ///< additional learning rate factor for model bias
  paramint s4delay; ///< number of timesteps of delay in the SML
  paramint s4avg; ///< number of timesteps used for smoothing the controller output values
  paramint logaE;  ///< logarithmic error is used for learning 1: controller 2: model 3: both
  paramint rootE;  ///< root error is used for learning 1: controller 2: model 3: both


  paramval squashSize; ///< size of the box, where the parameter updates are clipped to

  int t;
  unsigned short buffersize;
  bool initialised;

protected:
  /// put new value in ring buffer
  void putInBuffer(matrix::Matrix* buffer, const matrix::Matrix& vec, int delay=0){
    buffer[(t-delay)%buffersize] = vec;
  }

  /// calculate delayed values
  virtual matrix::Matrix calculateDelayedValues(const matrix::Matrix* buffer,
                                                  int number_steps_of_delay_){
    // number_steps_of_delay must not be smaller than buffersize
    assert ((unsigned)number_steps_of_delay_ < buffersize);
    return buffer[(t - number_steps_of_delay_) % buffersize];
  };

  /// calculate time-smoothed values
  virtual matrix::Matrix calculateSmoothValues(const matrix::Matrix* buffer,
                                                 int number_steps_for_averaging_){
    // number_steps_for_averaging_ must not be larger than buffersize
    assert ((int)number_steps_for_averaging_ <= buffersize);

    matrix::Matrix result(buffer[t % buffersize]);
    for (int k = 1; k < number_steps_for_averaging_; k++) {
      result += buffer[(t - k + buffersize) % buffersize];
    }
    result *= 1/((double) (number_steps_for_averaging_)); // scalar multiplication
    return result;
  };

  /// calculates the error_factor for either logarithmic (E=ln(e^T*e)) or square (E=sqrt(e^t*e)) error
  virtual double calcErrorFactor(const matrix::Matrix& e, bool loga, bool root) {
    double error_factor = 1;
    if (loga){   // using logarithmic error E=ln(v^T*v)
      error_factor= 1/(e.multTM().val(0,0)+0.000001)*0.01; // factor 1/100 for normalising (empirically)
    }
    if (root){  // using root error E=(v^T*v)^(1/2)
      error_factor= 1/sqrt(e.multTM().val(0,0)+0.000001)*0.1; // factor 1/10 for normalising (empirically)
    }
    return error_factor;
  }


  /// neuron transfer function
  static double g(double z)
  {
    return tanh(z);
  };



};

#endif
