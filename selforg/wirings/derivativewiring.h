/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/
#ifndef __DERIVATIVEWIRING_H
#define __DERIVATIVEWIRING_H

#include "abstractwiring.h"

/**  Configuration object for DerivativeWiring.
     If all boolean parametes are false, id is set to true (equivalent to One2OneWiring)
*/
typedef struct __DerivativeWiringConf {
  bool useId;        //< include zeroth derivative
  bool useFirstD;   ///< include first derivative
  bool useSecondD;  ///< second include second derivative
  double eps;       ///< update rate for floating average (0 -> no sensor variation, 1 -> no smoothing)
  double derivativeScale;   ///< factor for the derivatives
  unsigned int blindMotors;   ///< number of motors that are blind (not given to robot)
} DerivativeWiringConf;


/** Implements a wiring (between controller and robot)
    which includes the first and second derivative
    of the original robot sensor values
*/
class DerivativeWiring : public AbstractWiring{
public:
  /** constructor
      @param conf  for giving the wished configuration of DerivativeWiring
      via \ref __DerivativeWiringConf "DerivativeWiringConf"
      @param noise NoiseGenerator that is used for adding noise to sensor values
  */
  DerivativeWiring(const DerivativeWiringConf& conf,
                   NoiseGenerator* noise, const std::string& name = "DerivativeWiring");

  /** destructor
   */
  virtual ~DerivativeWiring();

  /** Providing default configuration for DerivativeWiring with first derivative.
      No smoothing and no scaling. ( as static method )
   */
  static DerivativeWiringConf getDefaultConf(){
    DerivativeWiringConf c;
    c.useId = true;        // use id
    c.useFirstD = false;    // use first derivative
    c.useSecondD = false;  // do not use secound derivative
    c.eps = 1;             // no smoothing
    c.derivativeScale=1;   // no scaleing
    c.blindMotors=0;       // no blind motors used
    return c;
  };

  /** Providing default configuration for DerivativeWiring for only first derivative.
      smoothing over 4 steps and scale of 5. Use smaller noise!
      ( as static method )
   */
  static DerivativeWiringConf getDefaultConf1(){
    DerivativeWiringConf c;
    c.useId = false;       // do not use id
    c.useFirstD = true;    // use first derivative
    c.useSecondD = false;  // do not use secound derivative
    c.eps = 0.5;          // smoothing over 2 steps
    c.derivativeScale=5;   // scaling with 5
    c.blindMotors=0;       // no blind motors used
    return c;
  };

  virtual void reset();

protected:

  virtual bool initIntern();

  virtual bool wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                 sensor* csensors, int csensornumber,
                                 double noise);

  virtual bool wireMotorsIntern(motor* rmotors, int rmotornumber,
                                const motor* cmotors, int cmotornumber);

protected:
  /** Calculate the first derivative of the sensorvalues given by the robot
   *  f'(x) = (f(x+1) - f(x-1)) / 2
   *  since we do not have f(x+1) we go one timestep in the past
   */
  void calcFirstDerivative();

  /** Calculate the secound derivative of the sensorvalues given by the robot
   *  f'(x) = f(x) - 2f(x-1) + f(x-2)
   */
  void calcSecondDerivative();

  /// used configuration
  DerivativeWiringConf conf;
  static const int buffersize=40;
  int time;
  /// number timesteps the sensor values are delayed for calculation of the derivative
  //  int delay;


  /// current and old smoothed sensor values of robot
  sensor* sensorbuffer[buffersize];

  /// current sensors (with noise)
  // sensor* id;

  /// current first derivative
  sensor* first;

  /// current second derivative
  sensor* second;

  /// array that stored the values of the blind motors
  motor *blindMotors;

};

#endif
