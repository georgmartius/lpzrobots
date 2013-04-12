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
#ifndef __SERVO1_H
#define __SERVO1_H

#include "pid.h"
#include "odehandle.h"

namespace lpzrobots {

  class OneAxisJoint;

  /** general servo motor to achieve position control
   */
  class OneAxisServo {
  public:
    /** min and max values are understood as travel bounds. Min should be less than 0.*/

    OneAxisServo(OneAxisJoint* joint, double _min, double _max,
                 double power, double damp=0.2, double integration=2, double maxVel=10.0,
                 double jointLimit = 1.3, bool minmaxCheck = true);

    virtual ~OneAxisServo();

    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max
    */
    virtual void set(double pos);

    /** returns the position of the slider in ranges [-1, 1] (scaled by min, max)*/
    virtual double get();

    virtual void setMinMax(double _min, double _max);

    /** adjusts the power of the servo*/
    virtual void setPower(double power);

    /** returns the power of the servo*/
    virtual double getPower();

    /** returns the damping of the servo*/
    virtual double getDamping();

    /** sets the damping of the servo*/
    virtual void setDamping(double damp);

    /** returns the integration term of the PID controller of the servo*/
    virtual double& offsetCanceling();

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel);

    /** adjusts maximal speed of servo*/
    virtual double getMaxVel();


  protected:
    OneAxisJoint* joint;
    double min;
    double max;
    PID pid;
    double maxVel;
    double jointLimit; ///< joint limit with respect to servo limit
  };

  typedef OneAxisServo SliderServo;
  typedef OneAxisServo HingeServo;
  typedef OneAxisServo Hinge2Servo;



  /** general servo motor to achieve position control with zero position centered
   */
  class OneAxisServoCentered : public OneAxisServo {
  public:
    /** min and max values are understood as travel bounds.
        The zero position is (max-min)/2
    */
    OneAxisServoCentered(OneAxisJoint* joint, double _min, double _max,
                         double power, double damp=0.2, double integration=2,
                         double maxVel=10.0, double jointLimit = 1.3);

    virtual ~OneAxisServoCentered();

    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max,
        however 0 is just in the center of min and max
    */
    virtual void set(double pos);

    /** returns the position of the slider in ranges [-1, 1] (scaled by min, max, centered)*/
    virtual double get();

  };

  /** general servo motor to achieve position control.
   *  It internally controls the velocity of the motor (much more stable)
   *  with centered zero position.
   *  The amount of body feeling can be adjusted by the damping parameter
   *   which is understood as a stiffness parameter
   */
  class OneAxisServoVel : public OneAxisServo {
  public:
    /** min and max values are understood as travel bounds.
        The zero position is (max-min)/2
        @param power is the maximal torque the servo can generate
        @param maxVel is understood as a speed parameter of the servo.
        @param damp adjusts the power of the servo in dependence of the distance
         to the set point. This regulates the stiffness and the body feeling
          0: the servo has no power at the set point (maximal body feeling);
          1: is servo has full power at the set point: perfectly damped.

    */
    OneAxisServoVel(const OdeHandle& odeHandle,
                    OneAxisJoint* joint, double _min, double _max,
                    double power, double damp=0.05, double maxVel=20,
                    double jointLimit = 1.3);

    virtual ~OneAxisServoVel();

    /** adjusts the power of the servo*/
    virtual void setPower(double _power);

    /** returns the power of the servo*/
    virtual double getPower();

    virtual double getDamping();

    virtual void setDamping(double _damp);

    /** offetCanceling does not exist for this type of servo */
    virtual double& offsetCanceling();

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel);

    /** adjusts maximal speed of servo*/
    virtual double getMaxVel();

    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max,
        however 0 is just in the center of min and max
    */
    virtual void set(double pos);

    /** returns the position of the servo in ranges [-1, 1] (scaled by min, max, centered)*/
    virtual double get();

  protected:
    double dummy;
    double power;
    double damp;
  };

}
#endif
