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
#ifndef __SERVO2_H
#define __SERVO2_H

#include "pid.h"
#include "odehandle.h"

namespace lpzrobots {

  class TwoAxisJoint;

  /** general servo motor for 2 axis joints
   */
  class TwoAxisServo {
  public:
    /** min and max values are understood as travel bounds. Min should be less than 0.*/

    TwoAxisServo(TwoAxisJoint* joint, double _min1, double _max1, double power1,
                 double _min2, double _max2, double power2,
                 double damp=0.2, double integration=2, double maxVel=10.0,
                 double jointLimit = 1.3, bool minmaxCheck=true);

    virtual ~TwoAxisServo();

    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max
    */
    virtual void set(double pos1, double pos2);

    /** returns the position of the servo (joint) of 1. axis in ranges [-1, 1] (scaled by min1, max1)*/
    virtual double get1();

    /** returns the position of the servo (joint) of 2. axis in ranges [-1, 1] (scaled by min2, max2)*/
    virtual double get2();

    /** returns the positions of the joint in ranges [-1, 1] (scaled by min, max)*/
    void get(double& p1, double& p2);

    /** adjusts the power of the servo*/
    virtual void setPower(double power1, double power2);

    /** returns the power of the servo*/
    virtual void setPower1(double power1);

    /** returns the power of the servo*/
    virtual void setPower2(double power2);

    /** returns the power of the servo*/
    virtual double getPower1();

    /** returns the power of the servo*/
    virtual double getPower2();

    /** returns the damping of the servo (axis 1) */
    virtual double getDamping1();

    /** returns the damping of the servo (axis 2) */
    virtual double getDamping2();

    virtual TwoAxisJoint* getJoint();

    /** sets the damping of the servo (axis 1) */
    virtual void setDamping1(double damp);

    /** sets the damping of the servo (axis 1) */
    virtual void setDamping2(double damp);

    /** sets the damping of the servo (both axis) */
    virtual void setDamping(double _damp);

    /** returns the damping of the servo*/
    virtual double& offsetCanceling();

    virtual void setMinMax1(double _min, double _max);

    virtual void setMinMax2(double _min, double _max);

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel);

    /** adjusts maximal speed of servo*/
    virtual double getMaxVel();


  protected:
    TwoAxisJoint* joint;
    double min1;
    double max1;
    double min2;
    double max2;
    PID pid1;
    PID pid2;
    double maxVel;
    double jointLimit;
  };

  typedef TwoAxisServo UniversalServo;


  /** general servo motor for 2 axis joints with zero position centered
   */
  class TwoAxisServoCentered : public TwoAxisServo {
  public:
    /** min and max values are understood as travel bounds.
        The zero position is (max-min)/2
    */
    TwoAxisServoCentered(TwoAxisJoint* joint, double _min1, double _max1, double power1,
                         double _min2, double _max2, double power2,
                         double damp=0.2, double integration=2, double maxVel=10.0,
                         double jointLimit = 1.3);

    virtual ~TwoAxisServoCentered();

    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max,
        however 0 is just in the center of min and max
    */
    virtual void set(double pos1, double pos2);

    /** returns the position of the servo (joint) of 1. axis in ranges [-1, 1]
        (scaled by min1, max1, centered)*/
    virtual double get1();

    /** returns the position of the servo (joint) of 2. axis in ranges [-1, 1]
        (scaled by min2, max2, centered)*/
    virtual double get2();

  };


  /** general servo motor to achieve position control for 2 axis joints
   *  that internally controls the velocity of the motor (much more stable)
   *  with centered zero position.
   *  The amount of body feeling can be adjusted by the damping parameter
   *   which is understood as a stiffness parameter
   */
  class TwoAxisServoVel : public TwoAxisServoCentered {
  public:
    /** min and max values are understood as travel bounds.
        The zero position is (max-min)/2
        @param power is the maximal torque the servo can generate
        @param maxVel is understood as a speed parameter of the servo.
        @param damp adjusts the power of the servo in dependence of the distance
         to the set point. This regulates the stiffness and the body feeling
          0: the servo has no power at the set point (maximal body feeling);
          1: is servo has full power at the set point: perfectly damped and stiff.
    */
    TwoAxisServoVel(const OdeHandle& odeHandle,
                    TwoAxisJoint* joint, double _min1, double _max1, double power1,
                    double _min2, double _max2, double power2,
                    double damp=0.05, double maxVel=10.0, double jointLimit = 1.3);

    virtual ~TwoAxisServoVel();

    virtual void setPower(double _power1, double _power2);

    virtual void setPower1(double _power1);

    virtual void setPower2(double _power2);

    virtual double getPower1();

    virtual double getPower2();

    virtual double getDamping1();

    virtual double getDamping2();

    virtual void setDamping1(double _damp);

    virtual void setDamping2(double _damp);

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
    virtual void set(double pos1, double pos2);

  protected:
    //    AngularMotor2Axis motor;
    double dummy;
    double damp;
    double power1;
    double power2;
  };


}
#endif
