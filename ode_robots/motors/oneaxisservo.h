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

#include "joint.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"
#include "angularmotor.h"
#include <selforg/controller_misc.h>

namespace lpzrobots {

  /** general servo motor to achieve position control
   */
  class OneAxisServo : virtual public Sensor, virtual public Motor {
  public:
    /** min and max values are understood as travel bounds. Min should be less than 0.*/

    OneAxisServo(OneAxisJoint* joint, double _min, double _max,
                 double power, double damp=0.2, double integration=2,
                 double maxVel=10.0,
                 double jointLimit = 1.3, bool minmaxCheck = true);

    virtual ~OneAxisServo();

    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max
    */
    virtual void set(double pos);

    /** returns the position of the joint in the range [-1, 1] (scaled by min, max)*/
    virtual double get() const {
      double pos =  joint->getPosition1();
      if(pos > 0){
        pos /= max;
      }else{
        pos /= -min;
      }
      return pos;
    }

    // --- Sensor interface ---
    virtual void init(Primitive* own, Joint* joint = 0) override { // and Motor interface
      if(joint!=0) {
        this->joint=dynamic_cast<OneAxisJoint*>(joint);
      }
      assert(this->joint);
    }

    virtual bool sense(const GlobalData& globaldata) override { return true;};
    virtual int getSensorNumber() const override {
      return 1;
    }
    virtual std::list<sensor> getList() const { return getListOfArray();};
    virtual int get(sensor* sensors, int length) const {
      assert(length>0);
      sensors[0]=get();
      return 1;
    }

    // --- Motor interface ---
    virtual int getMotorNumber() const override { return 1;};

    virtual bool act(GlobalData& globaldata) override {
      // here we should apply the forces etc, but due to backwards compatibility this remains in set()
      // which is also called each timestep.
      return true;
    };

    /** sends the action commands to the motor.
        It returns the number of used values. (should be equal to
        getMotorNumber)
     */
    virtual int set(const motor* values, int length)  override {
      assert(length>0);
      set(values[0]);
      return 1;
    };


    // --- Parameters ---
    virtual void setMinMax(double _min, double _max){
      min=_min;
      max=_max;
      joint->setParam(dParamLoStop, min  - abs(min) * (jointLimit-1));
      joint->setParam(dParamHiStop, max  + abs(max) * (jointLimit-1));
    }

    /** adjusts the power of the servo*/
    virtual void setPower(double power) {
      pid.KP = power;
    };

    /** returns the power of the servo*/
    virtual double getPower() {
      return pid.KP;
    };

    /** returns the damping of the servo*/
    virtual double getDamping() {
      return pid.KD;
    }
    /** sets the damping of the servo*/
    virtual void setDamping(double damp) {
      pid.KD = damp;
    };

    /** returns the integration term of the PID controller of the servo*/
    virtual double& offsetCanceling() {
      return pid.KI;
    };

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel) {
      this->maxVel = maxVel;
    };
    /** adjusts maximal speed of servo*/
    virtual double getMaxVel() {
      return maxVel;
    };


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

    virtual ~OneAxisServoCentered(){}

    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max,
        however 0 is just in the center of min and max
    */
    virtual void set(double pos) override ;

    /** returns the position of the slider in ranges [-1, 1] (scaled by min, max, centered)*/
    virtual double get() const override {
      double pos =  joint->getPosition1();

      return 2*(pos-min)/(max-min) - 1;
    }

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
         to the set point (current control error).
         This regulates the stiffness and the body feeling
          0: the servo has no power at the set point (maximal body feeling);
          1: is servo has full power at the set point: maximal stiffness, perfectly damped.
    */
    OneAxisServoVel(const OdeHandle& odeHandle,
                    OneAxisJoint* joint, double _min, double _max,
                    double power, double damp=0.05, double maxVel=20,
                    double jointLimit = 1.3);

    virtual ~OneAxisServoVel();

    virtual void init(Primitive* own, Joint* joint = 0) override {
      if(joint) { assert(joint==this->joint); } // we cannot attach the servo to a new joint
    }

    /** adjusts the power of the servo*/
    virtual void setPower(double _power) override;

    /** returns the power of the servo*/
    virtual double getPower() override {
      return power;
    };
    virtual double getDamping() override {
      return damp;
    };
    virtual void setDamping(double _damp) override {
      damp = clip(_damp,0.0,1.0);
    };
    /** offetCanceling does not exist for this type of servo */
    virtual double& offsetCanceling() override {
      dummy=0;
      return dummy;
    };

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel) override {
      this->maxVel = maxVel;
      pid.KP=maxVel/2;
    };
    /** adjusts maximal speed of servo*/
    virtual double getMaxVel() override {
      return maxVel;
    };

    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max,
        however 0 is just in the center of min and max
    */
    virtual void set(double pos) override ;

    /** returns the position of the servo in ranges [-1, 1] (scaled by min, max, centered)*/
    virtual double get() const override {
      double pos =  joint->getPosition1();
      return 2*(pos-min)/(max-min) - 1;
    }
  protected:
    AngularMotor1Axis motor;
    double dummy;
    double power;
    double damp;
  };


  /** Servo for sliders to achieve position control. Like OneAxisServoVel but
   *  suitable for sliders
   * @see OneAxisServoVel
   */
  class SliderServoVel : public OneAxisServo {
  public:
    /** min and max values are understood as travel bounds.
        The zero position is (max-min)/2
        @param power is the maximal torque the servo can generate
        @param maxVel is understood as a speed parameter of the servo.
        @param damp adjusts the power of the servo in dependence of the distance
         to the set point (current control error).
         This regulates the stiffness and the body feeling
          0: the servo has no power at the set point (maximal body feeling);
          1: is servo has full power at the set point: maximal stiffness, perfectly damped.
    */
    SliderServoVel(const OdeHandle& odeHandle,
                   OneAxisJoint* joint, double _min, double _max,
                   double power, double damp=0.05, double maxVel=20,
                   double jointLimit = 1.3);

    virtual ~SliderServoVel();

    /** adjusts the power of the servo*/
    virtual void setPower(double _power) override;

    /** returns the power of the servo*/
    virtual double getPower() override {
      return power;
    };
    virtual double getDamping() override {
      return damp;
    };
    virtual void setDamping(double _damp) override {
      damp = clip(_damp,0.0,1.0);
    };
    /** offetCanceling does not exist for this type of servo */
    virtual double& offsetCanceling() override {
      dummy=0;
      return dummy;
    };

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel) override {
      this->maxVel = maxVel;
      pid.KP=maxVel/2;
    };
    /** adjusts maximal speed of servo*/
    virtual double getMaxVel()override {
      return maxVel;
    };

    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max,
        however 0 is just in the center of min and max
    */
    virtual void set(double pos) override ;

    /** returns the position of the servo in ranges [-1, 1] (scaled by min, max, centered)*/
    virtual double get() const override {
      double pos =  joint->getPosition1();
      return 2*(pos-min)/(max-min) - 1;
    }

  protected:
    double dummy;
    double power;
    double damp;
  };

}
#endif
