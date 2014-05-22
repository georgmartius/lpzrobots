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

#include "joint.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"
#include "angularmotor.h"
#include <selforg/controller_misc.h>

namespace lpzrobots {

  /** general servo motor for 2 axis joints
   */
  class TwoAxisServo  : virtual public Sensor, virtual public Motor {
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
    virtual double get1() const {
      double pos =  joint->getPosition1();
      if(pos > 0){
        pos /= max1;
      }else{
        pos /= -min1;
      }
      return pos;
    }

    /** returns the position of the servo (joint) of 2. axis in ranges [-1, 1] (scaled by min2, max2)*/
    virtual double get2() const {
      double pos =  joint->getPosition2();
      if(pos > 0){
        pos /= max2;
      }else{
        pos /= -min2;
      }
      return pos;
    }

    /** returns the positions of the joint in ranges [-1, 1] (scaled by min, max)*/
    void get(double& p1, double& p2) const {
      p1=get1();
      p2=get2();
    }

    // --- Sensor interface ---
    virtual void init(Primitive* own, Joint* joint = 0) override { // and Motor interface
      if(joint!=0) {
        this->joint=dynamic_cast<TwoAxisJoint*>(joint);
      }
      assert(this->joint);
    }

    virtual bool sense(const GlobalData& globaldata) override { return true;};
    virtual int getSensorNumber() const override {
      return 2;
    }
    virtual std::list<sensor> getList() const { return getListOfArray();};
    virtual int get(sensor* sensors, int length) const {
      assert(length>1);
      sensors[0]=get1();
      sensors[1]=get2();
      return 2;
    }

    // --- Motor interface ---
    virtual int getMotorNumber() const override { return 2;};

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
      assert(length>1);
      set(values[0], values[1]);
      return 2;
    };

    // --- Parameters ---

    /** adjusts the power of the servo*/
    virtual void setPower(double power1, double power2) {
      pid1.KP = power1;
      pid2.KP = power2;
    };

    /** returns the power of the servo*/
    virtual void setPower1(double power1) {
      pid1.KP = power1;
    };

    /** returns the power of the servo*/
    virtual void setPower2(double power2) {
      pid2.KP = power2;
    };

    /** returns the power of the servo*/
    virtual double getPower1() {
      return pid1.KP;
    };
    /** returns the power of the servo*/
    virtual double getPower2() {
      return pid2.KP;
    };

    /** returns the damping of the servo (axis 1) */
    virtual double getDamping1() {
      return pid1.KD;
    };

    /** returns the damping of the servo (axis 2) */
    virtual double getDamping2() {
      return pid2.KD;
    };

    virtual TwoAxisJoint* getJoint() {
      return joint;
    }

    /** sets the damping of the servo (axis 1) */
    virtual void setDamping1(double damp) {
      pid1.KD = damp;
    };

    /** sets the damping of the servo (axis 1) */
    virtual void setDamping2(double damp) {
      pid2.KD = damp;
    };

    /** sets the damping of the servo (both axis) */
    virtual void setDamping(double _damp) {
      setDamping1(_damp);
      setDamping2(_damp);
    };

    /** returns the damping of the servo*/
    virtual double& offsetCanceling() {
      return pid1.KI;
    };

    virtual void setMinMax1(double _min, double _max){
      min1=_min;
      max1=_max;
      joint->setParam(dParamLoStop, _min  - abs(_min) * (jointLimit-1));
      joint->setParam(dParamHiStop, _max  + abs(_max) * (jointLimit-1));
    }

    virtual void setMinMax2(double _min, double _max){
      min2=_min;
      max2=_max;
      joint->setParam(dParamLoStop2, _min  - abs(_min) * (jointLimit-1));
      joint->setParam(dParamHiStop2, _max  + abs(_max) * (jointLimit-1));
    }

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel) {
      this->maxVel = maxVel;
    };
    /** adjusts maximal speed of servo*/
    virtual double getMaxVel() {
      return maxVel;
    };


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
    virtual void set(double pos1, double pos2) override ;

    /** returns the position of the servo (joint) of 1. axis in ranges [-1, 1]
        (scaled by min1, max1, centered)*/
    virtual double get1() const override {
      double pos =  joint->getPosition1();
      return 2*(pos-min1)/(max1-min1) - 1;
    }


    /** returns the position of the servo (joint) of 2. axis in ranges [-1, 1]
        (scaled by min2, max2, centered)*/
    virtual double get2() const override {
      double pos =  joint->getPosition2();
      return 2*(pos-min2)/(max2-min2) - 1;
    }

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

    virtual void init(Primitive* own, Joint* joint = 0) override {
      if(joint) { assert(joint==this->joint); } // we cannot attach the servo to a new joint
    }


    virtual void setPower(double _power1, double _power2) override;

    virtual void setPower1(double _power1) override;

    virtual void setPower2(double _power2) override;

    virtual double getPower1() override  {
      return power1;
    };
    virtual double getPower2() override {
      return power2;
    };

    virtual double getDamping1() override  {
      return damp;
    };
    virtual double getDamping2() override  {
      return damp;
    };
    virtual void setDamping1(double _damp) override {
      damp = clip(_damp,0.0,1.0);
    };
    virtual void setDamping2(double _damp) override  {
      setDamping1(_damp);
    };

    /** offetCanceling does not exist for this type of servo */
    virtual double& offsetCanceling() override  {
      dummy=0;
      return dummy;
    };

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel) override {
      this->maxVel = maxVel;
      pid1.KP=maxVel/2;
      pid2.KP=maxVel/2;
    };
    /** adjusts maximal speed of servo*/
    virtual double getMaxVel() override  {
      return maxVel;
    };


    /** sets the set point of the servo.
        Position must be between -1 and 1. It is scaled to fit into min, max,
        however 0 is just in the center of min and max
    */
    virtual void set(double pos1, double pos2) override ;

  protected:
    AngularMotor2Axis motor;
    double dummy;
    double damp;
    double power1;
    double power2;
  };


}
#endif
