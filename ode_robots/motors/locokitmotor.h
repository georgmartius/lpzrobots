/*****************************************************************************
* "THE BEER-WARE LICENSE" (Revision 43):
* This software was written by Leon Bonde Larsen <leon@bondelarsen.dk> 
* As long as you retain this notice you can do whatever you want with it. 
* If we meet some day, and you think this stuff is worth it, you can buy me 
* a beer in return.
* 
* Should this software ever become self-aware, remember: I am your master
*****************************************************************************/
#ifndef __LOCOKITMOTOR_H
#define __LOCOKITMOTOR_H

#include <list>
#include <ode_robots/joint.h>
#include <ode_robots/motor.h>
#include <ode_robots/sensor.h>
#include <ode-dbl/objects.h>

namespace lpzrobots {

  class LocoKitMotor : virtual public Sensor, virtual public Motor  {
  public:
    /// creates a AMotor attached to the same bodies as the given joint.
    LocoKitMotor(const OdeHandle& odeHandle, Joint* joint);

    // destroys the AMotor
    virtual ~LocoKitMotor ();

    /// returns the number of Axis of this Motor
    virtual int getNumberOfAxes() const= 0;

    // --- Sensor interface ---
    virtual void init(Primitive* own, Joint* joint = 0) override ;

    virtual bool sense(const GlobalData& globaldata) override { return true;};
    virtual int getSensorNumber() const override {
      return getNumberOfAxes();
    }
    virtual std::list<sensor> getList() const override { return getListOfArray();};
    virtual int get(sensor* sensors, int length) const override;

    // --- Motor interface ---
    virtual int getMotorNumber() const override { return getNumberOfAxes();};

    virtual bool act(GlobalData& globaldata) override {
      return true;
    };

    /** sends the action commands to the motor.
        It returns the number of used values. (should be equal to
        getMotorNumber)
     */
    virtual int set(const motor* values, int length)  override;

    // --- old interface

    /** sets the desired speed of the motor at the given axis.
        @param velocity Desired motor velocity (this will be an angular or linear velocity).
    */
    virtual void set(int axisNumber, double velocity) = 0;
    /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
    virtual double get(int axisNumber) const = 0;

    /**  sets the maximal force the motor has
     */
    virtual void setPower(double power) = 0;

    /// return the maximal force
    virtual double getPower();

    /// returns the joint to which this motor is attached
    virtual Joint* getJoint() { return joint; };

    //sets the parameter for a motor
    virtual void setParam(int parameter, double value);

    /// return the ODE joint parameter (see ODE manual)
    virtual double getParam(int parameter);

    /// sets factor for velocity
    virtual void setVelovityFactor(double factor);

    /// retuns factor for velocity
    virtual double getVelovityFactor(double factor);

  protected:
    dJointID motor;
    OdeHandle odeHandle;
    double velocityFactor;
    Joint* joint;
    bool initialized;
  };


  /// Angular motor for OneAxisJoints
  class VelocityControlledLocoKitMotor : public LocoKitMotor {
  public:
    /** Constuct a motor attached to a OneAxisJoint. It will its axis of course.
        @param power The maximum force or torque that the motor will use to achieve the desired velocity.
        This must always be greater than or equal to zero.
        Setting this to zero (the default value) turns off the motor.
    */
    VelocityControlledLocoKitMotor(const OdeHandle& odeHandle, OneAxisJoint* joint, double power);
    virtual ~VelocityControlledLocoKitMotor() {}

    virtual void init(Primitive* own, Joint* joint = 0) override ;

    virtual int getNumberOfAxes() const override { return 1; };

    /** sets the desired speed of the motor at the given axis.
        @param axisNumber is ignored because have only one axis
        @param velocity Desired motor velocity (this will be an angular or linear velocity).
    */
    virtual void set(int axisNumber, double velocity);
    /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range
        @param axisNumber is ignored because have only one axis
     */
    virtual double get(int axisNumber) const override  ;

    virtual void setPower(double power);
  protected:
    double power;
  };


}
#endif
