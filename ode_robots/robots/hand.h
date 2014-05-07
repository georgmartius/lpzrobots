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
#ifndef __HAND_H
#define __HAND_H

#include "oderobot.h"
#include <selforg/configurable.h>
#include "primitive.h"
#include "joint.h"
#include "angularmotor.h"
#include "oneaxisservo.h"

#include "primitive.h"
#include "osgforwarddecl.h"
#include "axis.h"

#include "irsensor.h"
#include "raysensorbank.h"
#include "raysensor.h"

namespace lpzrobots {

  enum Motor_type
    {
      With_servo_motor,
      Without_servo_motor
    };

  enum IrSensor_Type{
    irDrawAll,
    irBack,
    irSide,
    irFront
  };

  // struct containing geom and body for each beam (= box, (cappped)cylinder, sphere)
  typedef struct {

    public:
    double velocity;
    double power;       // used when non-servo motor is used
    double servo_motor_Power;  // used when servo motor is used
    bool show_contacts;
    enum Motor_type set_typ_of_motor;
    double factorSensor;
    bool fix_palm_joint;
    bool one_finger_as_one_motor;
    bool draw_joints;
    bool showFingernails;
    double fingerJointBendAngle;
    bool initWithOpenHand; // init hand with open or half closed hand

    //---------------InfrarRedSensor--------------------------
    double irRange;
    bool ir_sensor_used;
    bool irs_at_fingertip;
    bool irs_at_fingertop;
    bool irs_at_fingercenter;
    bool irs_at_fingerbottom;
    enum RaySensor::rayDrawMode ray_draw_mode; // for possible modes see sensors/raysensor.h
  } HandConf;

  enum GripMode{
    lateral,
    precision
  };

  /**
   * Artificial Hand
   *
   */
  class Hand : public OdeRobot{
  public:

    /**
     * constructor of hand
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration of robot
     */
    Hand(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const HandConf& conf, const std::string& name);

    static HandConf getDefaultConf()
      {
        HandConf conf;
        conf.velocity = 0.2;
        conf.power = 5;
        conf.servo_motor_Power = 0.1;
        conf.show_contacts = true;
        conf.set_typ_of_motor = Without_servo_motor;
        conf.irRange = 2;
        conf.ir_sensor_used=true;
        conf.irs_at_fingerbottom=true;
        conf.irs_at_fingercenter=true;
        conf.irs_at_fingertop =true;
        conf.irs_at_fingertip =false;
        conf.ray_draw_mode=RaySensor::drawAll;
        conf.factorSensor=1.0;
        conf.fix_palm_joint=true;
        conf.one_finger_as_one_motor=false;
        conf.draw_joints=false;
        conf.showFingernails=true;
        conf.fingerJointBendAngle=M_PI/2;
        conf.initWithOpenHand=true; // init with open hand
        return conf;
      }


    /**
     * update the subcomponents
     */
    virtual void update();

    /**
     * sets the pose of the vehicle
     * @param pose desired 4x4 pose matrix
     */
    virtual void placeIntern(const osg::Matrix& pose);

    virtual void sense(GlobalData& globalData) override;

    /** returns actual sensorvalues
        @param sensors sensors scaled to [-1,1]
        @param sensornumber length of the sensor array
        @return number of actually written sensors
    */
    virtual int getSensorsIntern(sensor* sensors, int sensornumber);

    /** sets actual motorcommands
        @param motors motors scaled to [-1,1]
        @param motornumber length of the motor array
    */
    virtual void setMotorsIntern(const double* motors, int motornumber);

    /** returns number of sensors
     */
    virtual int getSensorNumberIntern();

    /** returns number of motors
     */
    virtual int getMotorNumberIntern();

    /** returns a vector with the positions of all segments of the robot
        @param poslist vector of positions (of all robot segments)
        @return length of the list
    */
    //  virtual int getSegmentsPosition(vector<Position> &poslist);

    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);


  protected:
    /**
     * Returns the palm as the main object of the robot,
     * which is used for position and speed tracking.
     */
    virtual Primitive* getMainPrimitive() const {
      if(!objects.empty()){
        return (objects[0]); // returns forearm for fixation
        //return (objects[1]); // returns palm
      }else return 0;
    }


  private:

    /**
     * creates the hand at the desired pose
     * @param pose 4x4 pose matrix
     */
    virtual void create(const osg::Matrix& pose);

    /**
     * destroys robot and space
     */
    virtual void destroy();

    static void mycallback(void *data, dGeomID o1, dGeomID o2);

    /** true if robot was created */
    bool created;

  protected:

    /** configuration of hand */
    HandConf conf;

    /** vector containing OSGPrimitives */
    std::vector <OSGPrimitive*> osg_objects;
    /** vector containing Primitivesinfrared sensors */
    std::vector <IRSensor*> ir_sensors;

    /** true if contact joint is created  */
    bool contact_joint_created;

    //std::vector <HingeServo*> servos;
    //objects.reserve(number_beams);

    /** vector of the joints used in hand */


    /** vector of the angular motors */
    std::vector <AngularMotor*> frictionmotors;

    /** vector of the used hinge servos*/
    std::vector <HingeServo*> servos;

    /**  a collection of ir sensors */
    RaySensorBank irSensorBank;


    /** space containing the hand */
    dSpaceID hand_space;

    //Beam beam[number_beams]; // array of elements (rectangle and cylinders)
    //dJointID joint[number_joints];  // array containg "normal" joints for connecting the elementsconf
    //dJointID fix_joint[number_fix_joints]; //joints for keeping index, middle, ring and little finger together to achieve mor prosthetic like motion

    /** motorjoint for actuating the forearm_palm joint (ball joint) */
    AngularMotor* palm_motor_joint;

    /** motorjoint for actuating the palm_thumb joint (ball joint) */
    AngularMotor* thumb_motor_joint;

    /** Hinge Joint between thumb_buttom and thumb_top*/
    HingeJoint* thumb_bt;

    /** Hinge Joint between buttom, center and top part of the index finger*/
    Joint *palm_index, *index_bc, *index_ct;

    /** Hinge Joint between buttom, center and top part of the middle finger*/
    Joint *palm_middle, *middle_bc, *middle_ct;

    /** Hinge Joint between buttom, center and top part of the ring finger*/
    Joint *palm_ring, *ring_bc, *ring_ct;

    /** Hinge Joint between buttom, center and top part of the little finger*/
    Joint *palm_little, *little_bc, *little_ct;



    /** for handling lateral and precision grip modes */
    GripMode gripmode;



    /** initial position of robot */
    Position initial_pos;


    Pos oldp;

    int sensorno;
    int motorno;
    int sensor_number;
    paramval factorForce;
    paramval frictionGround;

    double velocity;


  };

}

#endif

