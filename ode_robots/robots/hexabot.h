/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *   Yuichi Ambe  <amby dot yu at gmail dot com>                           *
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
// 20140928_Revised to adopt to the georg lpzrobots master
//          Torque sensor is disabled (I did not use it).
//          Touch sensor is improved to adopt.
//          It seems to work well and converges to the two fixed points.
//
// 20140509_Revised to add y_trans_center
//

#ifndef __HEXABOT_H
#define __HEXABOT_H

#include "oderobot.h"
#include "raysensorbank.h"

#include "primitive.h"
#include <selforg/inspectable.h>
#include <ode_robots/contactsensor.h>
#include <ode_robots/torquesensor.h>
#include "hexabotsensormotordefinition.h"

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/twoaxisservo.h>
#include <ode_robots/spring.h>

#include "osgprimitive.h"

using namespace HEXABOT;
using namespace lpzrobots;

// If you want to use previous main programs (such as the version under Ver 6)
//  you should define this words
//#define HEXABOT_MAIN_VER_1_6

namespace lpzrobots {

// This is the improved Transform class
//  The aim is to get the position and some information of child object
//
class ImpTransform2 : public Transform{
public:
  // constructor
  ImpTransform2(Primitive* parent, Primitive* child, const Pose& pose);
  // destructor
  virtual ~ImpTransform2();

  // get Child pose
  Pose getChildPose();
};


/***********************************************************************

 Here, I determine the parameters of the robot Hexabot

    _     Joint Joint Joint
   /     \    TC    CT    FT
  |       |== o === | === | ======  Leg
   \  _  / Shld Coxa  Femur  Tibia
    Body    Link  Link   Link

************************************************************************/




/** Robot HEXABOT :-)
    3 legged robot which could be easily connected each other.
*/
class Hexabot : public OdeRobot, public Inspectable {
public:
  // Leg location enum
  enum LegPos {
    L1, L2, L3, L4, L5, L6, LEG_POS_MAX
  };
  // leg use mode (only Leg in this simulation)
  enum LegPosUsage {
    LEG, WHEEL, UNUSED
  };
  // Leg joint type enum
  enum LegJointType {
    // thoroca-coxal joint for forward (+) and backward (-) movements
    TC,
    // coxa-trochanteral joint for elevation (+) and depression (-) of
    // the leg
    CT,
    // femur-tibia joints for extension (+) and flexion (-) of the
    // tibia
    FT,
    // maximum value, used for iteration
    LEG_JOINT_TYPE_MAX
  };
  // motor name of hexabot
  typedef HexabotMotorNames MotorName;
  // sensor name of hexabot
  typedef HexabotSensorNames SensorName;

public: // Functions
  // Constructor
  Hexabot(const OdeHandle& odehandle, const OsgHandle& osgHandle,
          const HexabotConf& conf, const std::string& name);
  // get default configuration (static func because it could be used before construction of the class)
  static HexabotConf getDefaultConf();

  // get Motor name from legPos and joint name
  static MotorName getMotorName(LegPos leg, LegJointType joint);

  //destructor
  virtual ~Hexabot();

  /**
   * updates the OSG nodes of the vehicle
   */
  virtual void update();

  /** sets the pose of the vehicle
      @param pose desired 4x4 pose matrix
  */
  virtual void placeIntern(const osg::Matrix& pose);

  /** gets Primitives of Leg tibia
      @param LegPos Number
  */
  Primitive* getTibiaPrimitive(LegPos leg);


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
  virtual void setMotorsIntern(const motor* motors, int motornumber);

  /** returns number of sensors
   */
  virtual int getSensorNumberIntern(){
    return sensorno;
  };

  /** returns number of motors
   */
  virtual int getMotorNumberIntern(){
    return motorno;
  };

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments)
      @return length of the list
  */
  virtual int getSegmentsPosition(std::vector<Position> &poslist);


  /** this function is called in each timestep. It should perform robot-internal checks,
    like space-internal collision detection, sensor resets/update etc.
    @param globalData structure that contains global data from the simulation environment
  */
  virtual void doInternalStuff(GlobalData& globalData);

  virtual void sense(GlobalData &globalData) override;

  virtual double& getSumForce() { return sumForce; }

  virtual double& getContactPoints() { return contactPoints; }

protected: // Functions
  /** creates vehicle at desired pose
      @param pose 4x4 pose matrix
  */
  virtual void create(const osg::Matrix& pose);

  /** destroys vehicle and space
   */
  virtual void destroy();
  static void mycallback(void *data, dGeomID o1, dGeomID o2);

  /**
   * Assign a human readable name to a motor. This name is used for the
   * associated inspectable value as used e.g. in guilogger.
   *
   * @param motorNo index of the motor (for standard motors defined by
   *        the MotorName enum)
   * @param name human readable name for the motor
   */
  void nameMotor(const int motorNo, const char* name);

  /**
   * Assign a human readable name to a sensor. This name is used for the
   * associated inspectable value as used e.g. in guilogger.
   *
   * @param motorNo index of the motor (for standard motors defined by
   *        the SensorName enum)
   * @param name human readable name for the sensor
   */
  void nameSensor(const int sensorNo, const char* name);


  // getTorqueSensorData
  sensor getTorqueData(TorqueSensor*  torqueSensor);

  // convert Pose Matrix(Quatanion) to the roll, pitch, yaw angle (rad)
  osg::Vec3d convert_Quat_to_RollPitchYaw(osg::Quat quat);

  // calculate COG Position
  osg::Vec3d calc_COGPosition(void);


  /**
   * Inspectable interface
   */
  /*
  virtual std::list<iparamkey> getInternalParamNames() const  { return std::list<iparamkey>(); }

  virtual std::list<iparamval> getInternalParams() const { return std::list<iparamval>(); }*/
  /*
  virtual std::list<Inspectable::iparamkey> getInternalParamNames() const;

  virtual std::list<Inspectable::iparamval> getInternalParams() const;
  */

protected: // Values
  // config param
  HexabotConf conf;

  //! Leg struct
  // Contains Objects, joints and servos for each Leg
  struct Leg{
    Leg(); // constructor, it make all of the value "0" !!
    HingeJoint* tcJoint;
    HingeJoint* ctJoint;
    HingeJoint* ftJoint;
    SliderJoint* footJoint;
    OneAxisServo* tcServo;
    OneAxisServo* ctServo;
    OneAxisServo* ftServo;
    Spring* footSpring;
    ImpTransform2* shoulder;//shoulder trans thats object
    Primitive* shoulderBox;
    Primitive* coxa;
    Primitive* femur;
    Primitive* tibia;
    ImpTransform2* foot;
    Primitive* footSphere;
  };

  //! Trunk struct
  // Contains Objects for Body (two hexagonal plate)

  struct Trunk{
    Trunk(); // constructor, it make all of the value "0"
    Primitive* tPlate[2];
    Primitive* tTrans;
    ImpTransform2* tUpTrans;
  };


  double contactPoints;

  // created flag
  bool created;      // true if robot was created

  /** typedefs */
  typedef std::map<LegPos, HingeJoint*> HingeJointMap;
  typedef std::map<LegPos, Leg> LegMap;
  typedef std::map<LegPos, ContactSensor*> LegContactMap;
  typedef std::map<MotorName, OneAxisServo*> MotorMap;
  typedef std::map<MotorName, TorqueSensor*> MTorqMap;

  //typedef std::map<LegPos, LegPosUsage> LegPosUsageMap;
  //typedef std::map<LegPos, IRSensor*> LegIRSensorMap;
  typedef std::vector<Primitive*> PrimitiveList;
  typedef std::vector<Joint*> JointList;
  typedef std::vector<OneAxisServo*> ServoList;

  // tempolary
  int sensorno;      //number of sensors
  int motorno;       // number of motors

  // body
  Trunk trunk;

  // information on all legs
  LegMap legs;

  // all the objects
  //PrimitiveList objects;

  // all the joints
  //JointList joints;

  // passive servos without a Motorname
  ServoList passiveServos;

  // contains all active servos
  MotorMap servos;


  // sensors >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  // leg contact sensors
  LegContactMap legContactSensors;
  MTorqMap motorTorqSensors;



  //RaySensorBank irSensorBank; // a collection of ir sensors

  bool visForce; // decides if contact force is made visible in guilogger
  double sumForce; // stores the contact force made by collisions with external objects

  // to store the value of servo temporally
  //std::vector<lpzrobots::OneAxisServo*> servo;

};

}


#endif

