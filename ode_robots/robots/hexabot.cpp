/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Yuichi Ambe  <amby dot yu at gmail dot com>                          *
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

#include <iostream>
#include <ode-dbl/ode.h>
#include <assert.h>
#include <osg/Matrix>

#include "hexabot.h"
#include "irsensor.h"
#include "osgprimitive.h"

using namespace osg;
using namespace std;
using namespace HEXABOT;

#define DEBUG_MODE

namespace lpzrobots {

// this is the part of the ImpTransform2
  // constructor
  ImpTransform2::ImpTransform2(Primitive* parent, Primitive* child, const Pose& pose):Transform(parent, child, pose){}

  // destructor
  ImpTransform2::~ImpTransform2(){}

  // function]
  // thorough this function, we can get position or rotation or something about child
  Pose ImpTransform2::getChildPose(){
    Pose cPose;
    // the vector is multiplied from left hand, it was mentioned in another point
    cPose = pose * this->getPose();
    return cPose;
  }

  // constructor:
  // - give handle for ODE and OSG stuff, and default configuration
  Hexabot::Hexabot(const OdeHandle& odehandle, const OsgHandle& osgHandle, const HexabotConf& conf, const std::string& name):OdeRobot(odehandle,osgHandle,name,"$Id$"),conf(conf),trunk()
  {
    //
    contactPoints=0;
    // robot not created up to now
    created=false;
    sensorno=HEXABOT_SENSOR_MAX;
    motorno= HEXABOT_MOTOR_MAX;// HEXABOT_MOTOR_MAX   modifi


    // name the sensors
    // Leg joint angle sensor
    nameSensor(T1_as, "T1 angle sensor");
    nameSensor(T2_as, "T2 angle sensor");
    nameSensor(T3_as, "T3 angle sensor");
    nameSensor(T4_as, "T4 angle sensor");
    nameSensor(T5_as, "T5 angle sensor");
    nameSensor(T6_as, "T6 angle sensor");

    nameSensor(C1_as, "C1 angle sensor");
    nameSensor(C2_as, "C2 angle sensor");
    nameSensor(C3_as, "C3 angle sensor");
    nameSensor(C4_as, "C4 angle sensor");
    nameSensor(C5_as, "C5 angle sensor");
    nameSensor(C6_as, "C6 angle sensor");

    nameSensor(F1_as, "F1 angle sensor");
    nameSensor(F2_as, "F2 angle sensor");
    nameSensor(F3_as, "F3 angle sensor");
    nameSensor(F4_as, "F4 angle sensor");
    nameSensor(F5_as, "F5 angle sensor");
    nameSensor(F6_as, "F6 angle sensor");

    // Leg contact sensors
    nameSensor(L1_fs, "Leg1 force sensor");
    nameSensor(L2_fs, "Leg2 force sensor");
    nameSensor(L3_fs, "Leg3 force sensor");
    nameSensor(L4_fs, "Leg4 force sensor");
    nameSensor(L5_fs, "Leg5 force sensor");
    nameSensor(L6_fs, "Leg6 force sensor");

    // Leg joint angle sensor
    nameSensor(T1_ts, "T1 torque sensor");
    nameSensor(T2_ts, "T2 torque sensor");
    nameSensor(T3_ts, "T3 torque sensor");
    nameSensor(T4_ts, "T4 torque sensor");
    nameSensor(T5_ts, "T5 torque sensor");
    nameSensor(T6_ts, "T6 torque sensor");

    nameSensor(C1_ts, "C1 torque sensor");
    nameSensor(C2_ts, "C2 torque sensor");
    nameSensor(C3_ts, "C3 torque sensor");
    nameSensor(C4_ts, "C4 torque sensor");
    nameSensor(C5_ts, "C5 torque sensor");
    nameSensor(C6_ts, "C6 torque sensor");

    nameSensor(F1_ts, "F1 torque sensor");
    nameSensor(F2_ts, "F2 torque sensor");
    nameSensor(F3_ts, "F3 torque sensor");
    nameSensor(F4_ts, "F4 torque sensor");
    nameSensor(F5_ts, "F5 torque sensor");
    nameSensor(F6_ts, "F6 torque sensor");

    // attitude sensors
    nameSensor(POSE_r, "roll Pose sensor");
    nameSensor(POSE_p, "pitch Pose sensor");
    nameSensor(POSE_y, "yaw Pose sensor");

    // angular Velocity vector
    nameSensor(W_x, "angular vel x sensor");
    nameSensor(W_y, "angular vel y sensor");
    nameSensor(W_z, "angular vel z sensor");

    // grobal position of the Robot Center
    nameSensor(GPOS_Rx, "Grobal Pos of robot x");
    nameSensor(GPOS_Ry, "Grobal Pos of robot y");
    nameSensor(GPOS_Rz, "Grobal Pos of robot z");

    // grobal speed of the Center
    nameSensor(GSPD_Rx, "Grobal Spd of robot x");
    nameSensor(GSPD_Ry, "Grobal Spd of robot y");
    nameSensor(GSPD_Rz, "Grobal Spd of robot z");

    // grobal position of the COG
    nameSensor(GPOS_COGx, "Grobal COG Pos of robot x");
    nameSensor(GPOS_COGy, "Grobal COG Pos of robot y");
    nameSensor(GPOS_COGz, "Grobal COG Pos of robot z");

    // grobal position of the LegToe
    nameSensor(GPOS_L1x, "Grobal Leg1 toe Pos x");
    nameSensor(GPOS_L1y, "Grobal Leg1 toe Pos y");
    nameSensor(GPOS_L1z, "Grobal Leg1 toe Pos z");
    nameSensor(GPOS_L2x, "Grobal Leg2 toe Pos x");
    nameSensor(GPOS_L2y, "Grobal Leg2 toe Pos y");
    nameSensor(GPOS_L2z, "Grobal Leg2 toe Pos z");
    nameSensor(GPOS_L3x, "Grobal Leg3 toe Pos x");
    nameSensor(GPOS_L3y, "Grobal Leg3 toe Pos y");
    nameSensor(GPOS_L3z, "Grobal Leg3 toe Pos z");
    nameSensor(GPOS_L4x, "Grobal Leg4 toe Pos x");
    nameSensor(GPOS_L4y, "Grobal Leg4 toe Pos y");
    nameSensor(GPOS_L4z, "Grobal Leg4 toe Pos z");
    nameSensor(GPOS_L5x, "Grobal Leg5 toe Pos x");
    nameSensor(GPOS_L5y, "Grobal Leg5 toe Pos y");
    nameSensor(GPOS_L5z, "Grobal Leg5 toe Pos z");
    nameSensor(GPOS_L6x, "Grobal Leg6 toe Pos x");
    nameSensor(GPOS_L6y, "Grobal Leg6 toe Pos y");
    nameSensor(GPOS_L6z, "Grobal Leg6 toe Pos z");

    // name the motors
    nameMotor(T1_m, "T1 motor");
    nameMotor(T2_m, "T2 motor");
    nameMotor(T3_m, "T3 motor");
    nameMotor(T4_m, "T4 motor");
    nameMotor(T5_m, "T5 motor");
    nameMotor(T6_m, "T6 motor");

    nameMotor(C1_m, "C1 motor");
    nameMotor(C2_m, "C2 motor");
    nameMotor(C3_m, "C3 motor");
    nameMotor(C4_m, "C4 motor");
    nameMotor(C5_m, "C5 motor");
    nameMotor(C6_m, "C6 motor");

    nameMotor(F1_m, "F1 motor");
    nameMotor(F2_m, "F2 motor");
    nameMotor(F3_m, "F3 motor");
    nameMotor(F4_m, "F4 motor");
    nameMotor(F5_m, "F5 motor");
    nameMotor(F6_m, "F6 motor");

  //visForce = conf.visForce;
  //if (visForce) {
  //  sumForce=0;
  //}
  }


  Hexabot::~Hexabot(){
    destroy();
  }

  // name sensor
  void Hexabot::nameSensor(const int sensorNo, const char* name) {
    addInspectableDescription("x[" + std::itos(sensorNo) + "]", name);
  }

  // name motor
  void Hexabot::nameMotor(const int motorNo, const char* name) {
    addInspectableDescription("y[" + std::itos(motorNo) + "]", name);
  }


  // Configuration get function static
  HexabotConf Hexabot::getDefaultConf(){
    HexabotConf conf;
    // configuration of rate
    conf.rate = 1.; // we multiple 1 with the length
    // configuration of mass rate
    conf.massRate = 1.;

    // configuration of the COG position (Body center and joint center)
    conf.x_trans_center = 0.;
    conf.y_trans_center = 0.;

    // configuration for dynamixel
    conf.dyna.width = 0.025 /* m */ * conf.rate;
    conf.dyna.length = 0.050 * conf.rate;
    conf.dyna.height = 0.036 * conf.rate;
    conf.dyna.length_axis_to_center = 0.0135 * conf.rate;
    conf.dyna.length_from_axis_to_tip = 0.0265 * conf.rate;
    conf.dyna.mass = 0.0535 * conf.massRate;// kg //0.001;//

    // config. of body
    conf.body.height = 0.003 * conf.rate; // m
    conf.body.length = 0.420 * conf.rate;
    conf.body.width = 0.080 * conf.rate;

    conf.body.mass = (1.0)* conf.massRate; // kg 0.001;//

    // config. of foot
    conf.foot.height = 0.002 * conf.rate; // m
    conf.foot.length = 0.120 * conf.rate;
    conf.foot.width  = 0.038 * conf.rate;
    conf.foot.footRadius = 0.007 * conf.rate;

    conf.foot.mass = 0.035* conf.massRate; // kg 0.001;//

    // config. of joint length
    conf.jLength.length_x_center_to_TCJ = 0.054 * conf.rate; //m
    conf.jLength.length_y_TCJ_to_TCJ = 0.200 * conf.rate;
    conf.jLength.length_TCJ_to_CTJ = 0.068 * conf.rate; //m
    conf.jLength.length_CTJ_to_FTJ = 0.068 * conf.rate;
    conf.jLength.length_FTJ_to_Toe = 0.1145 * conf.rate;

    // config. of servo parameter
    conf.servoParam.TC_angle_MIN = -M_PI * 90. / 180.;
    conf.servoParam.TC_angle_MAX = M_PI * 90. / 180.;
    conf.servoParam.CT_angle_MIN = - 2. * M_PI * 90. / 180.;
    conf.servoParam.CT_angle_MAX = 0.; //M_PI * 90. / 180.;
    conf.servoParam.FT_angle_MIN = 0.;//-M_PI * 90. / 180.;
    conf.servoParam.FT_angle_MAX = 2. * M_PI * 90. / 180.;

    conf.servoParam.power = 50;//20.;
    conf.servoParam.damp = 0.;
    conf.servoParam.integ = 0.;
    conf.servoParam.maxVel = 1.7 * 1.961 * M_PI;


    // all mass calculation
    conf.wholeMass = conf.body.mass + 18. * conf.dyna.mass + 6. * conf.foot.mass;

    return conf;
  }

  // The function to get the motor Name
  Hexabot::MotorName Hexabot::getMotorName(LegPos leg, LegJointType joint){
    if (leg == L1){
      if(joint == TC){
        return T1_m;
      }else if(joint == CT){
        return C1_m;
      }else if(joint == FT){
        return F1_m;
      }
    }else if(leg == L2){
      if(joint == TC){
        return T2_m;
      }else if(joint == CT){
        return C2_m;
      }else if(joint == FT){
        return F2_m;
      }
    }else if(leg == L3){
      if(joint == TC){
        return T3_m;
      }else if(joint == CT){
        return C3_m;
      }else if(joint == FT){
        return F3_m;
      }
    }else if(leg == L4){
      if(joint == TC){
        return T4_m;
      }else if(joint == CT){
        return C4_m;
      }else if(joint == FT){
        return F4_m;
      }
    }else if(leg == L5){
      if(joint == TC){
        return T5_m;
      }else if(joint == CT){
        return C5_m;
      }else if(joint == FT){
        return F5_m;
      }
    }else if(leg == L6){
      if(joint == TC){
        return T6_m;
      }else if(joint == CT){
        return C6_m;
      }else if(joint == FT){
        return F6_m;
      }
    }

    return HEXABOT_MOTOR_MAX;
  }

  /** gets Primitives of Leg tibia
      @param LegPos Number
  */
  Primitive* Hexabot::getTibiaPrimitive(LegPos leg){
    assert(created);
    if(leg < LEG_POS_MAX){
      return legs[leg].tibia;
    }else{
      return 0;
    }
  }


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Hexabot::setMotorsIntern(const motor* motors, int motornumber){
    assert(created);
    assert(motornumber == motorno);

    for (MotorMap::iterator it = servos.begin(); it != servos.end(); it++) {
      MotorName const name = it->first;
      OneAxisServo * const servo = it->second;
      //We set the motor command in the servo
      if(servo){
        servo->set(motors[name]);
      }
    }
  }

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
//   sensor ir_old[4];
//   sensor ir_tmp[4];

  int Hexabot::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created);
    assert(sensornumber == getSensorNumber());

    // angle sensors
    //
    sensors[T1_as] = servos[T1_m] ? servos[T1_m]->get() : 0;
    sensors[T2_as] = servos[T2_m] ? servos[T2_m]->get() : 0;
    sensors[T3_as] = servos[T3_m] ? servos[T3_m]->get() : 0;
    sensors[T4_as] = servos[T4_m] ? servos[T4_m]->get() : 0;
    sensors[T5_as] = servos[T5_m] ? servos[T5_m]->get() : 0;
    sensors[T6_as] = servos[T6_m] ? servos[T6_m]->get() : 0;

    sensors[C1_as] = servos[C1_m] ? servos[C1_m]->get() : 0;
    sensors[C2_as] = servos[C2_m] ? servos[C2_m]->get() : 0;
    sensors[C3_as] = servos[C3_m] ? servos[C3_m]->get() : 0;
    sensors[C4_as] = servos[C4_m] ? servos[C4_m]->get() : 0;
    sensors[C5_as] = servos[C5_m] ? servos[C5_m]->get() : 0;
    sensors[C6_as] = servos[C6_m] ? servos[C6_m]->get() : 0;

    sensors[F1_as] = servos[F1_m] ? servos[F1_m]->get() : 0;
    sensors[F2_as] = servos[F2_m] ? servos[F2_m]->get() : 0;
    sensors[F3_as] = servos[F3_m] ? servos[F3_m]->get() : 0;
    sensors[F4_as] = servos[F4_m] ? servos[F4_m]->get() : 0;
    sensors[F5_as] = servos[F5_m] ? servos[F5_m]->get() : 0;
    sensors[F6_as] = servos[F6_m] ? servos[F6_m]->get() : 0;

    // Contact sensors
    sensors[L1_fs] = legContactSensors[L1] ? legContactSensors[L1]->get() : 0;
    sensors[L2_fs] = legContactSensors[L2] ? legContactSensors[L2]->get() : 0;
    sensors[L3_fs] = legContactSensors[L3] ? legContactSensors[L3]->get() : 0;
    sensors[L4_fs] = legContactSensors[L4] ? legContactSensors[L4]->get() : 0;
    sensors[L5_fs] = legContactSensors[L5] ? legContactSensors[L5]->get() : 0;
    sensors[L6_fs] = legContactSensors[L6] ? legContactSensors[L6]->get() : 0;

    // Torque sensors
    //motorTorqSensors[T0_m]->update();
    sensors[T1_ts] = getTorqueData(motorTorqSensors[T1_m]);
    sensors[T2_ts] = getTorqueData(motorTorqSensors[T2_m]);
    sensors[T3_ts] = getTorqueData(motorTorqSensors[T3_m]);
    sensors[T4_ts] = getTorqueData(motorTorqSensors[T4_m]);
    sensors[T5_ts] = getTorqueData(motorTorqSensors[T5_m]);
    sensors[T6_ts] = getTorqueData(motorTorqSensors[T6_m]);

    sensors[C1_ts] = getTorqueData(motorTorqSensors[C1_m]);
    sensors[C2_ts] = getTorqueData(motorTorqSensors[C2_m]);
    sensors[C3_ts] = getTorqueData(motorTorqSensors[C3_m]);
    sensors[C4_ts] = getTorqueData(motorTorqSensors[C4_m]);
    sensors[C5_ts] = getTorqueData(motorTorqSensors[C5_m]);
    sensors[C6_ts] = getTorqueData(motorTorqSensors[C6_m]);

    sensors[F1_ts] = getTorqueData(motorTorqSensors[F1_m]);
    sensors[F2_ts] = getTorqueData(motorTorqSensors[F2_m]);
    sensors[F3_ts] = getTorqueData(motorTorqSensors[F3_m]);
    sensors[F4_ts] = getTorqueData(motorTorqSensors[F4_m]);
    sensors[F5_ts] = getTorqueData(motorTorqSensors[F5_m]);
    sensors[F6_ts] = getTorqueData(motorTorqSensors[F6_m]);

    //Pose sensor
    osg::Vec3d a = this->convert_Quat_to_RollPitchYaw(this->getMainPrimitive()->getPose().getRotate());
    sensors[POSE_r] = a[0]; // rad angle
    sensors[POSE_p] = a[1];
    sensors[POSE_y] = a[2];

    //angular velocity of center of the robot
    a = this->getMainPrimitive()->getAngularVel();
    sensors[W_x] = a[0]; // angular vel vector
    sensors[W_y] = a[1];
    sensors[W_z] = a[2];

    // grobal position of robot center
    // (lower heagonal plate center position + upper hexagonal plate center position) / 2. is OK
    a = (this->getMainPrimitive()->getPosition() + trunk.tUpTrans->getChildPose().getTrans()) * 1./2.;
    sensors[GPOS_Rx] = a[0]; // grobal position of the robot
    sensors[GPOS_Ry] = a[1];
    sensors[GPOS_Rz] = a[2];

    // grobal speed of robot center
    //a = this->legs[L0].shoulder->getPose().getTrans();
    a = this->getMainPrimitive()->getVel();
    sensors[GSPD_Rx] = a[0]; // grobal spd of the robot
    sensors[GSPD_Ry] = a[1];
    sensors[GSPD_Rz] = a[2];

    // grobal position of the COG
    a = this->calc_COGPosition();
    sensors[GPOS_COGx] = a[0]; // grobal cog of the robot
    sensors[GPOS_COGy] = a[1];
    sensors[GPOS_COGz] = a[2];

    // grobal position of the leg toe
    a = this->legs[L1].foot->getChildPose().getTrans();
    sensors[GPOS_L1x] = a[0];
    sensors[GPOS_L1y] = a[1];
    sensors[GPOS_L1z] = a[2];

    a = this->legs[L2].foot->getChildPose().getTrans();
    sensors[GPOS_L2x] = a[0];
    sensors[GPOS_L2y] = a[1];
    sensors[GPOS_L2z] = a[2];

    a = this->legs[L3].foot->getChildPose().getTrans();
    sensors[GPOS_L3x] = a[0];
    sensors[GPOS_L3y] = a[1];
    sensors[GPOS_L3z] = a[2];

    a = this->legs[L4].foot->getChildPose().getTrans();
    sensors[GPOS_L4x] = a[0];
    sensors[GPOS_L4y] = a[1];
    sensors[GPOS_L4z] = a[2];

    a = this->legs[L5].foot->getChildPose().getTrans();
    sensors[GPOS_L5x] = a[0];
    sensors[GPOS_L5y] = a[1];
    sensors[GPOS_L5z] = a[2];

    a = this->legs[L6].foot->getChildPose().getTrans();
    sensors[GPOS_L6x] = a[0];
    sensors[GPOS_L6y] = a[1];
    sensors[GPOS_L6z] = a[2];

    return HEXABOT_SENSOR_MAX;
  }


  void Hexabot::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // length of foot is added (without this the wheels and half of the robot will be in the ground)
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, conf.foot.length + conf.foot.footRadius * 2. - conf.dyna.width /2.));
    create(p2);
  }

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments)
      @return length of the list
  */
  int Hexabot::getSegmentsPosition(std::vector<Position> &poslist){
    assert(created);
    for (int i=0; i<3; i++){
      poslist.push_back(Position(dBodyGetPosition(objects[i]->getBody())));
    }
    return 3;
  };

  /**
   * updates the osg notes and sensorbank
   */
  void Hexabot::update(){
    assert(created); // robot must exist

    // update object
    for (PrimitiveList::iterator i = objects.begin(); i != objects.end(); i++) {
      if (*i){
        (*i)->update();
      }
    }

    // update joints
    for (JointList::iterator i = joints.begin(); i != joints.end(); i++) {
      if (*i){
        (*i)->update();
      }
    }

    // update sensorbank with infrared sensors
    //irSensorBank.update();
    //this->legs[L1].shoulderBox->update();

    //update contact sensors
    for (int i = 0; i < LEG_POS_MAX; i++) {
      if (legContactSensors[LegPos(i)])
        legContactSensors[LegPos(i)]->update();
    }

  }


  //May be it is called every step??
  void Hexabot::doInternalStuff(GlobalData& globalData){
    //irSensorBank.reset(); // reset sensorbank (infrared sensors)

    // update statistics
    //position = getPosition();

    // reset contact sensors
    for (int i = 0; i < LEG_POS_MAX; i++) {
      if (legContactSensors[LegPos(i)])
        legContactSensors[LegPos(i)]->sense(globalData);

        //motorTorqSensors[getMotorName(LegPos(i), FT)]->sense(globalData);
        //motorTorqSensors[getMotorName(LegPos(i), TC)]->sense(globalData);
        //motorTorqSensors[getMotorName(LegPos(i), CT)]->sense(globalData);
    }

    // reset ir sensors to maximum value
    //irSensorBank->reset();

    // I dont know
    /*
  if (visForce) {
      sumForce=0;
      contactPoints=0;
  }
  */

  }

  // May be it is called every step to sense the sensor
  void Hexabot::sense(GlobalData& globalData) {
    OdeRobot::sense(globalData);
    // reset ir sensors to maximum value
    //irSensorBank->sense(globalData);

    for (int i = 0; i < LEG_POS_MAX; i++) {
      if (legContactSensors[LegPos(i)])
        legContactSensors[LegPos(i)]->sense(globalData);
    }

     // Added sound sensors (3) // sense from environment
    //for(SoundSensor* sensor: soundsensors)
    //sensor->sense(globalData);

  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void Hexabot::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

#ifdef DEBUG_MODE
    cout << "hello, This is the debug mode" << endl;
    cout << "@@@@ First, We create the robot!! @@@@" << endl;
#endif

    /*********************************************************************************:
     *  The ordinal configuration
     *     Environment, color, feature etc/
    ***********************************************************************************/

    // we want legs colliding with other legs, so we set internal collision
    // flag to "false".
    odeHandle.createNewSimpleSpace(parentspace,false);

    // Hexabot color ;-)
    osgHandle.color = Color(0/255., 30./255., 0./255., 1.0f);

    // Joint color ;-)
    OsgHandle dynaHandle(osgHandle);
    dynaHandle.color = Color(204/255., 51/255., 0./255., 1.0f);
    //dynaHandle.color = Color(0/255., 0/255., 0./255., 1.0f);

    // Tibia Color (colored by leg)
    OsgHandle tibiaHandle[6] = {osgHandle, osgHandle, osgHandle, osgHandle, osgHandle, osgHandle};
    tibiaHandle[0].color = Color(0./255., 0./255., 0./255., 1.0f);
    tibiaHandle[1].color = Color(153./255., 102./255., 0./255., 1.0f);
    tibiaHandle[2].color = Color(153./255., 204./255., 0./255., 1.0f);
    tibiaHandle[3].color = Color(153./255., 0./255., 30./255., 1.0f);
    tibiaHandle[4].color = Color(153./255., 102./255., 30./255., 1.0f);
    tibiaHandle[5].color = Color(153./255., 204./255., 30./255., 1.0f);

    // change Material substance
    OdeHandle odeHandleBody(odeHandle);
    odeHandleBody.substance.toMetal(3.0);

    // get a representation of the origin
     // By multipling this parameter to pose, we can get only the position data of pose
    const Pos nullpos(0, 0, 0);


    /*********************************************************************************:
    *  Create Body Structure
    *     Trunk, Leg etc.
    ***********************************************************************************/

    /*********************************************
     *  Main Frame */
    // Make the boxes connect each other to make rectangle
    /**********************************************/
#ifdef DEBUG_MODE
    cout << "@@@@ Creation of Body frame starts!! @@@@" << endl;
#endif
    // make the rectangle boxes to make hexagon (they are connected each other and make hexagon)
    // make 2 rectangle plate objects
    for(int i=0; i< 2;i++){
      trunk.tPlate[i] = new Box(  conf.body.length, conf.body.width, conf.body.height);
      trunk.tPlate[i]->getOSGPrimitive()->setTexture("Images/wood.rgb");
      // This function (setMass) seems not to be implemented
      //trunk.tPlate[i]->setMass(conf.body.mass / 6.);
    }

    // First object should be initialized (I choose the lower rectangle one)
    trunk.tPlate[0]->init(odeHandle, conf.body.mass / 2., osgHandle);
    trunk.tPlate[0]->setPose( pose );
    // add it to Object
    objects.push_back(trunk.tPlate[0]);

    // just for memorlizing transForm
    trunk.tUpTrans = new ImpTransform2(trunk.tPlate[0], trunk.tPlate[1],Matrix::rotate(0., Vec3(0, 0, 1)) *Matrix::translate(0, 0, conf.dyna.height));
    trunk.tTrans = trunk.tUpTrans;
    trunk.tTrans->init(odeHandle, conf.body.mass / 2., osgHandle);
    // add it to Object
    objects.push_back(trunk.tTrans);


    /*********************************************
       LEGs  **sholder, coxa femur tibia
       //
  **********************************************/
#ifdef DEBUG_MODE
    cout << "@@@@ Creation of Leg frame starts!! @@@@" << endl;
#endif

    // Useful Parameter to make robot model

    // Transition Matrix from origin of this robot to center of the robot
    //  notice: the orgin of the robot is on the center of the lower hexagonal plate
    osg::Matrix trans_rO_to_rC = Matrix::translate( 0., 0., conf.dyna.height / 2.);

    // Trans Matrix from center of the robot to Shoulder Dynamixel center
    // The horizontal length between robot center and Shoulder Dynamixel center
    double length_rC_to_sC_x = (conf.jLength.length_x_center_to_TCJ - conf.dyna.length_axis_to_center);
    // Matrix
    osg::Matrix trans_rC_to_sC_x =  Matrix::translate( length_rC_to_sC_x, 0, 0);

    // The horizontal length between Shoulder Dynamixel center and Shoulder Dynamixel center
    // Matrix
    osg::Matrix trans_sC_to_sC_y =  Matrix::translate(0, conf.jLength.length_y_TCJ_to_TCJ, 0);

    // Trans Matrix from center of the robot to Coxa Dynamixel center
    // The horizontal length between robot center and Coxa Dynamixel center
    double length_rC_to_cC = length_rC_to_sC_x + conf.jLength.length_TCJ_to_CTJ;
    // Matrix
    osg::Matrix trans_rC_to_cC =  Matrix::translate( length_rC_to_cC, 0, 0);

    // Trans Matrix from center of the robot to Femur Dynamixel center
    // The horizontal length between robot center and Femur Dynamixel center
    double length_rC_to_fC_x = conf.jLength.length_x_center_to_TCJ + conf.jLength.length_TCJ_to_CTJ;
    double length_rC_to_fC_z = conf.jLength.length_CTJ_to_FTJ - conf.dyna.length_axis_to_center;
    //double length_rC_to_fC = length_rC_to_cC + conf.jLength.length_CTJ_to_FTJ;
    // Matrix
    osg::Matrix trans_rC_to_fC =  Matrix::translate( length_rC_to_fC_x, 0, length_rC_to_fC_z);

    // Trans Matrix from center of the robot to  Foot Plate center
    // The horizontal length between robot center and Foot Plate center
    double length_rC_to_tC_x = length_rC_to_fC_x + conf.dyna.length_from_axis_to_tip;
    double length_rC_to_tC_z =  conf.jLength.length_CTJ_to_FTJ + (-conf.foot.length + conf.dyna.width)/2.;
    // Matrix
    osg::Matrix trans_rC_to_tC =  Matrix::translate( length_rC_to_tC_x, 0, length_rC_to_tC_z);

    // Trans Matrix from center of the robot to center of the foot sphere
    // Matrix
    osg::Matrix trans_rC_to_fsC =  Matrix::translate( length_rC_to_fC_x, 0, length_rC_to_tC_z - conf.foot.length/2. - conf.foot.footRadius  );

    // Trans Matrix from center of the tibia to center of the foot sphere
    // Matrix
    osg::Matrix trans_tC_to_fsC =  Matrix::translate( - conf.dyna.length_from_axis_to_tip, 0,  - conf.foot.length/2. - conf.foot.footRadius);


    // Trans Matrix from center of the robot to TC joint Center
    // Matrix
    osg::Matrix trans_rC_to_TCj = Matrix::translate( conf.jLength.length_x_center_to_TCJ, 0, 0);

    // Trans Matrix from center of the robot to CT joint Center
    // Matrix
    osg::Matrix trans_rC_to_CTj = Matrix::translate( conf.jLength.length_x_center_to_TCJ + conf.jLength.length_TCJ_to_CTJ, 0, 0);

    // Trans Matrix from center of the robot to FT joint Center
    // Matrix
    osg::Matrix trans_rC_to_FTj = Matrix::translate( conf.jLength.length_x_center_to_TCJ + conf.jLength.length_TCJ_to_CTJ, 0, + conf.jLength.length_CTJ_to_FTJ);


    for(int i = 0; i < LEG_POS_MAX; i++){
      // Name of the Leg
      LegPos leg = LegPos(i);

      // Rotation Matrix : change the rotation direction according to leg number
      // By using this, the direction of leg can be changed.. It is used to adapt to the location of the leg
      Matrix legRotate;
      Matrix legTrans;
      if(leg == L1){
        legRotate = Matrix::rotate(M_PI*1./2., Vec3(0, 0, 1));
        legTrans = Matrix::translate(0, conf.jLength.length_y_TCJ_to_TCJ + conf.y_trans_center, 0);
      }
      else if( leg == L2){
        legRotate = Matrix::rotate(M_PI*1./2., Vec3(0, 0, 1));
        legTrans = Matrix::translate(0, conf.y_trans_center, 0);
      }
      else if(leg == L3 ){
        legRotate = Matrix::rotate(M_PI*1./2., Vec3(0, 0, 1));
        legTrans = Matrix::translate(0, -conf.jLength.length_y_TCJ_to_TCJ +conf.y_trans_center, 0);
      }else if(leg == L4){
        legRotate = Matrix::rotate(-M_PI*1./2., Vec3(0, 0, 1));
        legTrans = Matrix::translate(0, -conf.jLength.length_y_TCJ_to_TCJ - conf.y_trans_center, 0);
      }else if(leg == L5){
        legRotate = Matrix::rotate(-M_PI*1./2., Vec3(0, 0, 1));
        legTrans = Matrix::translate(0, -conf.y_trans_center, 0);
      }else{
        legRotate = Matrix::rotate(-M_PI*1./2., Vec3(0, 0, 1));
        legTrans = Matrix::translate(0, conf.jLength.length_y_TCJ_to_TCJ - conf.y_trans_center, 0);
      }

      // Shoulder ******************************************
      //  We connect the sholder dynamixel to main Body
#ifdef DEBUG_MODE
      cout << "@@@@ Creation of Shoulder starts!! @@@@" << endl;
#endif

      // sholder Dynamixel Box
      Box* dBox =  new Box( conf.dyna.length, conf.dyna.width, conf.dyna.height);
      dBox->getOSGPrimitive()->setTexture("Images/wood.rgb");
      //dBox->setMass(conf.dyna.mass);
       // trans (locate the object on the hexagon plane)
       //  first, I turn the object to leg's direction and translate it to desired position ()
       // mention that this pose matrix is multipled vector from right hand <- it is different from usual one!!
      ImpTransform2* dTrans = new ImpTransform2(trunk.tPlate[0], dBox,
              trans_rC_to_sC_x * trans_rO_to_rC * legTrans * legRotate);
      dTrans->init(odeHandle, conf.dyna.mass, dynaHandle);

      // save to the leg struct
      legs[leg].shoulderBox = dBox;
      legs[leg].shoulder = dTrans;
      // add it to object
      objects.push_back(legs[leg].shoulder);

      // Coxa **********************************************
       // We make the first joint and link of Hexabot
#ifdef DEBUG_MODE
      cout << "@@@@ Creation of Coxa starts!! @@@@" << endl;
#endif

      // Make the Dyanmixel for link
      // Coxa (1st link) Dynamixel Box
      Box* link1dBox = new Box( conf.dyna.length, conf.dyna.width, conf.dyna.height);
      link1dBox->getOSGPrimitive()->setTexture("Images/wood.rgb");
      link1dBox->init(odeHandle, conf.dyna.mass , dynaHandle);
      // about the pose or something
      // set the pose
      //  pose:: pose represent the attitude and position of the object by quotanion
      //         we can use the function rotate and translate as if it is attitude change 4*4 matrix
      //
      // I should say about mechanism of the pose in my understanding.
      //   Pose ; this is the 4 by 4 matrix and it represents the rotation and translation
      //          It can be used as if it is normal 4 by 4 rotation matrix.
      //          But what you should notice is that this matrix is for Row Vector (1 by 4 vector)
      //          So, the row of the calculating is inverse to normal one!!
      //      You should be careful about it

      Matrix l1Pose = Matrix::rotate( M_PI / 2., Vec3(1, 0, 0)) * /* this rotation is used to turn the dymamixel to optimal attitude!! (because, we change the axis of joints) */
                trans_rC_to_cC * trans_rO_to_rC * legTrans* legRotate * pose;  //  first, I turn the object to leg's direction and translate it to desired position ()
      link1dBox->setPose(l1Pose);
      // save it to leg struct
      legs[leg].coxa = link1dBox;
      // add it to object
      objects.push_back(legs[leg].coxa);

      // TC joints *******************************:::::::
       // create the joint of dynamixel on Body (from 1st link to body)
       // calculate the joint pose (attitude and position) by using trans and rotate function
      osg::Matrix j1Pose = trans_rC_to_TCj * trans_rO_to_rC * legTrans* legRotate * pose;
      // To make a joint, we need the position vector only, so multiple nullpos to the pose to get the vector.
      //  the attitude is determined by axis (it is z axis)
      HingeJoint* TCj = new HingeJoint( trunk.tPlate[0], link1dBox,  nullpos * j1Pose, Axis(0,0,1) * legRotate );
      // ^ Notice: When we want to rotate the Axis() like Pose, we have to multiple the rotation matrix from RIGHT SIDE
      //     Not Left side, it is very different from real calculation
      TCj->init(odeHandle, osgHandle, true, conf.rate * 0.04);
      // add the joint to the joint vector
      joints.push_back(TCj);

      // create motor (by using servo motor, we will do position control)
      //OneAxisServo* servo1 = new OneAxisServoVel(odeHandle, TCj, conf.servoParam.TC_angle_MIN, conf.servoParam.TC_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
      //Debug
      OneAxisServo* servo1;
#ifdef HEXABOT_MAIN_VER_1_6
      servo1 = new OneAxisServoVel(odeHandle, TCj, conf.servoParam.TC_angle_MIN, conf.servoParam.TC_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
#else
      servo1 = new OneAxisServoCentered(TCj, conf.servoParam.TC_angle_MIN, conf.servoParam.TC_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.integ, conf.servoParam.maxVel, 1.0);
#endif

      // save it to leg struct
      legs[leg].tcJoint = TCj;
      legs[leg].tcServo = servo1;
      // add it to servo Map
      servos[getMotorName(leg, TC)] = servo1;

      // create torque sensor
      // In this time, I do not use scaller just get real value
      motorTorqSensors[getMotorName(leg, TC)] = new TorqueSensor(1., 1.);
      // initialize (own is Null, it was not used in simulation)
      motorTorqSensors[getMotorName(leg, TC)]->init(NULL, TCj);


      // Femur **********************************************
      // We make the second joint and link of Hexabot (CT joint and femur)
#ifdef DEBUG_MODE
      cout << "@@@@ Creation of Femur starts!! @@@@" << endl;
#endif

      // Make the Dyanmixel for link
      // Femur (2nd link) Dynamixel Box
      Box* link2dBox = new Box( conf.dyna.length, conf.dyna.width, conf.dyna.height);
      link2dBox->getOSGPrimitive()->setTexture("Images/wood.rgb");
      link2dBox->init(odeHandle, conf.dyna.mass , dynaHandle);
      // set the pose
      //  pose:: pose represent the attitude and position of the object by quotanion
      //         we can use the function rotate and translate as if it is attitude change 4*4 matrix
      osg::Matrix l2Pose = Matrix::rotate( M_PI / 2., Vec3(0, 0, 1)) * Matrix::rotate( M_PI / 2., Vec3(1, 0, 0)) * // this rotation is used to turn the dymamixel to optimal attitude!! (because, we change the axis of joints)
                  trans_rC_to_fC * trans_rO_to_rC * legTrans * legRotate * pose;  //  first, I turn the object to leg's direction and translate it to desired position ()
      link2dBox->setPose(l2Pose);
      // save it to leg struct
      legs[leg].femur = link2dBox;
      // add it to object
      objects.push_back(legs[leg].femur);

      // CT joint :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
      // create the joint of dynamixel on Body (from 2nd link to 1st Link)
      // calculate the joint pose (attitude and position) by using trans and rotate function
      osg::Matrix j2Pose = trans_rC_to_CTj * trans_rO_to_rC * legTrans* legRotate * pose;
      // To make a joint, we need the position vector only, so multiple nullpos to the pose to get the vector.
      //  the attitude is determined by axis (it is z axis)
      HingeJoint* CTj = new HingeJoint( legs[leg].coxa, link2dBox, nullpos * j2Pose,  Axis(0,1,0) * legRotate);
      // ^ Notice: When we want to rotate the Axis() like Pose, we have to multiple the rotation matrix from RIGHT SIDE
      //     Not Left side, it is very different from real calculation
      CTj->init(odeHandle, osgHandle, true, conf.rate * 0.04);
      // add the joint to the joint vector
      joints.push_back(CTj);

      // create motor (by using servo motor, we will do position control)
      // OneAxisServo* servo2 = new OneAxisServoVel(odeHandle, CTj, conf.servoParam.CT_angle_MIN, conf.servoParam.CT_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
      //Debug
      OneAxisServo* servo2;
#ifdef HEXABOT_MAIN_VER_1_6
      servo2 = new OneAxisServoVel(odeHandle, CTj, conf.servoParam.CT_angle_MIN, conf.servoParam.CT_angle_MAX, conf.servoParam.power, conf.servoParam.damp,conf.servoParam.maxVel, 1.0);
#else
      servo2 = new OneAxisServoCentered(CTj, conf.servoParam.CT_angle_MIN, conf.servoParam.CT_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.integ, conf.servoParam.maxVel, 1.0);
#endif

      // save it to leg struct
      legs[leg].ctJoint = CTj;
      legs[leg].ctServo = servo2;
      // add it to servo Map
      servos[Hexabot::getMotorName(leg, CT)] = servo2;

      // create torque sensor
      // In this time, I do not use scaller just get real value
      motorTorqSensors[getMotorName(leg, CT)] = new TorqueSensor(1., 1.);
      // initialize (own is Null, it was not used in simulation)
      motorTorqSensors[getMotorName(leg, CT)]->init(NULL, CTj);

      // Tibia **********************************************
      // We make the third joint and link of Hexabot (FT joint and femur)
#ifdef DEBUG_MODE
      cout << "@@@@ Creation of Tibia starts!! @@@@" << endl;
#endif

      // Make the Plate
      // Tibia (3rd link) Plate
      Box* link3dBox = new Box( conf.foot.height, conf.foot.width, conf.foot.length);
      link3dBox->getOSGPrimitive()->setTexture("Images/wood.rgb");
      link3dBox->init(odeHandle, conf.foot.mass , tibiaHandle[leg]);
      // set the pose
      //  pose:: pose represent the attitude and position of the object by quotanion
      //         we can use the function rotate and translate as if it is attitude change 4*4 matrix
      osg::Matrix l3Pose = trans_rC_to_tC * trans_rO_to_rC * legTrans* legRotate * pose;  //  first, I turn the object to leg's direction and translate it to desired position ()
      link3dBox->setPose(l3Pose);
      // save it to leg struct
      legs[leg].tibia = link3dBox;
      // add it to object
      objects.push_back(legs[leg].tibia);


      //Make Foot
      // Sphere
      Sphere* foot = new Sphere( conf.foot.footRadius );
      foot->getOSGPrimitive()->setTexture("Images/wood.rgb");
      // Set the substance of foot
      OdeHandle rubberHandle(odeHandle);
      const Substance FootSubstance(3.0, 0.0, 500.0, 0.1);//500
      rubberHandle.substance = FootSubstance;

      legs[leg].footSphere = foot;
      //translate
      legs[leg].foot = new ImpTransform2(legs[leg].tibia, foot, trans_tC_to_fsC);
      // initialize
      legs[leg].foot->init(rubberHandle, 0.0, osgHandle);
      //add it to objects
      objects.push_back(legs[leg].foot);

      // FT joint :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
      // create the joint of dynamixel on Femur (from 3rd link to 2nd Link)
      // calculate the joint pose (attitude and position) by using trans and rotate function
      osg::Matrix j3Pose = trans_rC_to_FTj * trans_rO_to_rC * legTrans* legRotate * pose;
      // To make a joint, we need the position vector only, so multiple nullpos to the pose to get the vector.
      //  the attitude is determined by axis (it is z axis)
      HingeJoint* FTj = new HingeJoint( legs[leg].femur, link3dBox, nullpos * j3Pose, Axis(0,1,0) * legRotate);
      // ^ Notice: When we want to rotate the Axis() like Pose, we have to multiple the rotation matrix from RIGHT SIDE
      //     Not Left side, it is very different from real calculation
      FTj->init(odeHandle, osgHandle, true, conf.rate * 0.04);
      // add the joint to the joint vector
      joints.push_back(FTj);

      // create motor (by using servo motor, we will do position control)
      // OneAxisServo* servo3 = new OneAxisServoVel(odeHandle, FTj, conf.servoParam.FT_angle_MIN, conf.servoParam.FT_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
      //Debug
      OneAxisServo* servo3;
#ifdef HEXABOT_MAIN_VER_1_6
      servo3 = new OneAxisServoVel(odeHandle, FTj, conf.servoParam.FT_angle_MIN, conf.servoParam.FT_angle_MAX, conf.servoParam.power, conf.servoParam.damp,conf.servoParam.maxVel, 1.0);
#else
      servo3 = new OneAxisServoCentered(FTj, conf.servoParam.FT_angle_MIN, conf.servoParam.FT_angle_MAX, conf.servoParam.power, conf.servoParam.damp,conf.servoParam.integ, conf.servoParam.maxVel, 1.0);
#endif

      // save it to leg struct
      legs[leg].ftJoint = FTj;
      legs[leg].ftServo = servo3;
      // add it to servo Map
      servos[Hexabot::getMotorName(leg, FT)] = servo3;

      // create torque sensor
      // In this time, I do not use scaller just get real value
      motorTorqSensors[getMotorName(leg, FT)] = new TorqueSensor(1., 1.);
      // initialize (own is Null, it was not used in simulation)
      motorTorqSensors[getMotorName(leg, FT)]->init(NULL, FTj);

      // Leg contact Sensors (Foot toe)
      legContactSensors[leg] = new ContactSensor(false, 100, conf.foot.footRadius*1.1, false, true, Color(0,5,0));
      //make the sphere a little bit larger than real foot to detect touching 1.01

      //legContactSensors[leg]->update();
      // We set the contact sensor at the point of foot sphere
      //legContactSensors[leg]->init(odeHandle, osgHandle, legs[leg].tibia, true, trans_tC_to_fsC);

      // this is changed to adopt the new version of lpzrobots
      legContactSensors[leg]->setInitData(odeHandle, osgHandle, TRANSM(0, 0, 0));//trans_tC_to_fsC);
      legContactSensors[leg]->init(legs[leg].foot);

      //legContactSensors[leg]->update();
      //odeHandle.addIgnoredPair(legs[leg].foot, legContactSensors[leg]->getTransformObject());
      //legContactSensors[LegPos(i)]->setInitData(odeHandle, osgHandle, TRANSM(0, 0, -(0.5) * l4));
      //legContactSensors[LegPos(i)]->init(foot);
    }

#ifdef DEBUG_MODE
    cout << "@@@@ Creation Phase ends!! @@@@" << endl;
#endif

    created=true;
  }

  // getTorqueSensorData
  sensor Hexabot::getTorqueData(TorqueSensor*  torqueSensor){
    std::list<sensor> a;
    std::list<sensor>::iterator it;
    if(0){//(torqueSensor){
      a = torqueSensor->getList();
      it = a.end();
      --it;
      //std::cout << "trq " << *it << std::endl;
      return *it;
    }else  return -1.;
  }

  // convert Pose Matrix(Quatanion) to the roll, pitch, yaw angle (rad)
  osg::Vec3d Hexabot::convert_Quat_to_RollPitchYaw(osg::Quat quat){
    osg::Vec3d rpy;
    // the 3 vetors of Matrix
    osg::Vec3d M0, M1, M2;

    // use the quaternion to rotate this vector and get Matrix elements
    M0 = quat * osg::Vec3(1., 0, 0);
    M1 = quat * osg::Vec3(0, 1., 0);
    M2 = quat * osg::Vec3(0, 0, 1.);

    // calc roll pitch yaw
    rpy[2] = atan2( M0[1], M0[0]); // yaw
    rpy[1] = atan2( - M0[2], sqrt(M1[2]*M1[2] +  M2[2]*M2[2]) );  // pitch
    rpy[0] = atan2( M1[2], M2[2]); // roll
    return rpy;
  }

  // calculate COG Position
  osg::Vec3d Hexabot::calc_COGPosition(void){
    // cogPos
    osg::Vec3d cogPos;

    // add many objects but I do it manually because many types of objects we have
    // Trunk lower hexagonal plate
    cogPos = this->getMainPrimitive()->getPosition() * (conf.body.mass / 2.);
    // Trunk higher hexagonal plate
    cogPos += trunk.tUpTrans->getChildPose().getTrans() * (conf.body.mass / 2.);

    // Leg clculation
    for(int i = 0; i < LEG_POS_MAX; i++){
      // Name of the Leg
      LegPos leg = LegPos(i);
      // shoulder
      cogPos += legs[leg].shoulder->getChildPose().getTrans() * conf.dyna.mass;
      // coxa
      cogPos += legs[leg].coxa->getPosition() * conf.dyna.mass;
      // femur
      cogPos += legs[leg].femur->getPosition() * conf.dyna.mass;
      // tibia plate
      cogPos += legs[leg].tibia->getPosition() * conf.foot.mass;
      // foot sphere has no weight!!
    }

    cogPos = cogPos / conf.wholeMass;
    return cogPos;
    // calc COG
  }

  /** destroys HEXABOT and space
   */
  void Hexabot::destroy(){
    if (created){
      //irSensorBank.clear();
      cleanup();
      odeHandle.deleteSpace();

      // delete contact sensors
      for (int i = 0; i < LEG_POS_MAX; i++) {
        if (legContactSensors[LegPos(i)])
          delete legContactSensors[LegPos(i)];
      }
      legContactSensors.clear();
    }
    created=false;
  }

  // constructor of struct Leg
  Hexabot::Leg::Leg():
    tcJoint(0), ctJoint(0), ftJoint(0), footJoint(0),
    tcServo(0), ctServo(0), ftServo(0),
    footSpring(0), shoulder(0), shoulderBox(0),
    coxa(0),femur(0),tibia(0),foot(0){
  }

  // constructor of struct trunk
  Hexabot::Trunk::Trunk() {
    tPlate[0]=0;
    tPlate[1]=0;
    tTrans=0;
  }

}
