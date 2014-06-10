/***************************************************************************
 *   Copyright (C) 2012 by                                                 *
 *    Martin Biehl <mab@physik3.gwdg.de>                                   *
 *    Guillaume de Chambrier <s0672742@sms.ed.ac.uk>                       *
 *    martius@informatik.uni-leipzig.de                                    *
 *    Timo Nachstedt <nachstedt@physik3.gwdg.de>                           *
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
 **************************************************************************/

//#define VERBOSE
#include <cmath>
#include <assert.h>

// #include <ode/ode.h>
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>

// include sensors
#include <ode_robots/raysensorbank.h>
#include <ode_robots/irsensor.h>
#include <ode_robots/speedsensor.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/twoaxisservo.h>
#include <ode_robots/spring.h>

#include <ode_robots/mathutils.h>

// include header file
#include "amos4legs.h"

// rotation and translation matrixes (to make the code shorter)
#define ROTM osg::Matrix::rotate
#define TRANSM osg::Matrix::translate

namespace lpzrobots {

  AmosFour::Leg::Leg() {
    tcJoint = 0;
    ctrJoint = 0;
    ftiJoint = 0;
    footJoint = 0;
    tcServo = 0;
    ctrServo = 0;
    ftiServo = 0;
    footSpring = 0;
    shoulder = 0;
    coxa = 0;
    second = 0;
    tibia = 0;
    foot = 0;
  }

  // constructor:
  // - give handle for ODE and OSG stuff
  // also initialize AmosII.conf with the configuration in the argument of
  // the constructor
  AmosFour::AmosFour(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const AmosFourConf& c, const std::string& name) :
          OdeRobot(odeHandle, osgHandle, name, "AMOSII 0.1"), conf(c) {
//    legPosUsage[L0] = LEG;
    legPosUsage[L1] = LEG;
    legPosUsage[L2] = LEG;
//    legPosUsage[R0] = LEG;
    legPosUsage[R1] = LEG;
    legPosUsage[R2] = LEG;

    // robot is not created till now
    created = false;
    backboneServo = 0;
    usSensorFrontLeft = 0;
    usSensorFrontRight = 0;
    speedsensor = 0;

    addParameter("coxaPower", &conf.coxaPower);
    addParameter("secondPower", &conf.secondPower);
    addParameter("coxaDamp", &conf.coxaDamping);
    addParameter("fcoxaJointLimitF", &conf.fcoxaJointLimitF);
    addParameter("fcoxaJointLimitB", &conf.fcoxaJointLimitB);
    addParameter("mcoxaJointLimitF", &conf.mcoxaJointLimitF);
    addParameter("mcoxaJointLimitB", &conf.mcoxaJointLimitB);
    addParameter("rcoxaJointLimitF", &conf.rcoxaJointLimitF);
    addParameter("rcoxaJointLimitB", &conf.rcoxaJointLimitB);
    addParameter("secondJointLimitD", &conf.secondJointLimitD);
    addParameter("secondJointLimitU", &conf.secondJointLimitU);
    addParameter("coxaMaxVel", &conf.coxaMaxVel);

    if (conf.useTebiaJoints) {
      addParameter("tebiaPower", &conf.tebiaPower);
      addParameter("tebiaDamp", &conf.tebiaDamping);
      addParameter("tebiaJointLimitD", &conf.tebiaJointLimitD);
      addParameter("tebiaJointLimitU", &conf.tebiaJointLimitU);
    }

    // name the sensors
    nameSensor(TR0_as, "TR0 angle sensor");
    nameSensor(TR0_as, "TR0 angle sensor");
    nameSensor(TR1_as, "TR1 angle sensor");
    nameSensor(TR2_as, "TR2 angle sensor");
    nameSensor(TL0_as, "TL0 angle sensor");
    nameSensor(TL1_as, "TL1 angle sensor");
    nameSensor(TL2_as, "TL2 angle sensor");
    nameSensor(CR0_as, "CR0 angle sensor");
    nameSensor(CR1_as, "CR1 angle sensor");
    nameSensor(CR2_as, "CR2 angle sensor");
    nameSensor(CL0_as, "CL0 angle sensor");
    nameSensor(CL1_as, "CL1 angle sensor");
    nameSensor(CL2_as, "CL2 angle sensor");
    nameSensor(FR0_as, "FR0 angle sensor");
    nameSensor(FR1_as, "FR1 angle sensor");
    nameSensor(FR2_as, "FR2 angle sensor");
    nameSensor(FL0_as, "FL0 angle sensor");
    nameSensor(FL1_as, "FL1 angle sensor");
    nameSensor(FL2_as, "FL2 angle sensor");
    nameSensor(BJ_as, "BJ angle sensor ");
    nameSensor(R0_fs, "R0 foot contact sensor");
    nameSensor(R1_fs, "R1 foot contact sensor");
    nameSensor(R2_fs, "R2 foot contact sensor");
    nameSensor(L0_fs, "L0 foot contact sensor");
    nameSensor(L1_fs, "L1 foot contact sensor");
    nameSensor(L2_fs, "L2 foot contact sensor");
    nameSensor(FR_us, "Front Ultrasonic sensors right");
    nameSensor(FL_us, "Front Ultrasonic sensors left");
    nameSensor(R0_irs, "R0 IR sensor");
    nameSensor(R1_irs, "R1 IR sensor");
    nameSensor(R2_irs, "R2 IR sensor");
    nameSensor(L0_irs, "L0 IR sensor");
    nameSensor(L1_irs, "L1 IR sensor");
    nameSensor(L2_irs, "L2 IR sensor");
    nameSensor(R0_us, "R0 ultrasonic sensor");
    nameSensor(R1_us, "R1 ultrasonic sensor");
    nameSensor(L0_us, "L0 ultrasonic sensor");
    nameSensor(L1_us, "L1 ultrasonic sensor");
    nameSensor(TR0_ts, "TR0 torque sensor");
    nameSensor(TR1_ts, "TR1 torque sensor");
    nameSensor(TR2_ts, "TR2 torque sensor");
    nameSensor(TL0_ts, "TL0 torque sensor");
    nameSensor(TL1_ts, "TL1 torque sensor");
    nameSensor(TL2_ts, "TL2 torque sensor");
    nameSensor(CR0_ts, "CR0 torque sensor");
    nameSensor(CR1_ts, "CR1 torque sensor");
    nameSensor(CR2_ts, "CR2 torque sensor");
    nameSensor(CL0_ts, "CL0 torque sensor");
    nameSensor(CL1_ts, "CL1 torque sensor");
    nameSensor(CL2_ts, "CL2 torque sensor");
    nameSensor(FR0_ts, "FR0 torque sensor");
    nameSensor(FR1_ts, "FR1 torque sensor");
    nameSensor(FR2_ts, "FR2 torque sensor");
    nameSensor(FL0_ts, "FL0 torque sensor");
    nameSensor(FL1_ts, "FL1 torque sensor");
    nameSensor(FL2_ts, "FL2 torque sensor");
    nameSensor(BJ_ts, "BJ torque sensor");
    nameSensor(BX_acs, "body accelerometer x");
    nameSensor(BY_acs, "body accelerometer y");
    nameSensor(BZ_acs, "body accelerometer z");
    nameSensor(L_ps, "photo sensor left");
    nameSensor(M_ps, "photo sensor middle");
    nameSensor(R_ps, "photo sensor right");
    nameSensor(G0x_s, "goal0 orientation x");
    nameSensor(G0y_s, "goal0 orientation y");
    nameSensor(G0z_s, "goal0 orientation z");
    nameSensor(A_cs, "average motor current sensor");
    nameSensor(BX_spd, "body speed sensor x");
    nameSensor(BY_spd, "body speed sensor y");
    nameSensor(BZ_spd, "body speed sensor z");
    nameSensor(G0angleroll_s, "goal0 angle roll (x)");
    nameSensor(G0anglepitch_s, "goal0 angle pitch (y)");
    nameSensor(G0angleyaw_s, "goal0 angle roll (z)");
    nameSensor(G1x_s, "goal1 orientation x");
    nameSensor(G1y_s, "goal1 orientation y");
    nameSensor(G1z_s, "goal1 orientation z");
    nameSensor(G1angleroll_s, "goal1 angle roll (x)");
    nameSensor(G1anglepitch_s, "goal1 angle pitch (y)");
    nameSensor(G1angleyaw_s, "goal1 angle roll (z)");
    nameSensor(G2x_s, "goal2 orientation x");
    nameSensor(G2y_s, "goal2 orientation y");
    nameSensor(G2z_s, "goal2 orientation z");
    nameSensor(G2angleroll_s, "goal2 angle roll (x)");
    nameSensor(G2anglepitch_s, "goal2 angle pitch (y)");
    nameSensor(G2angleyaw_s, "goal2 angle roll (z)");
    nameSensor(LaserNmbEdge_s, "laser scanner number of edges");
    nameSensor(LaserHeight_s, "laser scanner average height");
    nameSensor(LaserRough_s, "laser scanner roughness");
    nameSensor(LaserMaxHeight_s, "laser scanner maximum height in data");
    nameSensor(LaserMinHeight_s, "laser scanner minimum height in data");
    nameSensor(Poti_s, "potentiometer sensor");
    nameSensor(Compassx_s, "compass sensor x");
    nameSensor(Compassy_s, "compass sensor y");
    nameSensor(R0X_acs, "R0 accelerometer x");
    nameSensor(R0Y_acs, "R0 accelerometer y");
    nameSensor(R0Z_acs, "R0 accelerometer z");
    nameSensor(R1X_acs, "R1 accelerometer x");
    nameSensor(R1Y_acs, "R1 accelerometer y");
    nameSensor(R1Z_acs, "R1 accelerometer z");
    nameSensor(R2X_acs, "R2 accelerometer x");
    nameSensor(R2Y_acs, "R2 accelerometer y");
    nameSensor(R2Z_acs, "R2 accelerometer z");
    nameSensor(L0X_acs, "L0 accelerometer x");
    nameSensor(L0Y_acs, "L0 accelerometer y");
    nameSensor(L0Z_acs, "L0 accelerometer z");
    nameSensor(L1X_acs, "L1 accelerometer x");
    nameSensor(L1Y_acs, "L1 accelerometer y");
    nameSensor(L1Z_acs, "L1 accelerometer z");
    nameSensor(L2X_acs, "L2 accelerometer x");
    nameSensor(L2Y_acs, "L2 accelerometer y");
    nameSensor(L2Z_acs, "L2 accelerometer z");
    nameSensor(Microphone0_s, "microphone 0");
    nameSensor(Microphone1_s, "microphone 1");
    nameSensor(Microphone2_s, "microphone 2");
    nameSensor(In_x, "inclinometer x");
    nameSensor(In_y, "inclinometer y");
    nameSensor(BX_pos, "body position sensor x");
    nameSensor(BY_pos, "body position sensor y");
    nameSensor(BZ_pos, "body position sensor z");
    nameSensor(BX_ori, "body orientation sensor x");
    nameSensor(BY_ori, "body orientation sensor y");
    nameSensor(BZ_ori, "body orientation sensor z");

    // name the motors
    nameMotor(TR0_m, "TR0 motor");
    nameMotor(TR1_m, "TR1 motor");
    nameMotor(TR2_m, "TR2 motor");
    nameMotor(TL0_m, "TL0 motor");
    nameMotor(TL1_m, "TL1 motor");
    nameMotor(TL2_m, "TL2 motor");
    nameMotor(CR0_m, "CR0 motor");
    nameMotor(CR1_m, "CR1 motor");
    nameMotor(CR2_m, "CR2 motor");
    nameMotor(CL0_m, "CL0 motor");
    nameMotor(CL1_m, "CL1 motor");
    nameMotor(CL2_m, "CL2 motor");
    nameMotor(FR0_m, "FR0 motor");
    nameMotor(FR1_m, "FR1 motor");
    nameMotor(FR2_m, "FR2 motor");
    nameMotor(FL0_m, "FL0 motor");
    nameMotor(FL1_m, "FL1 motor");
    nameMotor(FL2_m, "FL2 motor");
    nameMotor(BJ_m, "BJ motor");

    // add further inspectables
//    addInspectableValue("posX", &position.x, "x Position of robot");
//    addInspectableValue("posY", &position.y, "y Position of robot");
//    addInspectableValue("posZ", &position.z, "z Position of robot");

  }

  AmosFour::~AmosFour() {
    destroy();
  }

  int AmosFour::getMotorNumberIntern() {
    return AMOSII_MOTOR_MAX;
  }
  ;

  /**
   * Assign a human readable name to a sensor. This name is used for the
   * associated inspectable value as used e.g. in guilogger.
   *
   * @param motorNo index of the motor (for standard motors defined by
   *        the SensorName enum)
   * @param name human readable name for the sensor
   */
  void AmosFour::nameSensor(const int sensorNo, const char* name) {
#ifdef VERBOSE
    std::cerr << "AmosII::nameSensor BEGIN\n";
#endif
    addInspectableDescription("x[" + std::itos(sensorNo) + "]", name);
#ifdef VERBOSE
    std::cerr << "AmosII::nameSensor END\n";
#endif
  }

  /**
   * Assign a human readable name to a motor. This name is used for the
   * associated inspectable value as used e.g. in guilogger.
   *
   * @param motorNo index of the motor (for standard motors defined by
   *        the MotorName enum)
   * @param name human readable name for the motor
   */
  void AmosFour::nameMotor(const int motorNo, const char* name) {
#ifdef VERBOSE
    std::cerr << "AmosII::nameMotor BEGIN\n";
#endif
    addInspectableDescription("y[" + std::itos(motorNo) + "]", name);
#ifdef VERBOSE
    std::cerr << "AmosII::nameMotor END\n";
#endif
  }

  /* sets actual motorcommands
   @param motors motors scaled to [-1,1]
   @param motornumber length of the motor array
   */
  void AmosFour::setMotorsIntern(const double* motors, int motornumber) {
#ifdef VERBOSE
    std::cerr << "AmosII::setMotors BEGIN\n";
#endif
    assert(created);
    // robot must exist
    assert(motornumber==getMotorNumber());
    for (MotorMap::iterator it = servos.begin(); it != servos.end(); it++) {
      MotorName const name = it->first;
      OneAxisServo * const servo = it->second;
      //We multiple with -1 to map to real hexapod
      if (servo)
        servo->set(-motors[name]);
    }
#ifdef VERBOSE
    std::cerr << "AmosII::setMotors END\n";
#endif
  }
  ;

  int AmosFour::getSensorNumberIntern() {
#ifdef VERBOSE
    std::cerr << "AmosII::getSensorNumberIntern BEGIN\n";
#endif
#ifdef VERBOSE
    std::cerr << "AmosII::getSensorNumberIntern END\n";
#endif
    return AMOSII_SENSOR_MAX;
  }
  ;

  /* returns actual sensorvalues
   @param sensors sensors scaled to [-1,1] (more or less)
   @param sensornumber length of the sensor array
   @return number of actually written sensors
   */
  int AmosFour::getSensorsIntern(sensor* sensors, int sensornumber) {
#ifdef VERBOSE
    std::cerr << "AmosII::getSensors BEGIN\n";
#endif
    assert(created);
    assert(sensornumber == getSensorNumberIntern());

    // angle sensors
    //We multiple with -1 to map to real hexapod
    sensors[TR0_as] = servos[TR0_m] ? -servos[TR0_m]->get() : 0;
    sensors[TR1_as] = servos[TR1_m] ? -servos[TR1_m]->get() : 0;
    sensors[TR2_as] = servos[TR2_m] ? -servos[TR2_m]->get() : 0;
    sensors[TL0_as] = servos[TL0_m] ? -servos[TL0_m]->get() : 0;
    sensors[TL1_as] = servos[TL1_m] ? -servos[TL1_m]->get() : 0;
    sensors[TL2_as] = servos[TL2_m] ? -servos[TL2_m]->get() : 0;
    sensors[CR0_as] = servos[CR0_m] ? -servos[CR0_m]->get() : 0;
    sensors[CR1_as] = servos[CR1_m] ? -servos[CR1_m]->get() : 0;
    sensors[CR2_as] = servos[CR2_m] ? -servos[CR2_m]->get() : 0;
    sensors[CL0_as] = servos[CL0_m] ? -servos[CL0_m]->get() : 0;
    sensors[CL1_as] = servos[CL1_m] ? -servos[CL1_m]->get() : 0;
    sensors[CL2_as] = servos[CL2_m] ? -servos[CL2_m]->get() : 0;
    sensors[FR0_as] = servos[FR0_m] ? -servos[FR0_m]->get() : 0;
    sensors[FR1_as] = servos[FR1_m] ? -servos[FR1_m]->get() : 0;
    sensors[FR2_as] = servos[FR2_m] ? -servos[FR2_m]->get() : 0;
    sensors[FL0_as] = servos[FL0_m] ? -servos[FL0_m]->get() : 0;
    sensors[FL1_as] = servos[FL1_m] ? -servos[FL1_m]->get() : 0;
    sensors[FL2_as] = servos[FL2_m] ? -servos[FL2_m]->get() : 0;
    sensors[BJ_as] = servos[BJ_m] ? -servos[BJ_m]->get() : 0;

    // foot contact sensors
    if (conf.legContactSensorIsBinary) { // No scaling since binary signals are already in the range of [0,..,1]
//      sensors[R0_fs] = legContactSensors[R0] ? legContactSensors[R0]->get() : 0;
      sensors[R1_fs] = legContactSensors[R1] ? legContactSensors[R1]->get() : 0;
      sensors[R2_fs] = legContactSensors[R2] ? legContactSensors[R2]->get() : 0;
//      sensors[L0_fs] = legContactSensors[L0] ? legContactSensors[L0]->get() : 0;
      sensors[L1_fs] = legContactSensors[L1] ? legContactSensors[L1]->get() : 0;
      sensors[L2_fs] = legContactSensors[L2] ? legContactSensors[L2]->get() : 0;
    } else { // Scaling since analog signals are used then we scale them to the range of [0,..,1]
      std::vector<double> max, min;
      if (conf.amos_version == 2) {
        max.push_back(0.16);
        max.push_back(0.20);
        max.push_back(0.14);
        max.push_back(0.24);
        max.push_back(0.20);
        max.push_back(0.14);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
      } else {
        //TODO: need to be recalibrated for amos version 1
        max.push_back(0.30);
        max.push_back(0.22);
        max.push_back(0.22);
        max.push_back(0.22);
        max.push_back(0.22);
        max.push_back(0.22);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
      }
//      sensors[R0_fs] =
//          legContactSensors[R0] ? ((legContactSensors[R0]->get() - min.at(0)) / (max.at(0) - min.at(0))) : 0;
      sensors[R1_fs] =
          legContactSensors[R1] ? ((legContactSensors[R1]->get() - min.at(1)) / (max.at(1) - min.at(1))) : 0;
      sensors[R2_fs] =
          legContactSensors[R2] ? ((legContactSensors[R2]->get() - min.at(2)) / (max.at(2) - min.at(2))) : 0;
//      sensors[L0_fs] =
//          legContactSensors[L0] ? ((legContactSensors[L0]->get() - min.at(3)) / (max.at(3) - min.at(3))) : 0;
      sensors[L1_fs] =
          legContactSensors[L1] ? ((legContactSensors[L1]->get() - min.at(4)) / (max.at(4) - min.at(4))) : 0;
      sensors[L2_fs] =
          legContactSensors[L2] ? ((legContactSensors[L2]->get() - min.at(5)) / (max.at(5) - min.at(5))) : 0;

      for (int i = R0_fs; i <= L2_fs; i++) {
        if (sensors[i] > 1.0)
          sensors[i] = 1.0;
      }

    }
    // Front Ultrasonic sensors (right and left)
    sensors[FR_us] = usSensorFrontRight->getValue();
    sensors[FL_us] = usSensorFrontLeft->getValue();

    // IR sensors at the legs
//    sensors[R0_irs] = irLegSensors[R0] ? irLegSensors[R0]->getValue() : 0;
    sensors[R1_irs] = irLegSensors[R1] ? irLegSensors[R1]->getValue() : 0;
    sensors[R2_irs] = irLegSensors[R2] ? irLegSensors[R2]->getValue() : 0;
//    sensors[L0_irs] = irLegSensors[L0] ? irLegSensors[L0]->getValue() : 0;
    sensors[L1_irs] = irLegSensors[L1] ? irLegSensors[L1]->getValue() : 0;
    sensors[L2_irs] = irLegSensors[L2] ? irLegSensors[L2]->getValue() : 0;

    // Reflex ultrasonic sensors at front, middle and rear legs
    sensors[R0_us] = 0;
    sensors[R1_us] = 0;
    sensors[L0_us] = 0;
    sensors[L1_us] = 0;

    // Torque sensors, used as Current sensors at each motor
    sensors[TR0_ts] = 0;
    sensors[TR1_ts] = 0;
    sensors[TR2_ts] = 0;
    sensors[TL0_ts] = 0;
    sensors[TL1_ts] = 0;
    sensors[TL2_ts] = 0;
    sensors[CR0_ts] = 0;
    sensors[CR1_ts] = 0;
    sensors[CR2_ts] = 0;
    sensors[CL0_ts] = 0;
    sensors[CL1_ts] = 0;
    sensors[CL2_ts] = 0;
    sensors[FR0_ts] = 0;
    sensors[FR1_ts] = 0;
    sensors[FR2_ts] = 0;
    sensors[FL0_ts] = 0;
    sensors[FL1_ts] = 0;
    sensors[FL2_ts] = 0;
    sensors[BJ_ts] = 0;

    // 3D Accelerometer (x,y,z) at body
    sensors[BX_acs] = 0;
    sensors[BY_acs] = 0;

    // photo (light) sensors Left, Middle and Right
    sensors[L_ps] = 0;
    sensors[M_ps] = 0;
    sensors[R_ps] = 0;

    // goal orientation sensors (relative position to reference object)
    sensors[G0x_s] = 0;
    sensors[G0y_s] = 0;
    sensors[G0z_s] = 0;

    // average current sensor
    sensors[A_cs] = 0;

    // Body speed sensors
    sensor speedsens[3] = { 0, 0, 0 };
    if (speedsensor)
      speedsensor->get(speedsens, 3);
    sensors[BX_spd] = speedsens[0];
    sensors[BY_spd] = speedsens[1];
    sensors[BZ_spd] = speedsens[2];

     // Body position sensors
     sensors[BX_pos] = position.x;
     sensors[BY_pos] = position.y;
     sensors[BZ_pos] = position.z;

    //------------------------Add GoalSensor by Ren-------------------
    if (GoalSensor_active) {
      //the first goal
      std::vector<RelativePositionSensor>::iterator it = GoalSensor.begin(); //we only use one goal sensor
      std::list<sensor> gls_val = it->getList();
      sensors[G0z_s] = gls_val.back();
      gls_val.pop_back();
      sensors[G0y_s] = gls_val.back();
      gls_val.pop_back();
      sensors[G0x_s] = gls_val.back();
      gls_val.pop_back();

      //the second goal
      it++;
      gls_val = it->getList();
      sensors[G1z_s] = gls_val.back();
      gls_val.pop_back();
      sensors[G1y_s] = gls_val.back();
      gls_val.pop_back();
      sensors[G1x_s] = gls_val.back();
      gls_val.pop_back();

      //the third goal
      it++;
      gls_val = it->getList();
      sensors[G2z_s] = gls_val.back();
      gls_val.pop_back();
      sensors[G2y_s] = gls_val.back();
      gls_val.pop_back();
      sensors[G2x_s] = gls_val.back();
      gls_val.pop_back();
    }
    //------------------------Add GoalSensor by Ren-------------------

    //------------------------Add Orientation Sensor by Ren-------------------
        std::list<sensor> Ori_lst =  OrientationSensor->getList();
        double ori1,ori2,ori3;
        ori1 = Ori_lst.front();
        Ori_lst.pop_front();
        ori2 = Ori_lst.front();
        Ori_lst.pop_front();
        ori3 = Ori_lst.front();

        sensors[BX_ori] = ori1; //atan2(ori2,ori1)*180/M_PI;
        sensors[BY_ori] = ori2;
        sensors[BZ_ori] = ori3;

        sensors[G0angleyaw_s] = atan2(ori2,ori1)*180/M_PI;
        Ori_lst.clear();
        //------------------------Add Orientation Sensor by Ren------------------

#ifdef VERBOSE
    std::cerr << "AmosII::getSensors END\n";
#endif
    return AMOSII_SENSOR_MAX;
  }
  ;

  void AmosFour::placeIntern(const osg::Matrix& pose) {
#ifdef VERBOSE
    std::cerr << "AmosII::place BEGIN\n";
#endif
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position
    // is 0
    //Matrix p2 = pose * ROTM(0, 0, conf.legLength + conf.legLength/8);
    osg::Matrix p = pose
        * TRANSM(0, 0, conf.tebiaLength - conf.shoulderHeight + 2 * conf.tebiaRadius + conf.footRadius);
    create(p);
#ifdef VERBOSE
    std::cerr << "AmosII::place END\n";
#endif
  }
  ;

  /**
   * updates the osg notes
   */
  void AmosFour::update() {
    OdeRobot::update();
#ifdef VERBOSE
    std::cerr << "AmosII::update BEGIN\n";
#endif
    assert(created);
    // robot must exist

    for (PrimitiveList::iterator i = objects.begin(); i != objects.end(); i++) {
      if (*i)
        (*i)->update();
    }
    for (JointList::iterator i = joints.begin(); i != joints.end(); i++) {
      if (*i)
        (*i)->update();
    }
    // update the graphical representation of the sensorbank
    irSensorBank->update();

    for (int i = 0; i < LEG_POS_MAX; i++) {
      if (legContactSensors[LegPos(i)])
        legContactSensors[LegPos(i)]->update();
    }

#ifdef VERBOSE
    std::cerr << "AmosII::update END\n";
#endif
  }
  ;

  double AmosFour::getMassOfRobot() {

    double totalMass = 0.0;

    for (unsigned int i = 0; i < objects.size(); i++) {
      dMass massOfobject;
      dBodyGetMass(objects[i]->getBody(), &massOfobject);
      totalMass += massOfobject.mass;
    }
    return totalMass;
  }

  void AmosFour::sense(GlobalData& globalData) {
    OdeRobot::sense(globalData);
    irSensorBank->sense(globalData);

    for (int i = 0; i < LEG_POS_MAX; i++) {
      if (legContactSensors[LegPos(i)])
        legContactSensors[LegPos(i)]->sense(globalData);
    }
  }

  /**
   * this function is called in each timestep. It should perform robot-
   * internal checks, like space-internal collision detection, sensor
   * resets/update etc.
   *
   * @param global structure that contains global data from the simulation
   * environment
   */
  void AmosFour::doInternalStuff(GlobalData& global) {

#ifdef VERBOSE
    std::cerr << "AmosII::doInternalStuff BEGIN\n";
#endif

    // update statistics
    position = getPosition();

    // passive servos have to be set to zero in every time step so they work
    // as springs
    for (ServoList::iterator it = passiveServos.begin(); it != passiveServos.end(); it++) {
      (*it)->set(0.0);
    }

#ifdef VERBOSE
    std::cerr << "AmosII::doInternalStuff END\n";
#endif
  }

  Primitive* AmosFour::getMainPrimitive() const {
    return center;
  }

  /**
   * creates vehicle at desired position
   *
   * @param pos struct Position with desired position
   */
  void AmosFour::create(const osg::Matrix& pose) {
#ifdef VERBOSE
    std::cerr << "AmosII::create BEGIN\n";
#endif
    if (created) {
      destroy();
    }

    // we want legs colliding with other legs, so we set internal collision
    // flag to "false".
    odeHandle.createNewSimpleSpace(parentspace, false);

    // color of robot
    osgHandle = osgHandle.changeColor("robot1");

    // color of joint axis
    OsgHandle osgHandleJoint = osgHandle.changeColor("joint");

    // change Material substance
    OdeHandle odeHandleBody = odeHandle;
    odeHandleBody.substance.toMetal(3.0);

    //get a representation of the origin
    const Pos nullpos(0, 0, 0);
    /**********************************************************************/
    /*  create body                                                       */
    /**********************************************************************/

    /** central position of the trunk */
    const osg::Matrix trunkPos = pose;

//    if (conf.useBack) {
//      front = new Box(conf.frontLength, conf.width, conf.height);
//      front->setTexture(conf.bodyTexture);
//      front->init(odeHandleBody, conf.frontMass, osgHandle.changeColor("robot2"));
//      osg::Matrix frontPos = TRANSM(conf.size / 2 - conf.frontLength / 2, 0, 0) * trunkPos;
//      front->setPose(frontPos);
//      objects.push_back(front);
//
//      center = new Box(conf.size - conf.frontLength, conf.width, conf.height);
//      center->setTexture(conf.bodyTexture);
//      center->init(odeHandleBody, conf.trunkMass - conf.frontMass, osgHandle.changeColor("robot2"));
//      osg::Matrix centerPos = TRANSM(-conf.size / 2 + (conf.size - conf.frontLength) / 2, 0, 0) * trunkPos;
//      center->setPose(centerPos);
//      objects.push_back(center);
//      const Axis axis = Axis(0, 1, 0) * frontPos;
//      // create the joint from front to center part of trunk
//      HingeJoint* k = new HingeJoint(front, center, nullpos * TRANSM(-conf.frontLength / 2, 0, 0) * frontPos, axis);
//      k->init(odeHandle, osgHandleJoint, true, conf.width * 1.05);
//      joints.push_back(k);
//      // parameters are set later
//      OneAxisServo* servo = new OneAxisServoVel(odeHandle, k, -1, 1, 1, 0.01, 0, 1.0);
//      servos[BJ_m] = servo;
//      backboneServo = servo;
//    } else {
//      trunk = new Box(conf.size, conf.width, conf.height);
//      trunk->setTexture(conf.bodyTexture);
//      trunk->init(odeHandleBody, conf.trunkMass, osgHandle.changeColor("robot2"));
//      trunk->setPose(trunkPos);
//      objects.push_back(trunk);
//      front = trunk;
//      center = trunk;
//    }

    center = new Box(conf.size - conf.frontLength, conf.width, conf.height);
    center->setTexture(conf.bodyTexture);
    center->init(odeHandleBody, conf.trunkMass - conf.frontMass, osgHandle.changeColor("robot2"));
//    osg::Matrix centerPos = TRANSM(-conf.size / 2 + (conf.size - conf.frontLength) / 2, 0, 0) * trunkPos;
//    center->setPose(centerPos);
    center->setPose(trunkPos);
    objects.push_back(center);

    if (conf.useLocalVelSensor) {
      // create speedsensor
      speedsensor = new SpeedSensor(1.0, SpeedSensor::TranslationalRel, SpeedSensor::XY);
      //initialize the speedsensor
      speedsensor->init(center);
    }

    // initialize the infrared sensors
    irSensorBank = new RaySensorBank();
    irSensorBank->setInitData(odeHandle, osgHandle, TRANSM(0,0,0));
    irSensorBank->init(0);

    // ultrasonic sensors at Front part
    usSensorFrontRight = new IRSensor();
    irSensorBank->registerSensor(usSensorFrontRight, center,
        ROTM(M_PI / 2, conf.usAngleX, conf.usAngleY, 0)
        * TRANSM(0.3 * (conf.useBack ? conf.frontLength : conf.size), -0.25 * conf.width, -0.45 * conf.height),
        conf.usRangeFront, RaySensor::drawRay);

    usSensorFrontLeft = new IRSensor();
    irSensorBank->registerSensor(usSensorFrontLeft, center,
        ROTM(M_PI / 2, (conf.usParallel ? 1 : -1) * conf.usAngleX, conf.usAngleY, 0)
        * TRANSM(0.3 * (conf.useBack ? conf.frontLength : conf.size), 0.25 * conf.width, -0.45 * conf.height),
        conf.usRangeFront, RaySensor::drawRay);

    /************************************
     * LEGS
     ***********************************/

    const osg::Matrix m0 = pose;

    const double l0 = conf.shoulderLength;
    const double t0 = conf.shoulderRadius;
    const double l1 = conf.coxaLength;
    const double t1 = conf.coxaRadius;
    const double l2 = conf.secondLength;
    const double t2 = conf.secondRadius;
    const double l3 = conf.tebiaLength - 2 * conf.tebiaRadius - conf.footRange;
    const double t3 = conf.tebiaRadius;
    const double l4 = 2 * conf.tebiaRadius + conf.footRange - conf.footRadius;
    const double t4 = conf.footRadius;

    std::map<LegPos, osg::Matrix> legtrunkconnections;
    std::map<LegPos, osg::Matrix> shouldertrunkconnections;

    for (int i = 0; i < LEG_POS_MAX; i++) {
      LegPos leg = LegPos(i);

      // +1 for L1,L2,L3, -1 for R1,R2,R3
      const double lr = (leg == L1 || leg == L2) - (leg == R1 || leg == R2);
      // create 3d-coordinates for the leg-trunk connection:
      Pos pos = Pos(
          // from (0,0,0) we go down x-axis, make two legs then up
          // legdist1 and so on
          -conf.size * 10 / 43.0 + (leg == L2 || leg == R2) * 0 + (leg == L1 || leg == R1) * conf.legdist1,
//          + (leg == L0 || leg == R0) * (conf.legdist1 + conf.legdist2),
          // switch left or right side of trunk for each leg
          lr * conf.width / 2,
          // height of leg fixation to trunk (trunk bottom sits at
          // total legLength)
          -conf.height / 2 + conf.shoulderHeight);

      // get a coordinate system at the position pos by rotating such that
      // z-axis points toward trunk, pose is where the robot will be
      // placed so we begin there.
      legtrunkconnections[leg] = ROTM(M_PI / 2, lr, 0, 0) * TRANSM(pos) * pose;

      // we create a transformation matrix that represents the
      // transformation from the trunk center to the trunk-shoulder
      // connections. we need it because we need the coordinates relative
      // to the trunk to create one body including the shoulders
//      if (conf.useBack)
//        shouldertrunkconnections[leg] = ROTM(M_PI / 2, lr, 0, 0)
//        * TRANSM(conf.frontLength / 2.0 - (leg == L0 || leg == R0) * conf.size / 2.0, 0, 0) * TRANSM(pos);
//      else
        shouldertrunkconnections[leg] = ROTM(M_PI / 2, lr, 0, 0) * TRANSM(pos);
    }

    // if wanted, leg trunk connections are rotated here:
    legtrunkconnections[R2] = ROTM(conf.rLegRotAngle, 0, 0, 1) * ROTM(conf.rLegTrunkAngleH, 1, 0, 0)
            * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[R2];
    legtrunkconnections[L2] = ROTM(conf.rLegRotAngle, 0, 0, -1) * ROTM(conf.rLegTrunkAngleH, -1, 0, 0)
            * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[L2];
    legtrunkconnections[R1] = ROTM(conf.mLegRotAngle, 0, 0, 1) * ROTM(conf.mLegTrunkAngleH, 1, 0, 0)
            * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[R1];
    legtrunkconnections[L1] = ROTM(conf.mLegRotAngle, 0, 0, -1) * ROTM(conf.mLegTrunkAngleH, -1, 0, 0)
            * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[L1];
//    legtrunkconnections[R0] = ROTM(conf.fLegRotAngle, 0, 0, 1) * ROTM(conf.fLegTrunkAngleH, 1, 0, 0)
//            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[R0];
//    legtrunkconnections[L0] = ROTM(conf.fLegRotAngle, 0, 0, -1) * ROTM(conf.fLegTrunkAngleH, -1, 0, 0)
//            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[L0];

    // also the relative coordinates for the shoulders
    shouldertrunkconnections[R2] = ROTM(conf.rLegRotAngle, 0, 0, 1) * ROTM(conf.rLegTrunkAngleH, 1, 0, 0)
            * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[R2];
    shouldertrunkconnections[L2] = ROTM(conf.rLegRotAngle, 0, 0, -1) * ROTM(conf.rLegTrunkAngleH, -1, 0, 0)
            * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[L2];
    shouldertrunkconnections[R1] = ROTM(conf.mLegRotAngle, 0, 0, 1) * ROTM(conf.mLegTrunkAngleH, 1, 0, 0)
            * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[R1];
    shouldertrunkconnections[L1] = ROTM(conf.mLegRotAngle, 0, 0, -1) * ROTM(conf.mLegTrunkAngleH, -1, 0, 0)
            * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[L1];
//    shouldertrunkconnections[R0] = ROTM(conf.fLegRotAngle, 0, 0, 1) * ROTM(conf.fLegTrunkAngleH, 1, 0, 0)
//            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[R0];
//    shouldertrunkconnections[L0] = ROTM(conf.fLegRotAngle, 0, 0, -1) * ROTM(conf.fLegTrunkAngleH, -1, 0, 0)
//            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[L0];

    // create the legs
    for (int i = 0; i < LEG_POS_MAX; i++) {
      LegPos leg = LegPos(i);
      if (legPosUsage[leg] == LEG) {
        // get a representation of the origin
        const Pos nullpos(0, 0, 0);

        // +1 for R1,R2,R3, -1 for L1,L2,L3
        const double pmrl = (leg == R1 || leg == R2) - (leg == L1 || leg == L2);

        osg::Matrix c1;

        // m0 is the position where the center of mass of the zeroth limb
        // capsule is placed
        osg::Matrix m0;

        if (conf.useShoulder) {
          //shift connection of coxa outward
          c1 = TRANSM(0, 0, -l0) * legtrunkconnections[leg];
          //create shoulder
          Primitive * should = new Capsule(t0, l0);
          should->setTexture(conf.texture);
          // add shoulder to trunk body
          // the shoulder's pose has to be given relative to the trunk's pose
          // add the first four shoulders to center the other two to front
          Primitive * trans = new Transform(center, should,
              TRANSM(0, 0, -l0 / 2) * shouldertrunkconnections[leg]);
          trans->init(odeHandle, conf.shoulderMass, osgHandle);
          legs[leg].shoulder = trans;
          objects.push_back(trans);
        } else {
          //first limb data
          c1 = legtrunkconnections[leg];
        }

        // m1 is the position where the center of mass of the first limb
        // capsule is placed
        osg::Matrix m1 = TRANSM(0, 0, -l1 / 2) * c1;

        // calculate anchor of the first joint
        const osg::Vec3 anchor1 = nullpos * c1;
        // and it's axis (multiplication with c1 indicates in which
        // (local) coordinate system it is)
        const Axis axis1 = Axis(0, 1, 0) * c1;

        // proceed along the leg (and the respective z-axis) for second
        // limb
        osg::Matrix c2 = TRANSM(0, 0, -l1 / 2) * m1;
        osg::Matrix m2 = TRANSM(0, 0, -l2 / 2) * c2;
        const osg::Vec3 anchor2 = nullpos * c2;
        const Axis axis2 = Axis(pmrl, 0, 0) * c2;

        //and third
        osg::Matrix c3 = TRANSM(0, 0, -l2 / 2) * m2;
        osg::Matrix m3 = TRANSM(0, 0, -l3 / 2) * c3;
        const osg::Vec3 anchor3 = nullpos * c3;
        const Axis axis3 = Axis(pmrl, 0, 0) * c3;

        // now create first limp
        Primitive* coxaThorax;
        // create upper limp with radius t1 and length l1 (length refers
        // only to length of the cylinder without the semispheres at
        // both ends)
        coxaThorax = new Capsule(t1, l1);
        coxaThorax->setTexture(conf.texture);
        coxaThorax->init(odeHandle, conf.coxaMass, osgHandle);
        //put it at m1
        coxaThorax->setPose(m1);
        legs[leg].coxa = coxaThorax;
        objects.push_back(coxaThorax);
        if (conf.useShoulder) {
          odeHandle.addIgnoredPair(legs[leg].shoulder, coxaThorax);
        }
        // powered hip joint of trunk to first limb
        HingeJoint* j = new HingeJoint(center, coxaThorax, anchor1, -axis1);
        j->init(odeHandle, osgHandleJoint, true, t1 * 2.1);
        joints.push_back(j);
        // create motor, overwrite the jointLimit argument with 1.0
        // because it is less obscure and setMinMax makes mistakes
        // otherwise. Parameters are set later
        OneAxisServo * servo1 = new OneAxisServoVel(odeHandle, j, -1, 1, 1, 0.01, 0, 1.0);
        //PUSH THIS STUFF BACK INTO STH!!!! not hipservos obviously
        legs[leg].tcServo = servo1;
        servos[getMotorName(leg, TC)] = servo1;

        // second limb
        Primitive* secondThorax;
        secondThorax = new Capsule(t2, l2);
        secondThorax->setTexture(conf.texture);
        secondThorax->init(odeHandle, conf.secondMass, osgHandle);
        secondThorax->setPose(m2);
        legs[leg].second = secondThorax;
        objects.push_back(secondThorax);

        // create the joint from first to second limb (coxa to second)
        HingeJoint* k = new HingeJoint(coxaThorax, secondThorax, anchor2, -axis2);
        k->init(odeHandle, osgHandleJoint, true, t1 * 2.1);
        legs[leg].ctrJoint = k;
        joints.push_back(k);
        /** parameters are set later */
        OneAxisServo * servo2 = new OneAxisServoVel(odeHandle, k, -1, 1, 1, 0.01, 0, 1.0);
        //PUSH THIS STUFF BACK INTO STH!!!! not hipservos obviously
        legs[leg].ctrServo = servo2;
        servos[getMotorName(leg, CTR)] = servo2;

        // third limb
        Primitive* tebia;
        tebia = new Capsule(t3, l3);
        tebia->setTexture(conf.texture);
        tebia->init(odeHandle, conf.tebiaMass, osgHandle);
        tebia->setPose(m3);
        //        tebiaPos.push_back(tebia->getPosition());
        legs[leg].tibia = tebia;
        objects.push_back(tebia);

        // IR sensor at each leg
        IRSensor* sensor = new IRSensor();
        irSensorBank->registerSensor(sensor, tebia,
            ROTM(M_PI / 2, 0, 1, 0) * TRANSM(1.01 * t3, 0, -0.2 * conf.tebiaLength), conf.irRangeLeg,
            RaySensor::drawRay);
        irLegSensors[leg] = sensor;

        // springy knee joint
        HingeJoint* l = new HingeJoint(secondThorax, tebia, anchor3, -axis3);
        l->init(odeHandle, osgHandleJoint, true, t3 * 2.1);
        legs[leg].ftiJoint = l;
        joints.push_back(l);
        // servo used as a spring
        /** parameters are set later */
        OneAxisServo * servo3 = new OneAxisServoVel(odeHandle, l, -1, 1, 1, 0.01, 0, 1.0);
        legs[leg].ftiServo = servo3;
        servos[getMotorName(leg, FTI)] = servo3;

        //spring foot at the end
        if (conf.useFoot) {
          osg::Matrix c4 = TRANSM(0, 0, -l3 / 2 - 2 * conf.tebiaRadius - conf.footRange + conf.footRadius) * m3;
          osg::Matrix m4 = TRANSM(0, 0, -conf.footSpringPreload) * c4;

          const osg::Vec3 anchor4 = nullpos * m4;
          const Axis axis4 = Axis(0, 0, -1) * c4;

          OdeHandle my_odeHandle = odeHandle;
          if (conf.rubberFeet) {
            const Substance FootSubstance(3.0, 0.0, 500.0, 0.1);
            my_odeHandle.substance = FootSubstance;
          }

          Primitive* foot;
          foot = new Capsule(t4, l4);
          foot->setTexture(conf.texture);
          foot->init(my_odeHandle, conf.footMass, osgHandle);
          foot->setPose(m4);
          //            footPos.push_back(foot->getPosition());
          legs[leg].foot = foot;
          objects.push_back(foot);

          SliderJoint* m = new SliderJoint(tebia, foot, anchor4, axis4);
          m->init(odeHandle, osgHandleJoint, true, t3, true);
          legs[leg].footJoint = m;
          joints.push_back(m);

          /** parameters are set later */
          Spring* spring = new Spring(m, -1, 1, 1);
          legs[leg].footSpring = spring;
          passiveServos.push_back(spring);
          odeHandle.addIgnoredPair(secondThorax, foot);

          legContactSensors[LegPos(i)] = new ContactSensor(conf.legContactSensorIsBinary, 50, 1.01 * t4, true);
          legContactSensors[LegPos(i)]->setInitData(odeHandle, osgHandle, TRANSM(0, 0, -0.5 * l4));
          legContactSensors[LegPos(i)]->init(foot);
          odeHandle.addIgnoredPair(tebia, legContactSensors[LegPos(i)]->getTransformObject());
        }
      } else if (legPosUsage[leg] == WHEEL) {
        //Sphere* sph = new Sphere(radius);
        Cylinder* wheel = new Cylinder(conf.wheel_radius, conf.wheel_width);
        wheel->setTexture(conf.texture);
        OsgHandle bosghandle = osgHandle;
        wheel->init(odeHandle, conf.wheel_mass, // mass
            bosghandle.changeColor("robot2"));
        const double pmlr = (leg == L1 || leg == L2) - (leg == R1 || leg == R2);
        Pos pos = Pos(
            // from (0,0,0) we go down x-axis, make two legs then up
            // legdist1 and so on
            -conf.size * 16.5 / 43.0 + (leg == L2 || leg == R2) * 0 + (leg == L1 || leg == R1) * conf.legdist1,
//            + (leg == L0 || leg == R0) * (conf.legdist1 + conf.legdist2),
            // switch left or right side of trunk for each leg
            pmlr * conf.width,
            // height of wheel fixation to trunk
            -0.7 * conf.height + conf.wheel_radius);

        wheel->setPose(ROTM(0.5 * M_PI, 1, 0, 0) * TRANSM(pos) * trunkPos);
        objects.push_back(wheel);
        // generate  joints to connect the wheels to the body
        Pos anchor(dBodyGetPosition(wheel->getBody()));
        anchor -= Pos(0, 0, 0);
        HingeJoint * wheeljoint = new HingeJoint(objects[0], wheel, anchor, Axis(0, 1, 0) * trunkPos);
        wheeljoint->init(odeHandle, osgHandleJoint, true, 1.1 * conf.wheel_width);
        joints.push_back(wheeljoint);
      }
    }

    //-----------------add GoalSensor by Ren------------------------
    if (conf.GoalSensor_references.size()>0)
    {
      // Relative position sensor
      for (std::vector<Primitive*>::iterator it = conf.GoalSensor_references.begin(); it<conf.GoalSensor_references.end();it++)
      {
        // Using 0 as second parameter if exact distance should be given as linear sensor
        RelativePositionSensor GoalSensor_tmp(1, 0,Sensor::X|Sensor::Y|Sensor::Z, true);
        //max distance for normalization
        //exponent for sensor characteristic
        //dimensions to sense
        //use Z as x-coordinate ( robot was created with vertical capsule or something like that)
        //local_coordinates
        GoalSensor_tmp.setReference(*it);
        GoalSensor.push_back(GoalSensor_tmp);
        //sensorno += rpos_sens_tmp.getSensorNumber(); // increase sensornumber of robot, have been declared in sensormotordefinition
      }
      GoalSensor_active = true;
    }
    else
    {
      GoalSensor_active =false;
    }
    //----------------------Goal Sensor by Ren-----------------------

    // --------------Add Goal Sensor by Ren -------------------
    // Relative position sensor
    if (GoalSensor_active) {
      for (std::vector<RelativePositionSensor>::iterator it = GoalSensor.begin(); it < GoalSensor.end(); it++) {
        it->init(center); // connect sensor to main body
      }
    }
    OrientationSensor = new AxisOrientationSensor(AxisOrientationSensor::Axis,Sensor::X |Sensor::Y | Sensor::Z);
    OrientationSensor->init(center);
    // --------------Add Goal Sensor by Ren -------------------

    setParam("dummy", 0); // apply all parameters.

    created = true;
#ifdef VERBOSE
    std::cerr << "AmosII::create END\n";
#endif
  }
  ;

  /** destroys vehicle and space
   */
  void AmosFour::destroy() {
    if (created) {
#ifdef VERBOSE
      std::cerr << "begin AmosII::destroy\n";
#endif
      // delete contact sensors
      for (int i = 0; i < LEG_POS_MAX; i++) {
        if (legContactSensors[LegPos(i)])
          delete legContactSensors[LegPos(i)];
      }
      legContactSensors.clear();

      // remove all ignored pairs (brute force method)
      for (PrimitiveList::iterator i = objects.begin(); i != objects.end(); i++) {
        for (PrimitiveList::iterator j = objects.begin(); j != objects.end(); j++) {
          if (odeHandle.isIgnoredPair((*i)->getGeom(), (*j)->getGeom())) {
            odeHandle.removeIgnoredPair((*i)->getGeom(), (*j)->getGeom());
          }
        }

      }

      irSensorBank->clear();
      delete irSensorBank;

      if (speedsensor) {
        delete speedsensor;
        speedsensor = 0;
      }

      for (MotorMap::iterator it = servos.begin(); it != servos.end(); it++) {
        if (it->second)
          delete (it->second);
      }
      servos.clear();

      for (ServoList::iterator it = passiveServos.begin(); it != passiveServos.end(); it++) {
        if (*it)
          delete (*it);
      }
      passiveServos.clear();

      for (JointList::iterator i = joints.begin(); i != joints.end(); i++) {
        if (*i)
          delete *i;
      }
      joints.clear();

      for (PrimitiveList::iterator i = objects.begin(); i != objects.end(); i++) {
        if (*i)
          delete *i;
      }
      objects.clear();

      //should all be empty as objects were cleared:
      legs.clear();

      //------------------ delete GoalSensor here by Ren--------------------
      GoalSensor.clear();
      //------------------ delete GoalSensor here by Ren--------------------

      odeHandle.deleteSpace();
#ifdef VERBOSE
      std::cerr << "end AmosII::destroy\n";
#endif
    }

    created = false;
  }

  bool AmosFour::setParam(const paramkey& key, paramval val) {
#ifdef VERBOSE
    std::cerr << "AmosII::setParam BEGIN\n";
#endif
    // the parameters are assigned here
    bool rv = Configurable::setParam(key, val);

    // we simply set all parameters here
    for (LegMap::iterator it = legs.begin(); it != legs.end(); it++) {
      Spring * const footspring = it->second.footSpring;
      if (footspring) {
        footspring->setPower(conf.footPower);
        footspring->setDamping(conf.footDamping);
        footspring->setMaxVel(conf.footMaxVel);
        //yes, min is up, up is negative
        footspring->setMinMax(conf.footSpringLimitD, conf.footSpringLimitU);
      }

      OneAxisServo * tc = it->second.tcServo;
      if (tc) {
        tc->setPower(conf.coxaPower);
        tc->setDamping(conf.coxaDamping);
        tc->setMaxVel(conf.coxaMaxVel);
        if (it->first == L2 || it->first == R2)
          tc->setMinMax(conf.rcoxaJointLimitF, conf.rcoxaJointLimitB);
        if (it->first == L1 || it->first == R1)
          tc->setMinMax(conf.mcoxaJointLimitF, conf.mcoxaJointLimitB);
//        if (it->first == L0 || it->first == R0)
//          tc->setMinMax(conf.fcoxaJointLimitF, conf.fcoxaJointLimitB);
      }

      OneAxisServo * ctr = it->second.ctrServo;
      if (ctr) {
        ctr->setPower(conf.secondPower);
        ctr->setDamping(conf.secondDamping);
        ctr->setMaxVel(conf.secondMaxVel);
        //yes, min is up, up is negative
        ctr->setMinMax(conf.secondJointLimitU, conf.secondJointLimitD);
      }

      OneAxisServo * fti = it->second.ftiServo;
      if (fti) {
        fti->setPower(conf.tebiaPower);
        fti->setDamping(conf.tebiaDamping);
        fti->setMaxVel(conf.tebiaMaxVel);
        //yes, min is up, up is negative
        fti->setMinMax(conf.tebiaJointLimitU, conf.tebiaJointLimitD);
      }
    }

    if (backboneServo) {
      backboneServo->setPower(conf.backPower);
      backboneServo->setDamping(conf.backDamping);
      backboneServo->setMaxVel(conf.backMaxVel);
      backboneServo->setMinMax(conf.backJointLimitU, conf.backJointLimitD);
    }

#ifdef VERBOSE
    std::cerr << "AmosII::setParam END\n";
#endif
    return rv;
  }

  /**
   * returns the MotorName enum value for the given joint at the given
   * leg. If the value for leg or joint are not valid AMOSII_MOTOR_MAX
   * is returned.
   *
   * @param leg leg position
   * @param joint leg joint type
   * @return the motor name value or AMOSII_MOTOR_MAX if parameters are
   *         invalid
   */
  AmosFour::MotorName AmosFour::getMotorName(LegPos leg, LegJointType joint) {
//    if (leg == L0 && joint == TC)
//      return TL0_m;
//    if (leg == L0 && joint == CTR)
//      return CL0_m;
//    if (leg == L0 && joint == FTI)
//      return FL0_m;
    if (leg == L1 && joint == TC)
      return TL1_m;
    if (leg == L1 && joint == CTR)
      return CL1_m;
    if (leg == L1 && joint == FTI)
      return FL1_m;
    if (leg == L2 && joint == TC)
      return TL2_m;
    if (leg == L2 && joint == CTR)
      return CL2_m;
    if (leg == L2 && joint == FTI)
      return FL2_m;
//    if (leg == R0 && joint == TC)
//      return TR0_m;
//    if (leg == R0 && joint == CTR)
//      return CR0_m;
//    if (leg == R0 && joint == FTI)
//      return FR0_m;
    if (leg == R1 && joint == TC)
      return TR1_m;
    if (leg == R1 && joint == CTR)
      return CR1_m;
    if (leg == R1 && joint == FTI)
      return FR1_m;
    if (leg == R2 && joint == TC)
      return TR2_m;
    if (leg == R2 && joint == CTR)
      return CR2_m;
    if (leg == R2 && joint == FTI)
      return FR2_m;
    return AMOSII_MOTOR_MAX;
  }

  /**
   * returns the joint type of the given motor. If the given motor name
   * is not associated with a leg joint JOINT_TYPE_MAX is returend and a
   * warning is given out.
   *
   * @param MotorName name of the motor
   * @return joint type controlled by this motor or JOINT_TYPE_MAX if
   *         MotorName is invalid
   */
  AmosFour::LegJointType AmosFour::getLegJointType(MotorName name) {
    assert(name!=AMOSII_MOTOR_MAX);
    switch (name) {
      case TR0_m:
      case TR1_m:
      case TR2_m:
      case TL0_m:
      case TL1_m:
      case TL2_m:
        return TC;
      case CR0_m:
      case CR1_m:
      case CR2_m:
      case CL0_m:
      case CL1_m:
      case CL2_m:
        return CTR;
      case FR0_m:
      case FR1_m:
      case FR2_m:
      case FL0_m:
      case FL1_m:
      case FL2_m:
        return FTI;
      default:
        std::cerr << "WARNING: point in AmosII::getMotorJointType reached " << "that should not" << std::endl;
        return LEG_JOINT_TYPE_MAX;
    }
  }

  /**
   * Returns the leg of the given motor. If the given motor name is not
   * associated wit a leg LEG_POS_MAX is returned and a warning is given
   * out
   *
   * @param MotorName name of the motor
   * @return the leg on which this motor operates or LEG_POS_MAX if
   *         MotorName is invalid
   */
  AmosFour::LegPos AmosFour::getMotorLegPos(MotorName name) {
    assert(name!=AMOSII_MOTOR_MAX);
    switch (name) {
//      case TR0_m:
//      case CR0_m:
//      case FR0_m:
//        return R0;
      case TR1_m:
      case CR1_m:
      case FR1_m:
        return R1;
      case TR2_m:
      case CR2_m:
      case FR2_m:
        return R2;
//      case TL0_m:
//      case CL0_m:
//      case FL0_m:
//        return L0;
      case TL1_m:
      case CL1_m:
      case FL1_m:
        return L1;
      case TL2_m:
      case CL2_m:
      case FL2_m:
        return L2;
      default:
        std::cerr << "WARNING: point in AmosII::getMotorLegPos reached " << "that should not" << std::endl;
        return LEG_POS_MAX;
    }
  }

  void AmosFour::setLegPosUsage(LegPos leg, LegPosUsage usage) {
    legPosUsage[leg] = usage;
  }

  AmosFourConf AmosFour::getDefaultConf(double _scale, bool _useShoulder, bool _useFoot, bool _useBack) {
    return getAmosIIv2Conf(_scale, _useShoulder, _useFoot, _useBack);
  }

  AmosFourConf AmosFour::getAmosIIv2Conf(double _scale, bool _useShoulder, bool _useFoot, bool _useBack) {

    AmosFourConf c;

    // "Internal" variable storing the currently used version
    c.amos_version = 2;
    // use shoulder (fixed joint between legs and trunk)
    c.useShoulder = _useShoulder;
    c.useTebiaJoints = 0;
    //create springs at the end of the legs
    c.useFoot = _useFoot;
    //create a joint in the back
    c.useBack = _useBack;
    c.rubberFeet = false;
    c.useLocalVelSensor = 0;
    c.legContactSensorIsBinary = false;

    // the trunk length. this scales the whole robot! all parts' sizes,
    // masses, and forces will be adapted!!
    c.size = 0.43 * _scale;
    //trunk width
    c.width = 7.0 / 43.0 * c.size;
    //trunk height
    c.height = 6.5 / 43.0 * c.size;
    c.frontLength = 12.0 / 43.0 * c.size;
    // we use as density the original trunk weight divided by the original
    // volume

    //Change mass by KOH to 3.0
    const double density = 3.0 / (0.43 * 0.07 * 0.065); //2.2 / (0.43 * 0.07 * 0.065);

    c.trunkMass = density * c.size * c.width * c.height;
    // use the original trunk to total mass ratio
    const double mass = 5.758 / 2.2 * c.trunkMass;
    c.frontMass = c.trunkMass * c.frontLength / c.size;
    // distribute the rest of the weight like this for now */
    c.shoulderMass = (mass - c.trunkMass) / (6 * (3.0 + c.useShoulder)) * (20.0 - c.useFoot) / 20.0;
    c.coxaMass = c.shoulderMass;
    c.secondMass = c.shoulderMass;
    c.tebiaMass = c.shoulderMass;
    // foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by
    // 3 or 4)
    c.footMass = (mass - c.trunkMass) / 6 * c.useFoot / 20.0;

    //As real robot!!
    const double shoulderHeight_cm = 6.5;
    //shoulder height "4.5 wrong" --> correct=6.5 cm from rotating point
    c.shoulderHeight = shoulderHeight_cm / 6.5 * c.height;

    // distance between hindlegs and middle legs
    c.legdist1 = 19.0 / 43.0 * c.size;
    // distance between middle legs and front legs
    c.legdist2 = 15.0 / 43.0 * c.size;

    // configure the wheels (if used). They don't have any counterpart in
    // reality, so the chosen values are arbitrary
    c.wheel_radius = 0.10 * c.size;
    c.wheel_width = 0.04 * c.size;
    c.wheel_mass = (mass - c.trunkMass) / 6.0;

    // -----------------------
    // 1) Biomechanics
    // Manual setting adjustable joint positions at the body
    // -----------------------

    // amosII has a fixed but adjustable joint that decides how the legs
    // extend from the trunk. Here you can adjust these joints

    // ------------- Front legs -------------
    // angle (in rad) around vertical axis at leg-trunk fixation 0:
    // perpendicular
    // => forward/backward
    c.fLegTrunkAngleV = 0.0;
    // angle around horizontal axis at leg-trunk fixation 0: perpendicular
    // => upward/downward
    c.fLegTrunkAngleH = 0.0;
    // rotation of leg around own axis 0: first joint axis is vertical
    // => till
    c.fLegRotAngle = 0.0;

    // ------------- Middle legs ----------------
    // => forward/backward
    c.mLegTrunkAngleV = 0.0;
    // => upward/downward
    c.mLegTrunkAngleH = 0.0;
    // => till
    c.mLegRotAngle = 0.0;

    // ------------- Rear legs ------------------
    // => forward/backward
    c.rLegTrunkAngleV = 0.0;
    // => upward/downward
    c.rLegTrunkAngleH = 0.0;
    // => till
    c.rLegRotAngle = 0.0;

    // be careful changing the following dimension, they may break the
    // simulation!! (they shouldn't but they do)
    const double shoulderLength_cm = 4.5;
    c.shoulderLength = shoulderLength_cm / 43.0 * c.size;
    c.shoulderRadius = .03 * c.size;

    const double coxaLength_cm = 3.5;
    c.coxaLength = coxaLength_cm / 43.0 * c.size;
    c.coxaRadius = .04 * c.size;

    const double secondLength_cm = 6.0;
    c.secondLength = secondLength_cm / 43.0 * c.size;
    c.secondRadius = .03 * c.size;
    c.tebiaRadius = 1.3 / 43.0 * c.size;

    const double tebiaLength_cm = 11.5; // 3)
    c.tebiaLength = tebiaLength_cm / 43.0 * c.size;

    // this determines the limit of the footspring
    c.footRange = .2 / 43.0 * c.size;
    c.footRadius = 1.5 / 43.0 * c.size;

    // -----------------------
    // 2) Joint Limits
    // Setting Max, Min of each joint with respect to real
    // -----------------------

    //Similar to real robot
    //-45 deg; downward (+) MIN
    c.backJointLimitD = M_PI / 180 * 45.0;
    // 45 deg; upward (-) MAX
    c.backJointLimitU = -M_PI / 180 * 45.0;

    // 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
    c.fcoxaJointLimitF = -M_PI / 180.0 * 70.0;
    //-70 deg; backward (+) MIN --> normal walking range -10 deg MIN
    c.fcoxaJointLimitB = M_PI / 180.0 * 70.0;

    //60 deg; forward (-) MAX --> normal walking range 30 deg MAX
    c.mcoxaJointLimitF = -M_PI / 180.0 * 60.0;
    //60 deg; backward (+) MIN --> normal walking range -40 deg MIN
    c.mcoxaJointLimitB = M_PI / 180 * 60.0;

    //70 deg; forward (-) MAX --> normal walking range 60 deg MAX
    c.rcoxaJointLimitF = -M_PI / 180.0 * 70.0;
    //70 deg; backward (+) MIN --> normal walking range -10 deg MIN
    c.rcoxaJointLimitB = M_PI / 180.0 * 70.0;

    // 70 deg; downward (+) MIN
    c.secondJointLimitD = M_PI / 180.0 * 75.0;
    // 70 deg upward (-) MAX
    c.secondJointLimitU = -M_PI / 180.0 * 75.0;

    //130 deg downward; (+) MIN
    c.tebiaJointLimitD = M_PI / 180.0 * 130.0;
    // 20 deg  downward; (+) MAX
    c.tebiaJointLimitU = M_PI / 180.0 * 20.0;

    // -----------------------
    // 3) Motors
    // Motor power and joint stiffness
    // -----------------------

    c.footSpringPreload = 8.0 / 43.0 * c.size;
    // negative is downwards (spring extends)
    c.footSpringLimitD = c.footSpringPreload;
    c.footSpringLimitU = c.footSpringPreload + c.footRange;

    const double backPower_scale = 30.0;
    const double coxaPower_scale = 10.0;
    const double springstiffness = 350.0;

    // use an original radius and mass and scale original torque by their
    // new values to keep acceleration constant
    c.backPower = backPower_scale * (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass;
    // torque in Nm
    c.coxaPower = coxaPower_scale * (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass;
    c.secondPower = c.coxaPower;
    c.tebiaPower = c.coxaPower;
    // this is the spring constant. To keep  acceleration for the body
    // constant, we use the above unscaled preload of 0.08 and original
    // trunkMass to and then multiply by the new ones
    c.footPower = (springstiffness * 0.08 / 2.2) * c.trunkMass / c.footSpringPreload;

    c.backDamping = 0.0;
    // Georg: no damping required for new servos
    c.coxaDamping = 0.0;
    c.secondDamping = 0.0;
    c.tebiaDamping = 0.01;
    c.footDamping = 0.05; // a spring has no damping??

    //Increasing MaxVel by KOH to a factor of 1.7
    c.backMaxVel = 1.7 * 1.961 * M_PI;
    // The speed calculates how it works
    c.coxaMaxVel = 1.7 * 1.961 * M_PI;
    c.secondMaxVel = 1.7 * 1.961 * M_PI;
    c.tebiaMaxVel = 1.7 * 1.961 * M_PI;
    c.footMaxVel = 1.7 * 1.961 * M_PI;

    c.usRangeFront = 0.3 * c.size;
    c.irRangeLeg = 0.2 * c.size;

    //Values by Dennis
    // 1 is parallel, -1 is antiparallel
    c.usParallel = false;
    c.usAngleX = 0.5;
    c.usAngleY = 1;

    c.texture = "Images/whiteground.rgb";
    c.bodyTexture = "Images/stripes.rgb";

    //----------------Add GoalSensor by Ren------------------
    c.GoalSensor_references.clear(); //enforce empty vector -> no relative position sensing
    //----------------Add GoalSensor by Ren------------------

    return c;
  }

  AmosFourConf AmosFour::getAmosIIv1Conf(double _scale, bool _useShoulder, bool _useFoot, bool _useBack) {
    // Take basic configuration from amosiiv2
    // and then make necessary changes
    AmosFourConf c = getAmosIIv2Conf(_scale, _useShoulder, _useFoot, _useBack);

    // "Internal" variable storing the currently used version
    c.amos_version = 1;

    //trunk height
    c.height = /*6.5*/8.5 / 43.0 * c.size; //---------------------------------------------------AMOSIIv1
    // -----------------------
    // 1) Biomechanics
    // Manual setting adjustable joint positions at the body
    // -----------------------

    // amosII has a fixed but adjustable joint that decides how the legs
    // extend from the trunk. Here you can adjust these joints

    // ------------- Front legs -------------
    // angle (in rad) around vertical axis at leg-trunk fixation 0:
    // perpendicular
    // => forward/backward
    c.fLegTrunkAngleV = 0.0;
    // angle around horizontal axis at leg-trunk fixation 0: perpendicular
    // => upward/downward
    c.fLegTrunkAngleH = 3.1416 / 6; //---------------------------------------------------AMOSIIv1
    // rotation of leg around own axis 0: first joint axis is vertical
    // => till
    c.fLegRotAngle = 0.0;

    // ------------- Middle legs ----------------
    // => forward/backward
    c.mLegTrunkAngleV = 0.0;
    // => upward/downward
    c.mLegTrunkAngleH = 3.1416 / 6; //---------------------------------------------------AMOSIIv1
    // => till
    c.mLegRotAngle = 0.0;

    // ------------- Rear legs ------------------
    // => forward/backward
    c.rLegTrunkAngleV = 0.0;
    // => upward/downward
    c.rLegTrunkAngleH = 3.1416 / 6; //---------------------------------------------------AMOSIIv1
    // => till
    c.rLegRotAngle = 0.0;

    // -----------------------
    // 2) Joint Limits
    // Setting Max, Min of each joint with respect to real
    // -----------------------
    //
    //Similar to real robot
    //-45 deg; downward (+) MIN
    c.backJointLimitD = M_PI / 180 * 45.0; //---------------------------------------------------AMOSIIv1
    // 45 deg; upward (-) MAX
    c.backJointLimitU = -M_PI / 180 * 45.0;

    // 45 deg; forward (-) MAX --> normal walking range 25 deg MAX
    c.fcoxaJointLimitF = -M_PI / 180.0 * 45.0;
    //-45 deg; backward (+) MIN --> normal walking range -30 deg MIN
    c.fcoxaJointLimitB = M_PI / 180.0 * 45.0;

    //45 deg; forward (-) MAX --> normal walking range 25 deg MAX
    c.mcoxaJointLimitF = -M_PI / 180.0 * 45.0;
    //45 deg; backward (+) MIN --> normal walking range -30 deg MIN
    c.mcoxaJointLimitB = M_PI / 180 * 45.0;

    //45 deg; forward (-) MAX --> normal walking range 25 deg MAX
    c.rcoxaJointLimitF = -M_PI / 180.0 * 45.0;
    //45 deg; backward (+) MIN --> normal walking range -30 deg MIN
    c.rcoxaJointLimitB = M_PI / 180.0 * 45.0;

    // 30 deg; downward (+) MIN --> normal walking range 65 deg MIN
    c.secondJointLimitD = M_PI / 180.0 * 30.0;
    // 100 deg upward (-) MAX --> normal walking range 115 deg MAX
    c.secondJointLimitU = -M_PI / 180.0 * 100.0;

    //140 deg downward; (+) MIN --> normal walking range 140 deg MIN
    c.tebiaJointLimitD = M_PI / 180.0 * 140.0;
    //15 deg  downward; (+) MAX --> normal walking range 120 deg MAX
    c.tebiaJointLimitU = M_PI / 180.0 * 15.0;

    return c;
  }
}
