/*
 * hexabot_sensormotor_def.h
 *
 *  Created on: 27, Jan, 2014
 *      Author: Yuichi
 *
 *	This is the definition of the name of the motors and sensors for Hexabot
 */

#ifndef HEXABOTSENSORMOTORDEFINITION_H_
#define HEXABOTSENSORMOTORDEFINITION_H_

/***********************************************************************

 Here, I determine the parameters of the robot Hexabot

         /\
   leg 4 | |leg 1
   leg 5 | |leg 2
   leg 6 | |leg 3


 	  _	    Joint Joint Joint
   |      |    TC    CT    FT
   |      |== o === | === | ======  Leg
   |  _   | Shld Coxa  Femur  Tibia
     Body    Link  Link   Link

************************************************************************/

namespace HEXABOT{
// hexabot's sensor name
enum HexabotSensorNames{
  // Angle sensors
  // I numbered as figure of the top of this page (number 1 is top one)
  T1_as=0, //Thoracic joint 0
  T2_as=1, //Thoracic joint 1
  T3_as=2, //Thoracic joint 2
  T4_as=3, //Thoracic joint 0
  T5_as=4, //Thoracic joint 1
  T6_as=5, //Thoracic joint 2

  C1_as=6, //Coxa joint 0
  C2_as=7, //Coxa joint 1
  C3_as=8, //Coxa joint 2
  C4_as=9, //Coxa joint 0
  C5_as=10, //Coxa joint 1
  C6_as=11, //Coxa joint 2

  F1_as=12, //Fibula joint 0
  F2_as=13, //Fibula joint 1
  F3_as=14, //Fibula joint 2
  F4_as=15, //Fibula joint 0
  F5_as=16, //Fibula joint 1
  F6_as=17, //Fibula joint 2

  //Foot contact sensors
  L1_fs= 18, // foot 0
  L2_fs= 19, // foot 1
  L3_fs= 20, // foot 2
  L4_fs= 21, // foot 0
  L5_fs= 22, // foot 1
  L6_fs= 23, // foot 2

  // Torque sensors, used as Current sensors at each motor
  T1_ts=24,
  T2_ts=25,
  T3_ts=26,
  T4_ts=27,
  T5_ts=28,
  T6_ts=29,

  C1_ts=30,
  C2_ts=31,
  C3_ts=32,
  C4_ts=33,
  C5_ts=34,
  C6_ts=35,

  F1_ts=36,
  F2_ts=37,
  F3_ts=38,
  F4_ts=39,
  F5_ts=40,
  F6_ts=41,

  // Attitude sensor
  POSE_r = 42, // roll (rad)
  POSE_p = 43, // pitch
  POSE_y = 44, // yaw

  // angular Vel
  W_x = 45,// angular velocity
  W_y = 46,
  W_z = 47,

  // grobal position of the Robot Center
  GPOS_Rx = 48,
  GPOS_Ry = 49,
  GPOS_Rz = 50,

  // grobal speed of the Center
  GSPD_Rx = 51,
  GSPD_Ry = 52,
  GSPD_Rz = 53,

  // grobal position of the COG
  GPOS_COGx = 54,
  GPOS_COGy = 55,
  GPOS_COGz = 56,

  // grobal position of the LegToe
  GPOS_L1x = 57, // leg pos of Leg 0
  GPOS_L1y = 58,
  GPOS_L1z = 59,

  GPOS_L2x = 60,
  GPOS_L2y = 61,
  GPOS_L2z = 62,

  GPOS_L3x = 63,
  GPOS_L3y = 64,
  GPOS_L3z = 65,

  GPOS_L4x = 66, // leg pos of Leg 0
  GPOS_L4y = 67,
  GPOS_L4z = 68,

  GPOS_L5x = 69,
  GPOS_L5y = 70,
  GPOS_L5z = 71,

  GPOS_L6x = 72,
  GPOS_L6y = 73,
  GPOS_L6z = 74,


  //Changing according to the maximum sensor number
  HEXABOT_SENSOR_MAX = 75
};

// Hexabot's motor name
enum HexabotMotorNames{
  // base joint
  T1_m = 0,
  T2_m = 1,
  T3_m = 2,// Upward (+), Downward (-)
  T4_m = 3,
  T5_m = 4,
  T6_m = 5,// Upward (+), Downward (-)

  // 2nd joint
  C1_m = 6,
  C2_m = 7,
  C3_m = 8,// Upward (+), Downward (-)
  C4_m = 9,
  C5_m = 10,
  C6_m = 11,// Upward (+), Downward (-)

  // 3rd joint
  F1_m = 12,
  F2_m = 13,
  F3_m = 14, // Upward (+), Downward (-)
  F4_m = 15,
  F5_m = 16,
  F6_m = 17, // Upward (+), Downward (-)

  //Changing according to the maximum motor number
  HEXABOT_MOTOR_MAX = 18
};

/***********************************************************************

 Here, I determine the parameters of the robot Hexabot

 	  _	    Joint Joint Joint
   /     \    TC    CT    FT
  |       |== o === | === | ======  Leg
   \  _  / Shld Coxa  Femur  Tibia
 Hexagon Body    Link  Link   Link

************************************************************************/
// The internal parameters of the dynamixel
typedef struct {
  //Unit m, kg
  double length;
  double width;
  double height;

  double length_axis_to_center;// The length between the center of dynamixel and the axis.
  double length_from_axis_to_tip; // The length between the axis and the tip of joint material(bra for servo)

  double mass;
}DynaAX12Conf;

// The internal parameters of the Body
typedef struct {
  //Unit m, kg
  double length;
  double width;
  double height;
  double mass;
}BodyConf;

// The internal parameters of Foot frame (Tibia)
typedef struct {
  //Unit m,kg
  double length;
  double width;
  double height;
  double footRadius;

  double mass;
}FootFrameConf;

typedef struct {
	// Unit m
  double length_x_center_to_TCJ;
  double length_y_TCJ_to_TCJ;
  double length_TCJ_to_CTJ;
  double length_CTJ_to_FTJ;
  double length_FTJ_to_Toe;
}JointLength;

//The internal parameter of the servo
typedef struct{
  // angle limit (rad)
  // about TC
  double TC_angle_MAX;
  double TC_angle_MIN;
  // about CT
  double CT_angle_MAX;
  double CT_angle_MIN;
  // about FT
  double FT_angle_MAX;
  double FT_angle_MIN;

  //servoParam
  // input current??
  double power;
  // damping coefficience
  double damp;
  // integration
  double integ;
  // Max vel which the servo can get
  double maxVel;
}ServoParam;


//! >HEXABOT Configuration struct
typedef struct {
  double rate;// What rate should I multiple to real length number
  double massRate; // what rate should I multiple to real mass number
  double wholeMass; // the whole mass of the robot
  double x_trans_center; // offset between the center of the joints and the center of the body in x
  double y_trans_center; // offset between the center of the joints and the center of the body in y

  DynaAX12Conf dyna; // Dynamixel's real configuration
  BodyConf body; // Body's real config.
  FootFrameConf foot; // foot's real config.
  JointLength jLength; // joint length param
  ServoParam servoParam; // parameter for servo
}HexabotConf;

}

#endif //HEXABOT_SENSORMOTOR_DEF_H_
