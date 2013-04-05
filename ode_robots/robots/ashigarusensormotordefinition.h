/*
 * ashigaru_sensormotor_def.h
 *
 *  Created on: 18, Sep, 2012
 *      Author: Yuichi
 *
 *        This is the definition of the name of the motors and sensors for Ashigaru
 */

#ifndef ASHIGARUSENSORMOTORDEFINITION_H_
#define ASHIGARUSENSORMOTORDEFINITION_H_

/***********************************************************************

 Here, I determine the parameters of the robot Ashigaru

           _            Joint Joint Joint
   /     \    TC    CT    FT
  |       |== o === | === | ======  Leg
   \  _  / Shld Coxa  Femur  Tibia
 Hexagon Body    Link  Link   Link

************************************************************************/

namespace ASHIGARU{
// ashigaru's sensor name
enum AshigaruSensorNames{
  // Angle sensors
    // I numbered the joint in clockwise (number 0 is top one)
  T0_as=0, //Thoracic joint 0
  T1_as=1, //Thoracic joint 1
  T2_as=2, //Thoracic joint 2

  C0_as=3, //Coxa joint 0
  C1_as=4, //Coxa joint 1
  C2_as=5, //Coxa joint 2

  F0_as=6, //Fibula joint 0
  F1_as=7, //Fibula joint 1
  F2_as=8, //Fibula joint 2

  //Foot contact sensors
  L0_fs= 9, // foot 0
  L1_fs= 10, // foot 1
  L2_fs= 11, // foot 2

  // Torque sensors, used as Current sensors at each motor
  T0_ts=12,
  T1_ts=13,
  T2_ts=14,

  C0_ts=15,
  C1_ts=16,
  C2_ts=17,

  F0_ts=18,
  F1_ts=19,
  F2_ts=20,

  // Attitude sensor
  //
  //      ----
  //    / 2     1
  //   [    center               z ---------> y
  //   [                           |
  //    \ ___0__                   |
  ///                                  V x
  //
  POSE_r = 21, // roll (rad)
  POSE_p = 22, // pitch
  POSE_y = 23, // yaw

  // angular Vel
  W_x = 24,// angular velocity
  W_y = 25,
  W_z = 26,

  // grobal position of the Robot Center
  GPOS_Rx = 27,
  GPOS_Ry = 28,
  GPOS_Rz = 29,

  // grobal speed of the Center
  GSPD_Rx = 30,
  GSPD_Ry = 31,
  GSPD_Rz = 32,

  // grobal position of the COG
  GPOS_COGx = 33,
  GPOS_COGy = 34,
  GPOS_COGz = 35,

  // grobal position of the LegToe
  GPOS_L0x = 36, // leg pos of Leg 0
  GPOS_L0y = 37,
  GPOS_L0z = 38,

  GPOS_L1x = 39,
  GPOS_L1y = 40,
  GPOS_L1z = 41,

  GPOS_L2x = 42,
  GPOS_L2y = 43,
  GPOS_L2z = 44,

  //Changing according to the maximum sensor number
  ASHIGARU_SENSOR_MAX = 45,

};

// Ashigaru's motor name
enum AshigaruMotorNames{
    // base joint
    T0_m = 0,
    T1_m = 1,
    T2_m = 2,// Upward (+), Downward (-)

    // 2nd joint
    C0_m = 3,
    C1_m = 4,
    C2_m = 5,// Upward (+), Downward (-)

    // 3rd joint
    F0_m = 6,
    F1_m = 7,
    F2_m = 8, // Upward (+), Downward (-)

    //Changing according to the maximum motor number
    ASHIGARU_MOTOR_MAX = 9,
};

/***********************************************************************

 Here, I determine the parameters of the robot Ashigaru

           _            Joint Joint Joint
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

// The internal parameters of the Hexagon Body
typedef struct {
        //Unit m, kg
        double length;
        double height;

        double mass;
}HexagonBodyConf;

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
        double length_center_to_TCJ;
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
         // Max vel which the servo can get
        double maxVel;

}ServoParam;

//The special parameter
//  this is not good parameter I think, I have to change it But, temporally I use it

typedef struct{
        // The leg which is used for connection
        int conectedLegNum;
        // The strength of the joint connection
        //  this is correspond to the servo parameter P
        //  By making this parameter too large to vibrate, we can treat it as rigid body
        double servoPower;

}SpecialParam;

//! >ASHIGARU Configuration struct
typedef struct {
        double rate;// What rate should I multiple to real length number
        double massRate; // what rate should I multiple to real mass number
        double connectLength; // Length between center of the robot and that of another robot when they are connected each other.
        double wholeMass; // the whole mass of the robot

        DynaAX12Conf dyna; // Dynamixel's real configuration
        HexagonBodyConf body; // Body's real config.
        FootFrameConf foot; // foot's real config.
        JointLength jLength; // joint length param
        ServoParam servoParam; // parameter for servo
        SpecialParam specialParam; // special parameter (temporally)

} AshigaruConf;

}

#endif //ASHIGARU_SENSORMOTOR_DEF_H_
