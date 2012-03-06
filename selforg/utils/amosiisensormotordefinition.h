/*
 * amosiisensormotordefinition.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 *
 *  Small adaptations and moved to oderobots/selforg/wirings
 *  	by fhesse
 */

#ifndef AMOSIISENSORMOTORDEFINITION_H_
#define AMOSIISENSORMOTORDEFINITION_H_

	enum AmosIISensorNames{

	  // Angle sensors
	  TR0_as=0, //Thoracic joint of right front leg
	  TR1_as=1, //Thoracic joint of right middle leg
	  TR2_as=2, //Thoracic joint of right hind leg

	  TL0_as=3, //Thoracic joint of left front leg
	  TL1_as=4, //Thoracic joint of left middle leg
	  TL2_as=5, //Thoracic joint of left hind leg

	  CR0_as=6, //Coxa joint of right front leg
	  CR1_as=7, //Coxa joint of right middle leg
	  CR2_as=8, //Coxa joint of right hind leg

	  CL0_as=9,  //Coxa joint of left hind leg
	  CL1_as=10, //Coxa joint of left hind leg
	  CL2_as=11, //Coxa joint of left hind leg

	  FR0_as=12, //Fibula joint of right front leg
	  FR1_as=13, //Fibula joint of right middle leg
	  FR2_as=14, //Fibula joint of right hind leg

	  FL0_as=15, //Fibula joint of left front leg
	  FL1_as=16, //Fibula joint of left middle leg
	  FL2_as=17, //Fibula joint of left hind leg

	  BJ_as= 18, //Backbone joint angle

	  //Foot contact sensors
	  R0_fs= 19, //Right front foot
	  R1_fs= 20, //Right middle foot
	  R2_fs= 21, //Right hind foot
	  L0_fs= 22, //Left front foot
	  L1_fs= 23, //Left middle foot
	  L2_fs= 24, //Left hind foot

	  FR_us=25, // US sensors = Front Ultrasonic sensors (right and left)
	  FL_us=26,

	  R0_irs=31, // IR sensors = IR sensors at legs
	  R1_irs=29,
	  R2_irs=27,
	  L0_irs=32,
	  L1_irs=30,
	  L2_irs=28,

	  R0_us= 33, // Reflex ultrasonic sensors at front, middle and rear legs
	  R1_us= 34,
	  L0_us= 35,
	  L1_us= 36,

	  TR0_ts=37, // Torque sensors, used as Current sensors at each motor
	  TR1_ts=38,
	  TR2_ts=39,

	  TL0_ts=40,
	  TL1_ts=41,
	  TL2_ts=42,

	  CR0_ts=43,
	  CR1_ts=44,
	  CR2_ts=45,

	  CL0_ts=46,
	  CL1_ts=47,
	  CL2_ts=48,

	  FR0_ts=49,
	  FR1_ts=50,
	  FR2_ts=51,

	  FL0_ts=52,
	  FL1_ts=53,
	  FL2_ts=54,

	  BJ_ts= 55,


	  BX_acs= 56, // 3D Accelerometer (x,y,z) at body
	  BY_acs= 57,
	  BZ_acs= 58,

	  L_ps = 59,  //photo (light) sensors Left, Middle and Right
	  M_ps = 60,
	  R_ps = 61,

	  G0x_s=62, // goal orientation sensors (relative position to reference object)
	  G0y_s=63,
	  G0z_s=64,

	  A_cs = 65, //average current sensor

	  BX_spd= 66, //Body speed sensors
	  BY_spd= 67,
	  BZ_spd= 68,



	  //---Adding more sensors**


//	  BX_acs= 56, // 3D Accelerometer (x,y,z) at body
//	  BY_acs= 57,
//	  //BZ_acs= 57,
//
//	  L_ps = 58,  //photo (light) sensors Left, Middle and Right
//	  M_ps = 59,
//	  R_ps = 60,
//
//	  //---Adding more sensors**
//
//	  G0x_s=61, // goal orientation sensors (relative position to reference object)
//	  G0y_s=62,
//	  G0z_s=62,
//
//	  A_cs = 63, //average current sensor




	  //Changing according to the maximum sensor number
	  AMOSII_SENSOR_MAX = 69,
  };



  enum AmosIIMotorNames{
	TR0_m = 0,
	TR1_m = 1,
	TR2_m = 2,// Upward (+), Downward (-)
	TL0_m = 3,
	TL1_m = 4,
	TL2_m = 5,

	CR0_m = 6,
	CR1_m = 7,
	CR2_m = 8,// Upward (+), Downward (-)
	CL0_m = 9,
	CL1_m = 10,
	CL2_m = 11,

	FR0_m = 12,
	FR1_m = 13,
	FR2_m = 14, // Upward (+), Downward (-)
	FL0_m = 15,
	FL1_m = 16,
	FL2_m = 17,

	BJ_m = 18,  // Upward (+), Downward (-)

	//Changing according to the maximum motor number
	AMOSII_MOTOR_MAX = 19,



  };

#endif /* AMOSIISENSORMOTORDEFINITION_H_ */
