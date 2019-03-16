#ifndef DUNGBEETLESENSORMOTORDEFINITION_H_
#define DUNGBEETLESENSORMOTORDEFINITION_H_

enum DungBeetleSensorNames{

  // Angle sensors (for actoric-sensor board (new board))
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

  //Foot contact sensors (AMOSII v1 and v2)
  R0_fs= 19, //Right front foot
  R1_fs= 20, //Right middle foot
  R2_fs= 21, //Right hind foot
  L0_fs= 22, //Left front foot
  L1_fs= 23, //Left middle foot
  L2_fs= 24, //Left hind foot

  // US sensors (AMOSII v1 and v2)
  FR_us=25, //Front Ultrasonic sensor (right)
  FL_us=26, //Front Ultrasonic sensor (left)

  // IR reflex sensors at legs (AMOSIIv2)
  R0_irs=31,
  R1_irs=29,
  R2_irs=27,
  L0_irs=32,
  L1_irs=30,
  L2_irs=28,

  R0_s1=33,
  R0_s2=34,
  R0_s3=35,
  R0_s4=36,
  R0_s5=37,

  L0_s1=38,
  L0_s2=39,
  L0_s3=40,
  L0_s4=41,
  L0_s5=42,

  R1_s1=43,//middle right
  R1_s2=44,
  R1_s3=45,
  R1_s4=46,
  R1_s5=47,

  L1_s1=48,//middle left
  L1_s2=49,
  L1_s3=50,
  L1_s4=51,
  L1_s5=52,

  R2_s1=53,
  R2_s2=54,
  R2_s3=55,
  R2_s4=56,
  R2_s5=57,

  L2_s1=58,
  L2_s2=59,
  L2_s3=60,
  L2_s4=61,
  L2_s5=62,




  DUNGBEETLE_SENSOR_MAX = 63,

};



enum DungBeetleMotorNames{
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
  DUNGBEETLE_MOTOR_MAX = 19,



};

#endif
