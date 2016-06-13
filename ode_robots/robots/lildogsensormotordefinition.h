#ifndef LILDOGSENSORMOTORDEFINITION_H_
#define LILDOGSENSORMOTORDEFINITION_H_

enum LilDogSensorNames{

  // Angle sensors (for actoric-sensor board (new board)
//----------------FRONT LEGS ANGLE SENSORS----------
  S1R0_as=0, //shoulder joint 1 of right leg
  S1L0_as=6, //shoulder joint 1 of left leg

  S2R0_as=1, //shoulder joint 2 of right leg
  S2L0_as=7, //shoulder joint 2 of left leg

  ELR0_as=2, //elbow joint right leg
  ELL0_as=8, //elbow joint left leg


  
//----------------REAR LEGS ANGLE SENSORS-----------
  S1R1_as=3, //Hip joint 1 right leg
  S1L1_as=9, //Hip joint 1 left leg

  S2R1_as=4, //Hip joint 2 right leg
  S2L1_as=10,//Hip joint 2 left leg

  ELR1_as=5, //knee joint of right leg
  ELL1_as=11, //knee joint of left leg


 //--------Foot contact sensors----------------------

  R0_fs= 12, //Right front foot
  L0_fs= 14, //Left front foot
  R1_fs= 13, //Right hind foot
  L1_fs= 15, //Left hind foot

LILDOG_SENSOR_MAX = 16,

};

enum LilDogMotorNames{

//----------------FRONT LEGS MOTORS----------
  S1R0_m=0, //shoulder joint 1 of right leg
  S1L0_m=6, //shoulder joint 1 of left leg

  S2R0_m=1, //shoulder joint 2 of right leg
  S2L0_m=7, //shoulder joint 2 of left leg

  ELR0_m=2, //elbow joint right leg
  ELL0_m=8, //elbow joint left leg


  
//----------------REAR LEGS MOTORS-----------
  S1R1_m=3, //Hip joint 1 right leg
  S1L1_m=9, //Hip joint 1 left leg

  S2R1_m=4, //Hip joint 2 right leg
  S2L1_m=10,//Hip joint 2 left leg

  ELR1_m=5, //knee joint of right leg
  ELL1_m=11, //knee joint of left leg

  LILDOG_MOTOR_MAX = 12,

};

#endif
