/*
 * amosiisensormotordefinition.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 *
 */

#ifndef AMOSIISENSORMOTORDEFINITION_H_
#define AMOSIISENSORMOTORDEFINITION_H_

enum AmosIISensorNames{

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

        // Ultrasonic reflex sensors at front, middle and rear legs (AMOSIIv1)
        R0_us= 33,
        R1_us= 34,
        L0_us= 35,
        L1_us= 36,

        // Torque sensors, used as Current sensors at each motor (for actoric-sensor board (new board))
        TR0_ts=37,
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

        // 3D Accelerometer (x,y,z) at body (for actoric-sensor board (new board))
        BX_acs= 56,
        BY_acs= 57,
        BZ_acs= 58,

        //photo (light) sensors Left, Middle and Right (AMOSIIv1 and v2)
        L_ps = 59,
        M_ps = 60,
        R_ps = 61,

        // goal orientation sensors (relative position to reference object 1, e.g. camera)
        G0x_s=62,
        G0y_s=63,
        G0z_s=64,

        //average current sensor of motors (ZAP 25)
        A_cs = 65,  //average motor current measurement (amosiiv1 & amosiiv2)
        B_cs = 121, //direct from Battery of only for amosiiv1

        //Body speed sensors (only simulation)
        BX_spd= 66,
        BY_spd= 67,
        BZ_spd= 68,

        //---Adding more sensors**

        // goal orientation sensors (relative angle to reference object 1, e.g. camera)
        G0angleroll_s=69,
        G0anglepitch_s=70,
        G0angleyaw_s=71,

        // goal orientation sensors (relative position to reference object 2, e.g. camera)
        G1x_s=72,
        G1y_s=73,
        G1z_s=74,

        // goal orientation sensors (relative angle to reference object 2, e.g. camera)
        G1angleroll_s=75,
        G1anglepitch_s=76,
        G1angleyaw_s=77,

        // goal orientation sensors (relative position to reference object 3, e.g. camera)
        G2x_s=78,
        G2y_s=79,
        G2z_s=80,

        // goal orientation sensors (relative angle to reference object 3, e.g. camera)
        G2angleroll_s=81,
        G2anglepitch_s=82,
        G2angleyaw_s=83,

        //laser scanner (number of edges found, average height of data points, roughness criterion and minimum/maximum height
        // AMOSIIv2
        LaserNmbEdge_s = 84,
        LaserHeight_s = 85,
        LaserRough_s = 86,
        LaserMaxHeight_s = 87,
        LaserMinHeight_s = 88,

        // AMOSIIv1
        Poti_s = 89,

        //Compass sensors at body (for actoric-sensor board (new board))
        Compassx_s = 90,
        Compassy_s = 91,

        // 3D Accelerometer (x,y,z) at RO
        // on Foot sensor board, AMOSIIv1 and v2
        R0X_acs= 92,
        R0Y_acs= 93,
        R0Z_acs= 94,

        // 3D Accelerometer (x,y,z) at R1
        // on Foot sensor board, AMOSIIv1 and v2
        R1X_acs= 95,
        R1Y_acs= 96,
        R1Z_acs= 97,

        // 3D Accelerometer (x,y,z) at R2
        // on Foot sensor board, AMOSIIv1 and v2
        R2X_acs= 98,
        R2Y_acs= 99,
        R2Z_acs= 100,

        // 3D Accelerometer (x,y,z) at L0
        // on Foot sensor board, AMOSIIv1 and v2
        L0X_acs= 101,
        L0Y_acs= 102,
        L0Z_acs= 103,

        // 3D Accelerometer (x,y,z) at L1
        // on Foot sensor board, AMOSIIv1 and v2
        L1X_acs= 104,
        L1Y_acs= 105,
        L1Z_acs= 106,

        // 3D Accelerometer (x,y,z) at L2
        // on Foot sensor board, AMOSIIv1 and v2
        L2X_acs= 107,
        L2Y_acs= 108,
        L2Z_acs= 109,

        //Microphone sensors (for actoric-sensor board (new board))
        Microphone0_s= 110, // Microphone 0
        Microphone1_s= 111, // Microphone 1
        Microphone2_s= 112, // Microphone 2

        // Inclinometer sensors of the AMOSIIv1 and v2 body
        In_x = 113, //around x axis (forward walking direction)
        In_y = 114, //around y axis (sideward walking direction)

        //Body position sensors (only simulation)
        BX_pos = 115, //(forward walking direction)
        BY_pos = 116, //(sideward walking direction)
        BZ_pos = 117, //(vertical direction)

        //Body orientation sensors (only simulation), comparable to compass of the real robot
        BX_ori = 118, // around x axis
        BY_ori = 119, // around y axis
        BZ_ori = 120, // around z axis

        //Changing according to the maximum sensor number
        AMOSII_SENSOR_MAX = 122,

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
