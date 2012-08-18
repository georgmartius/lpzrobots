/***************************************************************************
 *   Copyright (C) 2012 by Robot Group Goettingen                          *
 *                                    									   *
 *    fhesse@physik3.gwdg.de     			                               *
 *    xiong@physik3.gwdg.de                  	                           *
 *    poramate@physik3.gwdg.de                                             *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   AMOSII v2 has now only 18 sensors                                     *
 ***************************************************************************/

#ifndef AMOSIISERIALV2_H_
#define AMOSIISERIALV2_H_

#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <iterator>


#include <assert.h>
#include <stdlib.h>

#include <selforg/abstractrobot.h>
#include <selforg/matrix.h>

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // terminal control definitions
#include <time.h>   // time calls

#include<sys/time.h>

//#include "sensor_motor_definition.h"
#include <ode_robots/amosiisensormotordefinition.h>

using namespace std;

#define COM1 "/dev/ttyS0"

#define SENSOR_BUFFER_NUM 34     //numbers for M-board 1
#define MOTOR_BUFFER_NUM 33      //numbers for M-board 1
#define COMMAND_BUFFER_NUM 100


namespace lpzrobots {


/** This class communicates with amosII robot
 */
class AmosIISerialV2 : public AbstractRobot {
public:
	AmosIISerialV2(const char *port = "/dev/ttyS0");


	~AmosIISerialV2();

	// robot interface
	/** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1]
      @param sensornumber length of the sensor array
      @return number of actually written sensors
	 */
	virtual int getSensors(sensor* sensors, int sensornumber);

	/** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
	 */
	virtual void setMotors(const motor* motors, int motornumber);


	/** returns number of sensors */
	virtual int getSensorNumber(){ return sensornumber; }

	/** returns number of motors */
	virtual int getMotorNumber() { return motornumber; }

	/* the following are not used here, you can ignore them but keep them*/
	virtual Position getPosition()     const {return Position(0,0,0);}
	virtual Position getSpeed()        const {return Position(0,0,0);}
	virtual Position getAngularSpeed() const {return Position(0,0,0);}
	virtual matrix::Matrix getOrientation() const {
		matrix::Matrix m(3,3);
		m.toId();
		return m;
	};

	/*Default sensors processing*/
	virtual void processSensors(sensor* pSensor);
	/*Your own sensors processing*/
	virtual void processSensorsKOH(sensor* pSensor);



private:

	int fd1;// Return char after opening COM0
	double  Sensor[AMOSII_SENSOR_MAX]; //but only 18 sensors used now, others are set to zero
	double motorCom[AMOSII_MOTOR_MAX];
	char chBuff;

	int servoPosMin[AMOSII_MOTOR_MAX];
	int servoPosMax[AMOSII_MOTOR_MAX];

	int nSensorType;
	double potValue[SENSOR_BUFFER_NUM];

	int index;//=0;
	double sensor1;//=1;
	int wr, rd;
	char comByte;//=2;
	char end;//=0;

	struct termios port_settings;// structure to store the port settings in
	struct termios tio;
	struct termios stdio;

	char serial_motor[100];
	int serialPos[MOTOR_BUFFER_NUM];

	int motornumber;
	int sensornumber;

	const char * port;

	paramval power;
	paramval tempThesh;
	paramval accelFactor;
	int t;


	enum RealAmosIISensorNames{

	 /*Add more sensors here according to the port of the MBoard*/


		// Foot contact sensors
		R0_fs_real= 13,
		R1_fs_real= 11,
		R2_fs_real= 9,
		L0_fs_real= 14,
		L1_fs_real= 12,
		L2_fs_real= 10,

		//US sensor at front
		FR_us_real= 1,
		FL_us_real= 2,

		//Reflex ir sensors at leg
		R0_irs_real=27,
		R1_irs_real=28,
		R2_irs_real=30,

		L0_irs_real=25,
		L1_irs_real=26,
		L2_irs_real=18,

		//light sensors
		M_ps_real=23,
		R_ps_real=15,
		L_ps_real=32,

		//Average current sensor of all motors [ZAP25]
		A_cs_real= 29,

		//Inclinometer sensor
		In_x_real=8,
		In_y_real=7,

		//Port 7 on the Mboard = Inclinometer y NOT yet install
		//Port 8 on the Mboard = Inclinometer x NOT yet install
		//Port 24 on the Mboard = free cable outside for any 5v sensor

  };

};

}
#endif /* AMOSIISERIALV2_H_ */
