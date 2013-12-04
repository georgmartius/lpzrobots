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

// include header file
#include "amosIIserialv1.h"




namespace lpzrobots {

AmosIISerialV1::AmosIISerialV1(const char *port)
: AbstractRobot("AmosIISerialV1", "$Id: main.cpp,v 0.1 2011/14/07 18:00:00 fhesse $"),
  port(port) {

	fd1=open(port, O_RDWR | O_NOCTTY | O_NDELAY);//make sure your account in PC can have access to serial port


	if(fd1 == -1)
	{
		std::cout<<std::endl<<"unable to open"<<port<<std::endl;
		std::cout<<"check provided port (hint: \"/dev/ttyS0\" or \"/dev/ttyUSB0\" required?)"<<std::endl;
		assert (fd1 != -1);
	}
	else
	{
		fcntl(fd1, F_SETFL, 0);
		printf("port is open");

		memset(&tio,0,sizeof(tio));
		tio.c_iflag=0;
		tio.c_oflag=0;
		tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
		tio.c_lflag=0;
		tio.c_cc[VMIN]=1;
		tio.c_cc[VTIME]=5;


		cfsetospeed(&tio,B57600);            // 57600 baud
		cfsetispeed(&tio,B57600);            // 57600 baud

		tcsetattr(fd1,TCSANOW,&tio);


		printf("port finish configuration \n");
	}
	comByte=2;
	end=0;
	index=0;
	sensor1=1;


	t=0;  // global step counter

	sensornumber = AMOSII_SENSOR_MAX;
	motornumber = AMOSII_MOTOR_MAX;

	//Setting Motors
	for (int t=0;t<33;t++)
	{
		serialPos[t]=128; //Setting all motor to middle position as initialization
	}

}


AmosIISerialV1::~AmosIISerialV1(){


	//Close COM0
	if(fd1 != -1){
		close(fd1);
		std::cout<<"closed the serial communication port"<<std::endl;
	}
}

// robot interface
/** returns actual sensorvalues
  @param sensors sensors scaled to [-1,1]
  @param sensornumber length of the sensor array
  @return number of actually written sensors
 */
int AmosIISerialV1::getSensors(sensor* sensors, int sensornumber){

	assert(sensornumber >= this->sensornumber);

	for(int i=0; i<=AMOSII_SENSOR_MAX;i++){
			sensors[i]=0;
	}

	comByte=2;
	end=0;

	char serial_msg[COMMAND_BUFFER_NUM];
	do{

		//Sending "getSensors" command to the board
		sprintf(serial_msg, "%c%c",comByte,end);

		wr = write(fd1, serial_msg,sizeof(serial_msg));

		// --- Reading the potentiometer values
		for (int i=1;i<SENSOR_BUFFER_NUM;i++){
			do{

				rd = read(fd1, &chBuff, 1);
				if (rd){
					potValue[i]=(unsigned char)(chBuff);// potvalue are AMOS sensor data
				}
			}while(!rd);

		}

	}while((unsigned char)(chBuff)!=0);// "0" is sync byte


	// LpzRobot <-- AMOS
	//Foot sensors (FS,Group 1)
	sensors[R0_fs]=potValue[R0_fs_real]; //[min = 7 (off ground), max =  207 (touch ground)]
	sensors[R1_fs]=potValue[R1_fs_real]; //[min = 15 (off ground), max = 196 (touch ground)]
	sensors[R2_fs]=potValue[R2_fs_real]; //[min = 20 (off ground), max = 200 (touch ground)]

	sensors[L0_fs]=potValue[L0_fs_real]; //[min = 28 (off ground), max = 196 (touch ground)]
	sensors[L1_fs]=potValue[L1_fs_real]; //[min = 27 (off ground), max = 195 (touch ground)]
	sensors[L2_fs]=potValue[L2_fs_real]; //[min = 20 (off ground), max = 200 (touch ground)]


	//US sensor (US, Group 2) // UNDK30U6112 range = 6 cm-40cm
	sensors[FR_us]=potValue[FR_us_real];//[min = 1 (detect object ~ 6 cm at front), max 135 (detect object at 40 cm),max 220 (detect object at 62 cm), max = 255 (not detect very close object)]
	sensors[FL_us]=potValue[FL_us_real];//[min = 1 (detect object ~ 6 cm at front), max 135 (detect object at 40 cm),max 220 (detect object at 62 cm), max = 255 (not detect very close object)]

	//Reflex ir sensors at leg (IRS, Group 3)
	sensors[R0_irs]=potValue[R0_irs_real];//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	sensors[R1_irs]=potValue[R1_irs_real];//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	sensors[R2_irs]=potValue[R2_irs_real];//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	sensors[L0_irs]=potValue[L0_irs_real];//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	sensors[L1_irs]=potValue[L1_irs_real];//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	sensors[L2_irs]=potValue[L2_irs_real];//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)

	//Photo sensors (PS, group 4)
	sensors[M_ps]=potValue[M_ps_real];//[min = 1 (very dark), max 250 (detect light (very bright))
	sensors[R_ps]=potValue[R_ps_real];//[min = 1 (very dark), max 250 (detect light (very bright))
	sensors[L_ps]=potValue[L_ps_real];//[min = 1 (very dark), max 250 (detect light (very bright))

	//Average Current sensor (ACS, group 5)
	sensors[A_cs]=potValue[A_cs_real];// [min = 15 (wave gait in the air), max 250]
	sensors[B_cs]=potValue[B_cs_real];// [min = 15 (wave gait in the air), max 250]


	//Average Inclinometer sensor (BX,Y, group 6)
	sensors[BX_acs]=potValue[BX_cs_real];
	sensors[BY_acs]=potValue[BY_cs_real];

	//Poti sensor (group 7)
	sensors[BZ_acs]=potValue[BZ_cs_real];

	// Ultrasonic reflex sensors at front, middle and rear legs (AMOSIIv1)
	sensors[R0_us]=potValue[R0_us_real];
	sensors[R1_us]=potValue[R1_us_real];
	sensors[L0_us]=potValue[L0_us_real];
	sensors[L1_us]=potValue[L1_us_real];


	//Conversion to positive range [0,..,255]
	for(int i=0; i<=AMOSII_SENSOR_MAX;i++){
		if (sensors[i] < 0){
			sensors[i]+=256;
		}
	}


	bool default_preprocessing = true;
	if (default_preprocessing){
		processSensors(sensors);
	}

	//Your own,e.g.,
	bool koh_preprocessing = false;
	if (koh_preprocessing){
	processSensorsKOH(sensors);
	}


	return this->sensornumber;

}

/*Different sensors processing*/
void AmosIISerialV1::processSensors(sensor* psensors){

	//Need to ADJUST again 12.04.2012 max min range
	//Foot sensor (FS, Group 1): Scaling to 0 (off ground),..,1 (on ground)
	psensors[R0_fs]= ((psensors[R0_fs]-7)/(207-7));   //[min = 7 (off ground), max =  207 (on ground)]
	psensors[R1_fs]= ((psensors[R1_fs]-15)/(196-15)); //[min = 15 (off ground), max = 196 (on ground)]
	psensors[R2_fs]= ((psensors[R2_fs]-20)/(200-20)); //[min = 20 (off ground), max = 200 (on ground)]
	psensors[L0_fs]= ((psensors[L0_fs]-28)/(196-28)); //[min = 28 (off ground), max = 196 (on ground)]
	psensors[L1_fs]= ((psensors[L1_fs]-27)/(195-27)); //[min = 27 (off ground), max = 195 (on ground)]
	psensors[L2_fs]= ((psensors[L2_fs]-20)/(200-20)); //[min = 20 (off ground), max = 200 (on ground)]

	if(psensors[R0_fs]>1)
		psensors[R0_fs] = 1;
	if(psensors[R0_fs]<0)
		psensors[R0_fs] = 0;

	if(psensors[R1_fs]>1)
		psensors[R1_fs] = 1;
	if(psensors[R1_fs]<0)
		psensors[R1_fs] = 0;

	if(psensors[R2_fs]>1)
		psensors[R2_fs] = 1;
	if(psensors[R2_fs]<0)
		psensors[R2_fs] = 0;

	if(psensors[L0_fs]>1)
		psensors[L0_fs] = 1;
	if(psensors[L0_fs]<0)
		psensors[L0_fs] = 0;

	if(psensors[L1_fs]>1)
		psensors[L1_fs] = 1;
	if(psensors[L1_fs]<0)
		psensors[L1_fs] = 0;

	if(psensors[L2_fs]>1)
		psensors[L2_fs] = 1;
	if(psensors[L2_fs]<0)
		psensors[L2_fs] = 0;

	//Need to ADJUST again 12.04.2012 max min range 180
	//US sensor (US, Group 2) // UNDK30U6112 range = 6 cm-30cm : Scaling to 0 (not detect),...,1 (detect obstacles)
	psensors[FR_us]=((psensors[FR_us]-255)/(1-255));//[min = 1 (detect object ~ 6 cm at front), max 180 (detect object at 30 cm),max 220 (detect object at 62 cm), max = 255 (not detect very close object)]
	psensors[FL_us]=((psensors[FL_us]-255)/(1-255));//[min = 1 (detect object ~ 6 cm at front), max 177 (detect object at 30 cm),max 220 (detect object at 62 cm), max = 255 (not detect very close object)]

	//Need to ADJUST again 12.04.2012 max min range
	//Reflex ir psensors at leg (IRS, Group 3) : Scaling to 0 (not detect),...,1 (detect obstacles)
	psensors[R0_irs]=((psensors[R0_irs]-1)/(80-1));//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	psensors[R1_irs]=((psensors[R1_irs]-1)/(80-1));//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	psensors[R2_irs]=((psensors[R2_irs]-1)/(80-1));//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	psensors[L0_irs]=((psensors[L0_irs]-1)/(80-1));//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	psensors[L1_irs]=((psensors[L1_irs]-1)/(80-1));//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)
	psensors[L2_irs]=((psensors[L2_irs]-1)/(80-1));//[min = 2 (not detect object), max 75 (detect object ~ 3mm at front)

	//Need to ADJUST again 12.04.2012 max min range
	//Photo psensors (PS, group 4) : Scaling to 0 (dark),..,1 (bright)
	psensors[M_ps]=((psensors[M_ps]-1)/(250-1));//[min = 1 (very dark), max 250 (detect light (very bright))
	psensors[R_ps]=((psensors[R_ps]-1)/(250-1));//[min = 1 (very dark), max 250 (detect light (very bright))
	psensors[L_ps]=((psensors[L_ps]-1)/(250-1));//[min = 1 (very dark), max 250 (detect light (very bright))

	//Need to ADJUST again 12.04.2012 max min range
	//Average Current sensor (ACS, group 5) : Scaling to 0 (low power),..,1 (high power)
	//psensors[A_cs]=((psensors[A_cs]-15)/(250-15));// [min = 15 (wave gait in the air), max 250]
	//wave gait ~38 with belly on ground (0.03)
	//tetrapod gait ~45 w with belly on ground (0.14)
	//tripod gait ~40 w with belly on ground (0.18)

}




/** sets actual motorcommands
  @param motors motors scaled to [-1,1]
  @param motornumber length of the motor array
 */
void AmosIISerialV1::setMotors(const motor* motors, int motornumber){

	assert(motornumber >= this->motornumber);


	end =0; // Null as Sync-byte
	comByte=1;


	// -------------------- initializing the Motor range ------------------------

	//[-45,.., 45 deg]
	//TR0_m
	servoPosMin[0] = 145;//120;
	servoPosMax[0] = 5;//25;
	//TR1_m
	servoPosMin[1] = 195;//160;
	servoPosMax[1] = 45;//80;
	//TR2_m
	servoPosMin[2] = 200;//170;
	servoPosMax[2] = 40;//80;

	//TL0_m
	servoPosMin[3] = 70;//105;
	servoPosMax[3] = 225;//195;
	//TL1_m
	servoPosMin[4] = 55;//90;
	servoPosMax[4] = 210;//170;
	//TL2_m
	servoPosMin[5] = 45;//95;
	servoPosMax[5] = 210;//180;

	//[-30,..,100 deg]
	double walkingheigh = 0.0;//50;
	//CR0_m
	servoPosMin[6] = 220;//55;
	servoPosMax[6] = 1;//1-walkingheigh;
	//CR1_m
	servoPosMin[7] = 250;//90;
	servoPosMax[7] = 30;//30+walkingheigh;
	//CR2_m
	servoPosMin[8] = 225;//70;
	servoPosMax[8] = 5;//5+walkingheigh;

	//CL0_m
	servoPosMin[9] = 15;//10;//15;	//**
	servoPosMax[9] = 225;//230;//245;//245+walkingheigh;
	//CL1_m
	servoPosMin[10] = 10;//180;
	servoPosMax[10] = 243;//243-walkingheigh;
	//CL2_m
	servoPosMin[11] = 10;//175;
	servoPosMax[11] = 230;//245;//230-walkingheigh;//**


	//[-140,...,-15 deg]
	double walkingheigh2 = 0.0;//10.0;//20;
	//FR0_m
	servoPosMin[12] = 25;//-140 deg
	servoPosMax[12] = 230;//60+walkingheigh2;
	//FR1_m
	servoPosMin[13] = 1;//-140 deg
	servoPosMax[13] = 210;//35-walkingheigh2;
	//FR2_m
	servoPosMin[14] = 5;//-140 deg
	servoPosMax[14] = 210;//35-walkingheigh2;

	//FL0_m
	servoPosMin[15] = 250;
	servoPosMax[15] = 30;//215-walkingheigh2;
	//FL1_m
	servoPosMin[16] = 230;
	servoPosMax[16] = 15;//200+walkingheigh2;
	//FL2_m
	servoPosMin[17] = 240;
	servoPosMax[17] = 20;//200+walkingheigh2;

	//M18 backbone joint
	servoPosMin[18] = 250;
	servoPosMax[18] = 1;

	// ##################### move motors ################
	for(int i=0;i<AMOSII_MOTOR_MAX;i++)
	{
		motorCom[i] = motors[i];// set LpzMotor value before processing and sending
		if (i<12 && i>5) {
			motorCom[i]-=0.1;
		}
		if (motorCom[i]>1) motorCom[i]=1;
		if (motorCom[i]<-1) motorCom[i]=-1;
	}



	//TR0
	serialPos[23] = (int) (double)(((motorCom[0]+1.0)/2.0)*(servoPosMax[0]-servoPosMin[0])+servoPosMin[0]) ;
	serialPos[11] = (int) (double)(((motorCom[1]+1.0)/2.0)*(servoPosMax[1]-servoPosMin[1])+servoPosMin[1]) ;
	serialPos[29] = (int) (double)(((motorCom[2]+1.0)/2.0)*(servoPosMax[2]-servoPosMin[2])+servoPosMin[2]) ;
	serialPos[21] = (int) (double)(((motorCom[3]+1.0)/2.0)*(servoPosMax[3]-servoPosMin[3])+servoPosMin[3]) ;
	serialPos[12] = (int) (double)(((motorCom[4]+1.0)/2.0)*(servoPosMax[4]-servoPosMin[4])+servoPosMin[4]) ;
	serialPos[31] = (int) (double)(((motorCom[5]+1.0)/2.0)*(servoPosMax[5]-servoPosMin[5])+servoPosMin[5]) ;

	//CR0
	serialPos[19] = (int) (double)(((motorCom[6]+1.0)/2.0)*(servoPosMax[6]-servoPosMin[6])+servoPosMin[6]) ;
	serialPos[10] = (int) (double)(((motorCom[7]+1.0)/2.0)*(servoPosMax[7]-servoPosMin[7])+servoPosMin[7]) ;
	serialPos[27] = (int) (double)(((motorCom[8]+1.0)/2.0)*(servoPosMax[8]-servoPosMin[8])+servoPosMin[8]) ;
	serialPos[20] = (int) (double)(((motorCom[9]+1.0)/2.0)*(servoPosMax[9]-servoPosMin[9])+servoPosMin[9]) ;
	serialPos[14] = (int) (double)(((motorCom[10]+1.0)/2.0)*(servoPosMax[10]-servoPosMin[10])+servoPosMin[10]) ;
	serialPos[32] = (int) (double)(((motorCom[11]+1.0)/2.0)*(servoPosMax[11]-servoPosMin[11])+servoPosMin[11]) ;

	//FR0
	serialPos[2] = (int) (double)(((motorCom[12]+1.0)/2.0)*(servoPosMax[12]-servoPosMin[12])+servoPosMin[12]) ;
	serialPos[9] = (int) (double)(((motorCom[13]+1.0)/2.0)*(servoPosMax[13]-servoPosMin[13])+servoPosMin[13]) ;
	serialPos[28] = (int) (double)(((motorCom[14]+1.0)/2.0)*(servoPosMax[14]-servoPosMin[14])+servoPosMin[14]) ;
	serialPos[24] = (int) (double)(((motorCom[15]+1.0)/2.0)*(servoPosMax[15]-servoPosMin[15])+servoPosMin[15]) ;
	serialPos[13] = (int) (double)(((motorCom[16]+1.0)/2.0)*(servoPosMax[16]-servoPosMin[16])+servoPosMin[16]) ;
	serialPos[30] = (int) (double)(((motorCom[17]+1.0)/2.0)*(servoPosMax[17]-servoPosMin[17])+servoPosMin[17]) ;

	// Backbone Joint
	//serialPos[1] = 120;
	serialPos[22] = (int) (double)(((motorCom[18]+1.0)/2.0)*(servoPosMax[18]-servoPosMin[18])+servoPosMin[18]) ;


	//usleep(1000);
	usleep (10000);//10000);
	// do some processing for motor commands before sending AMOS sensors

	sprintf(serial_motor, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
			,comByte,serialPos[1],serialPos[2],
			serialPos[3],serialPos[4],serialPos[5],serialPos[6],serialPos[7],serialPos[8],
			serialPos[9],serialPos[10],serialPos[11],serialPos[12],serialPos[13],serialPos[14],
			serialPos[15],serialPos[16],serialPos[17],serialPos[18],serialPos[19],serialPos[20],
			serialPos[21],serialPos[22],serialPos[23],serialPos[24],serialPos[25],serialPos[26],
			serialPos[27],serialPos[28],serialPos[29],serialPos[30],serialPos[31],serialPos[32],end);

	//Sendding command to serial port
	write(fd1, serial_motor, 34);//sizeof(serial_msg));

	// to slow down process a bit
	//usleep(10000);

	usleep (10000);//10000);

	// increase time counter
	t++;


}

/*Process your sensor signals here to match to your need*/
void AmosIISerialV1::processSensorsKOH(sensor* sensors){

}


}




