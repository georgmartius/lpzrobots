/*
 * This file was created by Tobias Jahn on Tuesday May 01, 2012
 * 
 * This file provides an Bluetoothinterface to the EPuck. 
 * The embedded Programm can be found in "./epuck_embedded/"
 *
 * This file uses the SerialComm.h wich was created by Stefano Morgani on 11/18/08 as an part of "EPuckMonitor".
 * 
 */

#include <iostream>
#include <cmath>

#include "SerialComm.h"

#include "epuckbluetooth.h"

namespace lpzrobots{
  
  
  EPuckBluetooth::EPuckBluetooth(EPuckConf _conf){
    numOfMotor.LED0=numOfMotor.LED1=numOfMotor.LED2=numOfMotor.LED3=numOfMotor.LED4=numOfMotor.LED5=numOfMotor.LED6=numOfMotor.LED7=numOfMotor.LED_BODY=numOfMotor.LED_FRONT=numOfMotor.MOTOR_LEFT=numOfMotor.MOTOR_RIGHT=numOfMotor.SOUND=-1;
    numOfSensor.ACCX=numOfSensor.ACCY=numOfSensor.ACCZ=numOfSensor.IR0=numOfSensor.IR1=numOfSensor.IR2=numOfSensor.IR3=numOfSensor.IR4=numOfSensor.IR5=numOfSensor.IR6=numOfSensor.IR7=numOfSensor.AMBIENT_LIGHT0=numOfSensor.AMBIENT_LIGHT1=numOfSensor.AMBIENT_LIGHT2=numOfSensor.AMBIENT_LIGHT3=numOfSensor.AMBIENT_LIGHT4=numOfSensor.AMBIENT_LIGHT5=numOfSensor.AMBIENT_LIGHT6=numOfSensor.AMBIENT_LIGHT7=numOfSensor.GROUND0=numOfSensor.GROUND1=numOfSensor.GROUND2=numOfSensor.MIC0=numOfSensor.MIC1=numOfSensor.MIC2=numOfSensor.CAM=-1;
    
    connectionSpeedHz=0;
    abortThread=1;
    conf=_conf;
    motorCount=0;
    sensorCount=0;
    bytes=0;
    comm=NULL;
    arrUpdateMotor=NULL;
    arrMotor=NULL;
    arrSensor=NULL;

    memset(command, 0x0, 20);
    memset(version, 0x0, 100);
    memset(RxBuffer, 0x0, 45);
    
    init();
  } //end EPuckBluetooth()
  
  EPuckBluetooth::~EPuckBluetooth(){
    disconnectConnection();
    
    connectionSpeedHz=0;
    abortThread=1;
    motorCount=0;
    sensorCount=0;
    bytes=0;
    usleep(100000);
    
    if(comm!=NULL)delete comm; comm=NULL;
    if(arrUpdateMotor!=NULL)delete arrUpdateMotor; arrUpdateMotor=NULL;
    if(arrMotor!=NULL)delete arrMotor; arrMotor=NULL;
    if(arrSensor!=NULL)delete arrSensor; arrSensor=NULL;

    memset(command, 0x0, 20);
    memset(version, 0x0, 100);
    memset(RxBuffer, 0x0, 45);
  }
  
  void EPuckBluetooth::disconnectConnection(){
    for(unsigned int i=0; i<motorCount; i++){arrMotor[i]=0; arrUpdateMotor[i]=true;}
    usleep(1e6);
    abortThread=true;
    wait();
  }
  
  
  void EPuckBluetooth::init(){
    
    if(conf.SENSOR_STATE){
      numOfSensor.ACCX=sensorCount++;
      numOfSensor.ACCY=sensorCount++;
      numOfSensor.ACCZ=sensorCount++;
      numOfSensor.IR0=sensorCount++;
      numOfSensor.IR1=sensorCount++;
      numOfSensor.IR2=sensorCount++;
      numOfSensor.IR3=sensorCount++;
      numOfSensor.IR4=sensorCount++;
      numOfSensor.IR5=sensorCount++;
      numOfSensor.IR6=sensorCount++;
      numOfSensor.IR7=sensorCount++;
      numOfSensor.AMBIENT_LIGHT0=sensorCount++;
      numOfSensor.AMBIENT_LIGHT1=sensorCount++;
      numOfSensor.AMBIENT_LIGHT2=sensorCount++;
      numOfSensor.AMBIENT_LIGHT3=sensorCount++;
      numOfSensor.AMBIENT_LIGHT4=sensorCount++;
      numOfSensor.AMBIENT_LIGHT5=sensorCount++;
      numOfSensor.AMBIENT_LIGHT6=sensorCount++;
      numOfSensor.AMBIENT_LIGHT7=sensorCount++;
      numOfSensor.GROUND0=sensorCount++;
      numOfSensor.GROUND1=sensorCount++;
      numOfSensor.GROUND2=sensorCount++;
    }
    if(conf.MIC_STATE){
      numOfSensor.MIC0=sensorCount;
      sensorCount+=100;
      numOfSensor.MIC1=sensorCount;
      sensorCount+=100;
      numOfSensor.MIC2=sensorCount;
      sensorCount+=100;
    }
    if(conf.CAM_STATE){
      conf.CAM_TYPE?camPixNum = conf.CAM_WIDTH*conf.CAM_HEIGHT*3:camPixNum = conf.CAM_WIDTH*conf.CAM_HEIGHT;
      numOfSensor.CAM=sensorCount;
      sensorCount+=camPixNum;
    }
    
    
    if(1){//all motors are active
      numOfMotor.LED0=motorCount++;
      numOfMotor.LED1=motorCount++;
      numOfMotor.LED2=motorCount++;
      numOfMotor.LED3=motorCount++;
      numOfMotor.LED4=motorCount++;
      numOfMotor.LED5=motorCount++;
      numOfMotor.LED6=motorCount++;
      numOfMotor.LED7=motorCount++;
      numOfMotor.LED_BODY=motorCount++;
      numOfMotor.LED_FRONT=motorCount++;
      numOfMotor.MOTOR_LEFT=motorCount++;
      numOfMotor.MOTOR_RIGHT=motorCount++;
      numOfMotor.SOUND=motorCount++;
    }
    
    arrSensor = new int[sensorCount];
    arrMotor = new int[motorCount];
    arrUpdateMotor = new bool[motorCount];
    
    for(unsigned int i=0; i<motorCount;i++)arrMotor[i]=0;
    for(unsigned int i=0; i<motorCount;i++)arrUpdateMotor[i]=true; // at initialisation all values have to be updated/send to the epuck
    for(unsigned int i=0; i<sensorCount;i++)arrSensor[i]=0;
    
    initConnection(conf.port);
  } //end init()
  
  
  
  void EPuckBluetooth::initConnection(std::string port_string){
    char * port = new char[port_string.length()+1];
    strcpy(port,port_string.c_str());
    port[port_string.length()]=0;  
    
    
    int error = 0;
    comm=new SerialComm();
    error = comm->connect(port);
    if(error!=0){
      std::cerr << "Unable to open serial port \"" << port << "\"." << std::endl;
      return;
    }
    sprintf(command, "\r");
    comm->writeData(command, 1, 6000);        // clear output buffer
    comm->readData(version, 100, 200000);
    usleep(10000);
    
    sprintf(command, "V\r"); //get embedded softwareversion
    comm->writeData(command, 2, 2000);
    comm->readData(version, 100, 200000);
    usleep(10000);
    
    
    
    if(conf.CAM_STATE){//Update CameraParameters	
	memset(command, 0x0, 20);
	sprintf(command,"J,%d,%d,%d,%d\r", conf.CAM_TYPE, conf.CAM_WIDTH, conf.CAM_HEIGHT, conf.CAM_ZOOM);

	comm->writeData(command, strlen(command), 100000);
	comm->readData(RxBuffer, 3, 100000);    //response is j\r\n
	
	std::cout << RxBuffer << std::endl;
	usleep(20000);
    }//*/ //end update CameraParameters
    

    start();
  } //end initConnection()
  
  
  EPuckConf EPuckBluetooth::getDefaultConfig(){
    EPuckConf config;
    config.port = "/dev/rfcomm0";
    
    config.CAM_WIDTH=40;
    config.CAM_HEIGHT=40;
    config.CAM_TYPE=1;
    config.CAM_ZOOM=8;
    
    config.SENSOR_STATE=1;
    config.MIC_STATE=0;
    config.CAM_STATE=0;
    
    return config;
  } //end getDefaultConfig()
  
  int EPuckBluetooth::getRecommendConnectionSpeed(EPuckConf config){
    int pixNum=0;
    config.CAM_TYPE?pixNum = config.CAM_WIDTH*config.CAM_HEIGHT*3:pixNum = config.CAM_WIDTH*config.CAM_HEIGHT;
    
    if(config.SENSOR_STATE && config.CAM_STATE && config.MIC_STATE) return 2;
    if(config.SENSOR_STATE && !config.CAM_STATE && config.MIC_STATE) return 8;
    if(config.SENSOR_STATE && !config.CAM_STATE && !config.MIC_STATE) return 25;
    if(!config.SENSOR_STATE && !config.CAM_STATE && !config.MIC_STATE) return 40;
    
  }
  
  void EPuckBluetooth::getNumOfMotSens(SensorNumbers& sens, int& _sensorCount, MotorNumbers& mot,int& _motorCount){
    sens=numOfSensor;
    _sensorCount=sensorCount;
    mot=numOfMotor;
    _motorCount=motorCount;
  }
  
  
  void EPuckBluetooth::receiveSensors(){
    memset(command, 0x0, 20);
    memset(RxBuffer, 0x0, 45);
    
    if(conf.SENSOR_STATE){
      int length=0;
      command[length++]=-'A'; //acc length of answer 12 char
      command[length++]=-'N';    //proximity lenght of answer 16 char
      command[length++]=-'M';    //ground	lenght of answer 6 char
      command[length++]=-'O';    //ambient light lenght of answer 16 char
      command[length++]=0;
      bytes = comm->writeData(command,length,10000);
      
      
      //read ACC
      bytes=comm->readData((char*)RxBuffer,12,1000000);
      if(bytes<12){
	sprintf(msg , "IRs: only %d bytes red", bytes);
	std::cerr << msg << std::endl;
      }
      else{
	long  mantis=0;short  exp=0;float flt=0;
	mantis = (RxBuffer[0] & 0xff) + ((RxBuffer[1] & 0xffl) << 8) + (((RxBuffer[2] &0x7fl) | 0x80) << 16);
	exp = (RxBuffer[3] & 0x7f) * 2 + ((RxBuffer[2] & 0x80) ? 1 : 0);
	if (RxBuffer[3] & 0x80) mantis = -mantis;
	flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
	arrSensor[numOfSensor.ACCX]=flt;
	
	mantis = (RxBuffer[4] & 0xff) + ((RxBuffer[5] & 0xffl) << 8) + (((RxBuffer[6] &0x7fl) | 0x80) << 16);
	exp = (RxBuffer[7] & 0x7f) * 2 + ((RxBuffer[6] & 0x80) ? 1 : 0);
	if (RxBuffer[7] & 0x80) mantis = -mantis;
	flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
	arrSensor[numOfSensor.ACCY]=flt;
	
	mantis = (RxBuffer[8] & 0xff) + ((RxBuffer[9] & 0xffl) << 8) + (((RxBuffer[10] &0x7fl) | 0x80) << 16);
	exp = (RxBuffer[11] & 0x7f) * 2 + ((RxBuffer[10] & 0x80) ? 1 : 0);
	if (RxBuffer[11] & 0x80) mantis = -mantis;
	flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
	arrSensor[numOfSensor.ACCZ]=flt;
      }//*/
      
      /*
       *    //read ACC unfiltered ("a" instead of "A")
       *    bytes=comm->readData((char*)RxBuffer,6,1000000);
       *    if(bytes<6){
       *      sprintf(msg , "ACC: only %d bytes red", bytes);
       *      std::cerr << msg << std::endl;
    }
    else{
      for(int i=0; i<3; i++)arrSensor[numOfSensor.ACCX+i]=4000*(RxBuffer[0+2*i]+RxBuffer[1+2*i]*256);
    }//*/
    
    
    //read PROX
    bytes=comm->readData((char*)RxBuffer,16,1000000);
    if(bytes<16){
      sprintf(msg , "IRs: only %d bytes red", bytes);
      std::cerr << msg << std::endl;
    }
    else{
      for(int i=0; i<8; i++)arrSensor[numOfSensor.IR0+i]=(RxBuffer[0+2*i]+RxBuffer[1+2*i]*256>2000)?2000:RxBuffer[0+2*i]+RxBuffer[1+2*i]*256;
    }//*/
    
    
    //read GROUND
    bytes=comm->readData((char*)RxBuffer,6,1000000);
    if(bytes<6){
      sprintf(msg , "IRs: only %d bytes red", bytes);
      std::cerr << msg << std::endl;
    }
    else{
      for(int i=0; i<3; i++)arrSensor[numOfSensor.GROUND0+i]=(RxBuffer[0+2*i]+RxBuffer[1+2*i]*256>2000)?2000:RxBuffer[0+2*i]+RxBuffer[1+2*i]*256;
    }//*/
    
    
    //read AMBIENT_LIGHT
    bytes=comm->readData((char*)RxBuffer,16,1000000);
    if(bytes<16){
      sprintf(msg , "IRs: only %d bytes red", bytes);
      std::cerr << msg << std::endl;
    }
    else{
      for(int i=0; i<8;i++)arrSensor[numOfSensor.AMBIENT_LIGHT0+i]= (RxBuffer[2*i]+RxBuffer[2*i+1]*256);
    }//*/
    }//end if(conf.SENSOR_STATE)
    
  }//end receiveSensors
  
  
  
  void EPuckBluetooth::receiveMics(){
    if(conf.MIC_STATE){
      command[0]=-'U';    //binary micro buffer receiving command
      command[1]=0;       //end command
      //send command
      
      bytes = comm->writeData(command,2,6000);
      

      //read micro header
      bytes = comm->readData((char*)micCharBuffer,600,100000);
      //Little Endian
      if(bytes<600){
          sprintf(msg , "IRs: only %d bytes red", bytes);
          std::cerr << msg << std::endl;
      }
      else{
          for(int i=0; i<300; i++) arrSensor[numOfSensor.MIC0+i] = 0;
          for(int i=0,j=0; i<300&&j<600; i++){
            arrSensor[numOfSensor.MIC0+i] = (unsigned char)micCharBuffer[j] | ((char)micCharBuffer[j+1]<<8);
            j+=2;
          }
      }

      bytes = comm->readData((char*)RxBuffer,1,10000);
    }//end if(conf.MIC_STATE)
  }//end of receiveMics()
  
  
  
  
  void EPuckBluetooth::receiveCam(){
    if(conf.CAM_STATE){
        /*
        std::cout << "Type" << conf.CAM_TYPE << std::endl;
        std::cout << "PixNum" << camPixNum << std::endl;
        std::cout << "Height" << conf.CAM_HEIGHT << std::endl;
        std::cout << "Width" << conf.CAM_WIDTH << std::endl;
        //*/

      command[0]=-'I';      //binary image receiving command
      command[1]= 0;        //end command
      //send command
      bytes = comm->writeData(command,2,6000);
      bytes = comm->readData((char*)camCharBuffer, camPixNum+3, 10000000);
      if(bytes<camPixNum+3){
	sprintf(msg , "CAM: only %d bytes red", bytes);
	std::cerr << msg << std::endl;
      }
      else{
        for(int i=0; i<camPixNum;i++){
	  arrSensor[numOfSensor.CAM+i]=camCharBuffer[i];
	}
      }
    }//end if(conf.CAM_STATE)
  }//end of receiveCam()
  
  
  
  int EPuckBluetooth::getSensors(sensor* sensors, int _sensorCount){
    
    //    std::cout << "Sensors=" << sensorCount << " Motors=" << motorCount << std::endl;
    //    std::cout << "numofCAM=" << numOfSensor.CAM << "\npixnum=" << camPixNum << std::endl;
    //    std::cout << " " << numOfSensor.ACCX << "\n" << numOfSensor.IR0 << std::endl;
    
    for(int i=0; i<(int)sensorCount;i++){
      if(conf.SENSOR_STATE){
	if( i>=numOfSensor.IR0&&i<=numOfSensor.IR7 )	  sensors[i]=(double)arrSensor[i]/2000.;
	if( i>=numOfSensor.GROUND0&&i<=numOfSensor.GROUND2 )	  sensors[i]=(double)arrSensor[i]/2000.;
	if( i>=numOfSensor.AMBIENT_LIGHT0&&i<=numOfSensor.AMBIENT_LIGHT7 )	  sensors[i]=1-(double)arrSensor[i]/4000.;
	if( i>=numOfSensor.ACCX&&i<=numOfSensor.ACCZ )	  sensors[i]=(double)arrSensor[i]/4000.;
	
	if(sensors[i]>1)sensors[i]=1;
	if(sensors[i]<0)sensors[i]=0;
      }
    }
      
      if(conf.MIC_STATE){
          double medMic0=0, medMic1=0, medMic2=0;
          for(int i=0; i<100; i++){
              medMic0+=(double)arrSensor[numOfSensor.MIC0+i]/100.;
              medMic1+=(double)arrSensor[numOfSensor.MIC1+i]/100.;
              medMic2+=(double)arrSensor[numOfSensor.MIC2+i]/100.;
          }

          double* tmpMic[3];
          tmpMic[0]= &sensors[numOfSensor.MIC0];
          tmpMic[1]= &sensors[numOfSensor.MIC1];
          tmpMic[2]= &sensors[numOfSensor.MIC2];

          for(int id=e_last_mic_scan_id, i=0; i<100; id=(e_last_mic_scan_id+i)%100, ++i){
              tmpMic[0][i] = arrSensor[numOfSensor.MIC0+i] - medMic0;
              tmpMic[1][i] = arrSensor[numOfSensor.MIC1+i] - medMic1;
              tmpMic[2][i] = arrSensor[numOfSensor.MIC2+i] - medMic2;
          }
      }

      if(conf.CAM_STATE)for( int i=numOfSensor.CAM; i<numOfSensor.CAM+camPixNum;i++){
          sensors[i]=arrSensor[i];
      }
      return sensorCount;
    }//end getSensors



    void EPuckBluetooth::setMotors(const motor* motors, int _motorCount){

      //set LEDs
      for(unsigned int i=0; i<10; i++){
        if( (int)motors[numOfMotor.LED0+i]!=arrMotor[numOfMotor.LED0+i] ) arrUpdateMotor[numOfMotor.LED0+i]=true;
        arrMotor[numOfMotor.LED0+i]=motors[numOfMotor.LED0+i];
      }

      double speed_max=1000;
      if( (int) (speed_max*motors[numOfMotor.MOTOR_LEFT ]) != arrMotor[numOfMotor.MOTOR_LEFT] ){
        arrUpdateMotor[numOfMotor.MOTOR_LEFT]=true;
        arrMotor[numOfMotor.MOTOR_LEFT] =(int) (speed_max*motors[numOfMotor.MOTOR_LEFT ]);
      }
      if( (int) (speed_max*motors[numOfMotor.MOTOR_RIGHT]) != arrMotor[numOfMotor.MOTOR_RIGHT]){
        arrUpdateMotor[numOfMotor.MOTOR_RIGHT]=true;
        arrMotor[numOfMotor.MOTOR_RIGHT]=(int) (speed_max*motors[numOfMotor.MOTOR_RIGHT ]);
      }

      if( arrMotor[numOfMotor.SOUND] != (int) motors[numOfMotor.SOUND] ) arrUpdateMotor[numOfMotor.SOUND]=1;
      arrMotor[numOfMotor.SOUND]=motors[numOfMotor.SOUND];
    }//end setMotors



    void EPuckBluetooth::transmitMotors(){
      memset(command, 0x0, 20);

      //Set Motors
      if(arrUpdateMotor[numOfMotor.MOTOR_LEFT]||arrUpdateMotor[numOfMotor.MOTOR_RIGHT]){
        arrUpdateMotor[numOfMotor.MOTOR_LEFT]=arrUpdateMotor[numOfMotor.MOTOR_RIGHT]=false; //Update reset
        char high_left = (arrMotor[numOfMotor.MOTOR_LEFT]>>8) & 0xFF;
        char low_left = arrMotor[numOfMotor.MOTOR_LEFT] & 0xFF;
        char high_right = (arrMotor[numOfMotor.MOTOR_RIGHT]>>8) & 0xFF;
        char low_right = arrMotor[numOfMotor.MOTOR_RIGHT] & 0xFF;

        sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
        bytes = comm->writeData(command, 6, 20000);
        usleep(20000);
      }


      /**Set LED*/
      if(arrUpdateMotor[numOfMotor.LED0]){
        arrUpdateMotor[numOfMotor.LED0]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED0] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 0, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 0, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }
      if(arrUpdateMotor[numOfMotor.LED1]){
        arrUpdateMotor[numOfMotor.LED1]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED1] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 1, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 1, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }
      if(arrUpdateMotor[numOfMotor.LED2]){
        arrUpdateMotor[numOfMotor.LED2]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED2] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 2, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 2, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }
      if(arrUpdateMotor[numOfMotor.LED3]){
        arrUpdateMotor[numOfMotor.LED3]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED3] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 3, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 3, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }
      if(arrUpdateMotor[numOfMotor.LED4]){
        arrUpdateMotor[numOfMotor.LED4]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED4] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 4, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 4, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }
      if(arrUpdateMotor[numOfMotor.LED5]){
        arrUpdateMotor[numOfMotor.LED5]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED5] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 5, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 5, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }
      if(arrUpdateMotor[numOfMotor.LED6]){
        arrUpdateMotor[numOfMotor.LED6]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED6] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 6, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 6, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }
      if(arrUpdateMotor[numOfMotor.LED7]){
        arrUpdateMotor[numOfMotor.LED7]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED7] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 7, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 7, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }
      if(arrUpdateMotor[numOfMotor.LED_FRONT]){
        arrUpdateMotor[numOfMotor.LED_FRONT]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED_FRONT] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 9, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 9, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }
      if(arrUpdateMotor[numOfMotor.LED_BODY]){
        arrUpdateMotor[numOfMotor.LED_BODY]=false; //Update reset
        memset(command, 0x0, 20);
        if(arrMotor[numOfMotor.LED_BODY] == 0) {
          sprintf(command, "%c%c%c%c",-'L', 8, 0, 0);
        } else {
          sprintf(command, "%c%c%c%c",-'L', 8, 1, 0);
        }
        comm->writeData(command, 4, 20000);
        usleep(20000);
      }

      //set SOUND
      if(arrUpdateMotor[numOfMotor.SOUND]){
        arrUpdateMotor[numOfMotor.SOUND]=false;
        sprintf(command, "T,%c\r", (char)(arrMotor[numOfMotor.SOUND]+48));
        arrMotor[numOfMotor.SOUND]=0;

        comm->writeData(command,4,12000);
        comm->readData(RxBuffer,3,20000);
        usleep(20000);
      }

    }//end transmitMotors()

    void EPuckBluetooth::run(){
      abortThread=0;
      QTime t;
      while(!abortThread){
        t.restart();
        usleep(1000);

        transmitMotors();
        receiveSensors();
        receiveMics();
        receiveCam();

        if(!connectionSpeedHz)connectionSpeedHz=1000./t.elapsed();
        connectionSpeedHz*=0.9;
        connectionSpeedHz += 0.1*1000./t.elapsed();
      }
    }//end run()


};
//namespace lpzrobots
