/*
 * This file was created by Tobias Jahn on Tuesday May 01, 2012
 * 
 * This file provides an Bluetoothinterface to the EPuck. 
 * The embedded Programm can be found in "./epuck_embedded/"
 *
 * This file uses the SerialComm.h wich was created by Stefano Morgani on 11/18/08 as an part of "EPuckMonitor".
 * 
 */

#ifndef EPUCKBLUETOOTH
#define EPUCKBLUETOOTH

#include <QThread>
#include <QTime>
#include <iostream>
#include <string>

#include "SerialComm.h"

#include "selforg/abstractrobot.h"
#include "selforg/matrix.h"

#define COMMAND_BUFFER_NUM 20
#define MIC_CHAR_BUFFER_NUM 600
#define CAM_CHAR_BUFFER_NUM 4050
#define MAX_NUM_OF_BYTES 4050
#define EPUCK_SENSORS_MAX 2400
#define EPUCK_MOTORS_MAX 13



namespace lpzrobots{
    struct SensorNumbers{
    int IR0;
    int IR1;
    int IR2;
    int IR3;
    int IR4;
    int IR5;
    int IR6;
    int IR7;

    int GROUND0;
    int GROUND1;
    int GROUND2;

    int ACCX;
    int ACCY;
    int ACCZ;

    int AMBIENT_LIGHT0;
    int AMBIENT_LIGHT1;
    int AMBIENT_LIGHT2;
    int AMBIENT_LIGHT3;
    int AMBIENT_LIGHT4;
    int AMBIENT_LIGHT5;
    int AMBIENT_LIGHT6;
    int AMBIENT_LIGHT7;

    int MIC0;
    int MIC1;
    int MIC2;

    int CAM;
  }; //end Struct SensorNumbers

  struct MotorNumbers{
    int LED0;
    int LED1;
    int LED2;
    int LED3;
    int LED4;
    int LED5;
    int LED6;
    int LED7;
    int LED_BODY;
    int LED_FRONT;

    int MOTOR_LEFT;
    int MOTOR_RIGHT;

    int SOUND;
  }; //end Struct MotorNumbers
  
  typedef struct {
      std::string port;	//serial port address. In Linux per example /dev/rfcomm0
      
      bool SENSOR_STATE; //enables IR AMBIENT_LIGHT ACC and GROUND
      bool MIC_STATE; //enables all 3 microphones
      bool CAM_STATE; //enables CAMERA. connection will become very slow

      bool CAM_TYPE;  //color or black&white
      int CAM_HEIGHT; 
      int CAM_WIDTH; //maximum of Pixels is 3200
      int CAM_ZOOM; //1 2 4 8 or 16
  } EPuckConf;
      
  typedef double sensor;
  typedef double motor;
  
  
  
  
  class EPuckBluetooth:public QThread,public AbstractRobot{
  public:
    char version[100];
    EPuckBluetooth(EPuckConf);
    ~EPuckBluetooth();
    static EPuckConf getDefaultConfig();
    static int getRecommendConnectionSpeed(EPuckConf);
    void disconnectConnection();
    void init();

    virtual int getSensors(sensor* sensors, int _sensorCount);
    virtual void setMotors(const motor* motors, int _motorCount);
    virtual int getSensorNumber(){ return sensorCount; } /** returns number of sensors */
    virtual int getMotorNumber() { return motorCount; }/** returns number of motors */
    /* the following are not used here, you can ignore them but keep them*/
    virtual Position getPosition()     const {return Position(0,0,0);}
    virtual Position getSpeed()        const {return Position(0,0,0);}
    virtual Position getAngularSpeed() const {return Position(0,0,0);}
    virtual matrix::Matrix getOrientation() const {            matrix::Matrix m(3,3);            m.toId();            return m;        }
    virtual void processSensors(sensor* pSensor){};/*Default sensors processing*/
    virtual void processSensorsKOH(sensor* pSensor){};/*Your own sensors processing*/
    
    char camCharBuffer[CAM_CHAR_BUFFER_NUM];

    double connectionSpeedHz;
    void getNumOfMotSens(SensorNumbers&, int&, MotorNumbers&, int&);
    bool isRunning(){return abortThread;}
  private:
    void run();
    bool abortThread;

    void initConnection(std::string);

    
    EPuckConf conf;
    SerialComm *comm;
    
    int bytes; //number of received/transmitted bytes
    char command[COMMAND_BUFFER_NUM];
    char RxBuffer[45];
    char msg[20];

    unsigned int e_last_mic_scan_id;
    char micCharBuffer[MIC_CHAR_BUFFER_NUM];
    int camPixNum;
    
    void receiveSensors();
    void receiveMics();
    void receiveCam();
    void transmitMotors();

    SensorNumbers numOfSensor;
    MotorNumbers numOfMotor;
    
    unsigned int motorCount;
    unsigned int sensorCount;
    

    bool *arrUpdateMotor;
    int *arrMotor;
    int *arrSensor;
  };
  
  


    

} //end of namespace


#endif //EPUCKBLUETOOTH
