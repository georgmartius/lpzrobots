#ifndef __ABSTRACTROBOT_H
#define __ABSTRACTROBOT_H

#include <ode/common.h>

#include <vector>
using namespace std;

typedef double sensor;
typedef double motor;

typedef struct
{
	double r;
	double g;
	double b;
} Color;

typedef struct
{
  dBodyID body;
  dGeomID geom;
} Object;


/**
 * Abstract class (interface) for robot 
 * 
 * 
 */
class AbstractRobot{
public:


  /**
   * Constructor
   * @param w world in which robot should be created
   * @param s space in which robot should be created
   */
  AbstractRobot(dWorldID *w, dSpaceID *s){
    world=w;
    space=s;
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] 
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber)=0;

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  virtual void setMotors(motor* motors, int motornumber)=0;

  /** returns number of sensors
  */
  virtual int getSensorNumber()=0;

  /** returns number of motors
  */
  virtual int getMotorNumber()=0;

  /** sets the position of robot to pos
      @param pos vector of desired position (x,y,z)
   */
  virtual void setPosition(const dVector3& pos)=0;
 
  /** returns position of robot 
      @param pos vector of desired position (x,y,z)
   */
  virtual void getPosition(dVector3& pos)=0;


  /** returns a list with the positionvectors of all segments of the robot
      @param poslist vector with positionvectors (of all robot segments) as elements
  */
  //  virtual void getSegmentsPosition(&vector<dVector3> poslist)=0;
  //remove commentar !!
 protected:

  dSpaceID *space;
  dWorldID *world;

  Color color;

};

#endif
 
