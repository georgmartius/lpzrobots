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
	double x;
	double y;
	double z;
} Position;

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
   * @param c contactgroup for collision treatment
   */
  AbstractRobot(dWorldID *w, dSpaceID *s, dJointGroupID *c){
    world=w;
    space=s;
    contactgroup=c;
  };

  /// draws the robot
  virtual void draw() = 0;

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
  virtual void place(Position pos , Color *c = 0) = 0;

  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects and the default routine)
   *  else false (collision is passed to other objects and (if not treated) to the default routine).
   */
  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2){
    return false;
  }
  

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

  /** returns position of robot 
      @param pos vector of desired position (x,y,z)
   */
  virtual Position getPosition()=0;

  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist) = 0;


 protected:

  dSpaceID *space;
  dWorldID *world;

  dJointGroupID *contactgroup;

  Color color;

} ;



 #endif
 
