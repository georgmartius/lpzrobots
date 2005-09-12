/************************************************************************/
/*schlangeforce.h							*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/
#ifndef __SCHLANGEFORCE_H
#define __SCHLANGEFORCE_H

#include<vector>
#include<assert.h>
using namespace std;

#include "abstractrobot.h"
#include "configurable.h"
#include "hingeservo.h"

typedef struct {
public:
  int segmNumber;     ///  number of snake elements
  double segmLength;  /// length of one snake element
  double segmDia;     ///  diameter of a snake element
  double segmMass;    ///  mass of one snake element
  double servoPower;  ///  power of the servos
  double frictionGround; /// friction with ground
  double jointLimit;     /// maximal angle for the joints (M_PI/2 = 90 degree)
} SchlangeServoConf;


/**
 * This is a class, which models a snake like robot. 
 * It consists of a number of equal elements, each linked 
 * by a joint powered by 2 servos
 **/
class SchlangeServo: public AbstractRobot, public Configurable
{
private:

  char name[50];
  bool created;

  vector <Object> objects;
  vector <dJointID> joints;
  vector <HingeServo*> servos;
  dSpaceID snake_space;
  SchlangeServoConf conf;
  int texture;

public:
  SchlangeServo ( const ODEHandle& odeHandle, 
		  const SchlangeServoConf& conf, const char* name);

  static SchlangeServoConf getDefaultConf(){
    SchlangeServoConf conf;
    conf.segmNumber = 10;     ///  number of snake elements
    conf.segmLength = 0.8;  /// length of one snake element
    conf.segmDia    = 0.2;     ///  diameter of a snake element
    conf.segmMass   = 0.4;    ///  mass of one snake element
    conf.servoPower = 10;  ///  power of the servos
    conf.frictionGround = 1.0; /// friction with ground
    conf.jointLimit =  M_PI/4;
    return conf;
  }

  /**
   *Destruktor
   *@author Marcel Kretschmann
   *@version beta
   **/
  virtual ~SchlangeServo();
	
  

  /**sets the robot to position pos, sets color to c, and creates robot if necessary
   *Only sets the color, there is no special place operation for this unspecified robot.
   *@param pos new position of the robot
   *@param c desired color for the robot in struct Color
   **/
  virtual void place(Position pos, Color *c);

  /**
   *draws all geometrical objects
   **/
  virtual void draw();

  /**
   *This is the collision handling function for snake robots.
   *This overwrides the function collisionCallback of the class robot.
   *@param data
   *@param o1 first geometrical object, which has taken part in the collision
   *@param o2 second geometrical object, which has taken part in the collision
   *@return true if the collision was threated  by the robot, false if not
   **/
  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);	

	
  /**
   *Reads the actual motor commands from an array, 
   *an sets all motors of the snake to this values.
   *It is an linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1] 
   *@param motornumber length of the motor array
   **/
  virtual void setMotors ( const motor* motors, int motornumber );

  /**
   *Writes the sensor values to an array in the memory.
   *@param sensor* pointer to the array
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  virtual int getSensors ( sensor* sensors, int sensornumber );
	
  /** returns number of sensors
   */
  virtual int getSensorNumber() { assert(created); return servos.size(); }

  /** returns number of motors
   */
  virtual int getMotorNumber(){ assert(created); return servos.size(); }

  /** returns position of robot 
      @return position robot position in struct Position  
  */
  virtual Position getPosition();

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);

  /// returns the name of the object (with version number)
  virtual constparamkey getName() const { return name; } 

  /** The list of all parameters with there value as allocated lists.
      @param keylist,vallist will be allocated with malloc (free it after use!)
      @return length of the lists
  */
  virtual int getParamList(paramkey*& keylist,paramval*& vallist) const;

  virtual paramval getParam(paramkey key) const;

  virtual bool setParam(paramkey key, paramval val);


  /** sets the texture */
  virtual void setTexture(int texture);

private:
  void create(const Position& pos);
  void destroy();
};

#endif
