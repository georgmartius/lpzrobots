/************************************************************************/
/*shpererobot.h								*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/
#ifndef __SPHEREROBOTARMS_H
#define __SPHEREROBOTARMS_H


using namespace std;

#include "sliderservo.h"
#include "abstractrobot.h"
#include "raysensorbank.h"

typedef struct {
public:
  double diameter;
  double spheremass;
  double pendulardiameter; // automatically set
  double pendularmass;
  double pendularrange; 
  bool irAxis1;
  bool irAxis2;
  bool irAxis3;
} SphererobotArmsConf;


/**
 *This is a class, which models a snake like robot. It consists of a number of equal elements, each linked 
 *by a joint. This class is based upon the class roboter by the same author.
 *@author Marcel Kretschmann
 *@version beta
 **/
class SphererobotArms : public AbstractRobot
{
public:
  typedef enum objects { Base, Pendular1, Pendular2, Pendular3, Last } ;

private:
  static const int servono=3;

  dSpaceID sphererobot_space;
  SliderServo* servo[servono];
  int sensorno;
  char* name;
  int texture;
  double transparency;

protected:
  SphererobotArmsConf conf;
  RaySensorBank irSensorBank; // a collection of ir sensors

public:
  Object object[Last];

  /**
   *constructor
   **/ 
  SphererobotArms ( const OdeHandle& odeHandle, 
		    const SphererobotArmsConf& conf, double transparency=0.5 );
  
  virtual ~SphererobotArms();
	
  static SphererobotArmsConf getStandartConf(){
    SphererobotArmsConf c;
    c.diameter     = 1;
    c.spheremass   = 0.1;
    c.pendularmass  = 1.0;
    c.pendularrange  = 0.25; // range of the slider from center in multiple of diameter [-range,range]
    c.irAxis1=false;
    c.irAxis2=false;
    c.irAxis3=false;
  return c;
  }

  void setTexture(int tex) { texture = tex; }

  /**
   *Draws the geometrical objects of the robot.
   **/
  virtual void draw();
	
  /**Sets the sphere to position pos, sets color to c, and creates sphere if necessary.
   *This overwrides the function place of the class robot.
   *@param pos desired position of the snake in struct Position
   *@param c desired color for the snake in struct Color
   **/
  virtual void place (Position pos, Color *c = 0);
  
  /**
   *This is the collision handling function for snake robots.
   *This overwrides the function collisionCallback of the class robot.
   *@param data
   *@param o1 first geometrical object, which has taken part in the collision
   *@param o2 second geometrical object, which has taken part in the collision
   *@return true if the collision was threated  by the robot, false if not
   **/
  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
   */
  virtual void doInternalStuff(const GlobalData& globalData);
	
  /**
   *Writes the sensor values to an array in the memory.
   *@param sensor* pointer to the array
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  virtual int getSensors ( sensor* sensors, int sensornumber );
	
  /**
   *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
   *It is an linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1] 
   *@param motornumber length of the motor array
   **/
  virtual void setMotors ( const motor* motors, int motornumber );
	
  /**
   *Returns the number of motors used by the snake.
   *@return number of motors
   **/
  virtual int getMotorNumber();
  
  /**
   *Returns the number of sensors used by the robot.
   *@return number of sensors
   **/
  virtual int getSensorNumber();
	
  /**
   *Returns the position of the snake. Here the position of the snake is the position of the first element of the snake.
   *@return Position (x,y,z)
   **/
  virtual Position getPosition ();

  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);	
};

#endif
