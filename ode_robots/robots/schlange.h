/************************************************************************/
/*schlange.h								*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/
#ifndef __SCHLANGE_H
#define __SCHLANGE_H


using namespace std;

#include "roboter.h"

//dadurch wird mit den Double-Genauigkeitszeichenmethoden gearbeitet
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

typedef struct {
public:
  int armAnzahl; ///  number of snake elements
  double gliederLaenge; /// length of one snake element
  double gliederDurchmesser; ///  diameter of a snake element
  /**  distance between two snake elements; 
       0 means there is a distance of the length of one snake element 
       between each snake element an its successor */
  double gliederAbstand; 
  double gliederMasse; ///  mass of one snake element
  double maxMotorKraft; ///  maximal force used by the motors of the snake
  double factorForce; ///  factor for the speed, which the motors of the snake use
  double factorSensors; /// sensors values are multiplied with this value
  double frictionGround; /// friction with ground
  /** angle: sensor values are the angle of the joints; 
      anglerate: sensor values are the angle rates of the joints*/
  ausgabemodus ausgabeArt;     
  double maxWinkel; /// maximal angle for the joints (M_PI/2 = 90 degree)
} SchlangenConf;


/**
 *This is a class, which models a snake like robot. It consists of a number of equal elements, each linked 
 *by a joint. This class is based upon the class roboter by the same author.
 *@author Marcel Kretschmann
 *@version beta
 **/
class Schlange : public Roboter
{
private:
  
  std::vector<dJointID> skyJoints; // for fixing segment 0 in the sky
  dSpaceID snake_space;

protected:
  SchlangenConf conf;

public:

  /**
   *constructor
   *@param startRoboterID ID, which should be managed clearly
   *@author Marcel Kretschmann
   *@version beta
   **/ 
  Schlange ( int startRoboterID , const ODEHandle& odeHandle, 
	     const SchlangenConf& conf );
	
  /**
   *Destruktor
   *@author Marcel Kretschmann
   *@version beta
   **/
  virtual ~Schlange();
	
  static SchlangenConf getStandartConf(){
    SchlangenConf c;
    c.armAnzahl=4;
    c.gliederLaenge=0.8;
    c.gliederDurchmesser=0.2;
    c.gliederAbstand=0;
    c.gliederMasse=0.4;
    c.maxMotorKraft=2;
    c.ausgabeArt=anglerate;    
    c.maxWinkel=M_PI/4;
    c.factorForce=4.0;
    c.factorSensors=5.0;
    c.frictionGround=0.1;
    c.maxWinkel=M_PI/4;
    return c;
  }

  /**
   *Zeichnet die Koerper-GeometrieObjekte.
   *@author Marcel Kretschmann
   *@version beta
   **/
  virtual void draw();
	
  /** fix segment 0 in the sky
   */
  virtual void fixInSky();
	
  /**Sets the snake to position pos, sets color to c, and creates snake if necessary.
   *This overwrides the function place of the class robot.
   *@param pos desired position of the snake in struct Position
   *@param c desired color for the snake in struct Color
   *@author Marcel Kretschmann
   *@version beta
   **/
  virtual void place (Position pos, Color *c = 0);
	
  static void mycallback(void *data, dGeomID o1, dGeomID o2);
  
  /**
   *This is the collision handling function for snake robots.
   *This overwrides the function collisionCallback of the class robot.
   *@param data
   *@param o1 first geometrical object, which has taken part in the collision
   *@param o2 second geometrical object, which has taken part in the collision
   *@return true if the collision was threated  by the robot, false if not
   *@author Marcel Kretschmann
   *@version beta
   **/
  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

	
  /**
   *Writes the sensor values to an array in the memory.
   *@param sensor* pointer to the array
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   *@author Marcel Kretschmann
   *@version beta
   **/
  virtual int getSensors ( sensor* sensors, int sensornumber );
	
  /**
   *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
   *It is an linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1] 
   *@param motornumber length of the motor array
   *@author Marcel Kretschmann
   *@version beta
   **/
  virtual void Schlange::setMotors ( const motor* motors, int motornumber );
	
  /**
   *Returns the number of motors used by the snake.
   *@return number of motors
   *@author Marcel Kretschmann
   *@version final
   **/
  virtual int Schlange::getMotorNumber();
	
  /**
   *Updates the sensorarray.
   *This overwrides the function sensoraktualisierung of the class robot
   *@author Marcel Kretschmann
   *@version beta
   **/
  void sensoraktualisierung ( );
	
  /**
   *Returns the position of the snake. Here the position of the snake is the position of the first element of the snake.
   *@return Position (x,y,z)
   *@author Marcel Kretschmann
   *@version final
   **/
  virtual Position getPosition ();
	
  /**
   *Returns the position of one element of the snake.
   @param n number of the snake element
   *@return Position (x,y,z)
   *@author Marcel Kretschmann
   *@version final
   **/
  virtual Position getPosition ( int n );
	
  /**
   *Prints some internal robot parameters. Actualy it prints all sensor data of one callculation step.
   *@author Marcel Kretschmann
   *@version beta
   **/
  virtual void Schlange::getStatus ();

};

#endif
