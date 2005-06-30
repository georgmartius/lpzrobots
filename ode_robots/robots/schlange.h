/************************************************************************/
/*schlange.h								*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

using namespace std;

#include "roboter.h"


//dadurch wird mit den Double-Genauigkeitszeichenmethoden gearbeitet
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

/**
 *This is a class, which models a snake like robot. It consists of a number of equal elements, each linked 
 *by a joint. This class is based upon the class roboter by the same author.
 *@author Marcel Kretschmann
 *@version beta
 **/
class Schlange : public Roboter
{
private:

	int schlangenarmanzahl;
	double gliederdurchmesser;
	double gliederlaenge;
	
	double geschwindigkeitsfaktor;
	double maxmotorkraft;
	
	ausgabemodus ausgabeart;

public:

	/**
	 *constructor
	 *@param startRoboterID ID, which should be managed clearly
	 *@param welt pointer to the ODE-simulation world, which contains the whole simulation
	 *@param raum pointer to the ODE-simulation space, which contains all geometrical objects
	 *@param start_contactgroup pointer to the JointGroup, which is used for collision management
	 *@param start_x x coordinate at the begin of the simulation
	 *@param start_y y coordinate at the begin of the simulation
	 *@param start_z z coordinate at the begin of the simulation
	 *@param armanzahl number of snake elements
	 *@param start_laenge length of one snake element
	 *@param start_durchmesser diameter of a snake element
	 *@param start_abstand distance between two snake elements; 0 means there is a distance of the length of one snake element between each snake element an its successor
	 *@param start_masse mass of one snake element
	 *@param start_maxmotorkraft maximal force used by the motors of the snake
	 *@param start_geschwindigkeitsfaktor factor for the speed, which the motors of the snake use
	 *@param start_ausgabeart angle: sensor values are the angle of the joints; anglerate: sensor values are the angle rates of the joints
	 *@author Marcel Kretschmann
	 *@version beta
	 **/ 
	Schlange ( int startRoboterID , dWorldID* welt , dSpaceID* raum , dJointGroupID* start_contactgroup , double start_x , double start_y , double start_z , int armanzahl , double glieder_laenge , double glieder_durchmesser , double glieder_abstand , double glieder_masse , double start_maxmotorkraft , double start_geschwindigkeitsfaktor , ausgabemodus start_ausgabeart );
	
	/**
	*Destruktor
	*@author Marcel Kretschmann
	*@version beta
	**/
	virtual ~Schlange();
	
	/**
	*Zeichnet die Koerper-GeometrieObjekte.
	*@author Marcel Kretschmann
	*@version beta
	**/
	virtual void draw();
	
	/**
	 *Decides if some collisions of the robot should not be threated by by the collision management.
	 *This overwrides the function in the roboter class.
	 *Here it makes the simulation ignore collisions between neighbouring snake elements, so that the snake can move, an does not explode.
	 *@param o1 Geometrieobjekt 1, dass an der Kollision beteiligt ist
	 *@param o2 Geometrieobjekt 2, dass an der Kollision beteiligt ist
	 *@return true, if the collision should not be threated, false else
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	bool kollisionsermittlung ( dGeomID o1 , dGeomID o2 );
	
	/**Sets the snake to position pos, sets color to c, and creates snake if necessary.
	 *This overwrides the function place of the class robot.
	 *@param pos desired position of the snake in struct Position
	 *@param c desired color for the snake in struct Color
	 *@author Marcel Kretschmann
	 *@version beta
	**/
	virtual void place (Position pos, Color *c);
	
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

