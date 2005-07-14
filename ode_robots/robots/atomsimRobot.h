/************************************************************************/
/*atomsimRobot.h							*/
/*molecular robot, which controls a group auf connected atoms		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

using namespace std;

#include "atomsimAtom.h"


/**
 *
 *@author Marcel Kretschmann
 *@version alpha 0.5
 **/
class atomsimRobot : public Roboter
{
private:
	//Eigenschaften
	int roboterID;
	int* roboterIDzaehler;
	
	dWorldID* welt;
	dSpaceID* raum;
	
	int atomanzahl;
	int maxatomanzahl;
	double rekombinationstrennverhaeltniss;
	
	atomsimAtom* ursprungsatom;
	
	vector<atomsimAtom*>* atomsammlung;

public:

	/**
 	 *Konstruktor
 	 *@author
 	 *@version
 	 **/
	atomsimRobot ( int* start_roboterIDzaehler , dWorldID* start_welt , dSpaceID* start_raum , dJointGroupID* start_contactgroup , vector<atomsimAtom*>* start_atomsammlung , atomsimAtom* start_ursprungsatom , int start_maxatomanzahl , double start_rekombinationstrennverhaeltniss );
	
	/**
 	 *Destructor
 	 *@author
 	 *@version
 	 **/
	virtual ~atomsimRobot ();
	
	
	/**
 	 *Recursive deletation function for all atoms, connected to the robot
 	 *@author
 	 *@version
 	 **/
	//virtual void rekursiveAtomDeletation ( atomsimAtom* a );
	
	/**
	 *Calls the function drawRobot-function
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual void draw ();
	
	/**
	 *Draws recursive all Atoms, belonging to the robot
	 *@param i should be normaly true, false causes that the Origin Atom is not drawn
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual void drawRobot ( atomsimAtom* atom , bool i );
	
	/**
	 *Decides if some collisions of the robot should not be threated by by the collision management.
	 *This overwrides the function in the roboter class.
	 *@param o1 Geometrieobjekt 1, dass an der Kollision beteiligt ist
	 *@param o2 Geometrieobjekt 2, dass an der Kollision beteiligt ist
	 *@return true, if the collision should not be threated, false else
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual bool kollisionsermittlung ( dGeomID o1 , dGeomID o2 );
	
	/**Sets the simatomRobot to position pos, sets color to c, and creates robot if necessary.
	 *This overwrides the function place of the class robot.
	 *@param pos desired position of the atomsimRobot in struct Position
	 *@param c desired color for the atomsimRobot in struct Color
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
	virtual void setMotors ( const motor* motors, int motornumber );
	
	/**
	 *Returns the number of sensors used by the robot.
	 *@return number of sensors
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual int getSensorNumber();
	
	/**
	*Returns the number of motors used by the snake.
	*@return number of motors
	*@author Marcel Kretschmann
	*@version final
	**/
	virtual int getMotorNumber();
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	//rekursive Funktion, welches das lineare Sensorfeld mittels Tiefensuche aktualisiert
	virtual int sensoraktualisierung ( atomsimAtom* atom , int i , bool steuerung );
	
	/**
	*Returns the position of the snake. Here the position of the snake is the position of the first element of the snake.
	*@return Position (x,y,z)
	*@author Marcel Kretschmann
	*@version final
	**/
	virtual Position getPosition ();
	
	/**
	 *Returns a list with the positionvectors of all segments of the robot
	 *no function
	 *@param poslist list with positionvectors (of all robot segments) (free after use!)
	 *@return length of the list
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual int getSegmentsPosition ( vector<Position> &poslist );
	
	/**
	*Prints some internal robot parameters. Actualy it prints all sensor data of one callculation step.
	*@author Marcel Kretschmann
	*@version beta
	**/
	virtual void getStatus ();
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	virtual atomsimAtom* getUrsprungsatom ();
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	virtual double getRekombinationsTrennverhaeltniss ();
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	//fuegt einen leeren Sensor zum Sensor-Array hinzu
	virtual void addSensor ();
	
	//entfernt einen bestehenden Sensor aus dem Sensor-Array
	virtual void delSensor ();
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	//rekursive Funktion, welches das lineare Sensorfeld mittels Tiefensuche aktualisiert
	virtual const motor* motoraktualisierung ( atomsimAtom* atom , const motor* motors , int motornumber , bool steuerung );
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	virtual int getAtomAnzahl ();
	
	/**
	 *Calculates the difference off the actual angle and the last calculated angle of a sensor.
	 *This overwrites the function from Roboter, because an tanh-operation is used to set the sensor value maxima to 1 and -1
	 *@param motor number of the motor in the motor-list
	 *@param X pointer to a variable, to save the calculated difference
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual void getWinkelDifferenz ( int motor , double* X );
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	virtual int rekursionszaehler ( atomsimAtom* a , int z );
	
	/**
 	 *no optimal performance, but so it is an easy way
 	 *@author
 	 *@version
 	 **/
	virtual void rekursivVerschieben ( atomsimAtom* a , Position pos );
		
//********************************Evolutionsabschnitt*********************************

//*************************************Selektion**************************************
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	virtual double getFitness ();

//************************************Rekombination***********************************

	
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	//rekursives Kopieren eines atomsimRobots ab dem Atom  a
	virtual atomsimRobot* rekursivKopieren ( atomsimAtom* a , bool firstcall );
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	virtual bool roboterAuftrennen ( atomsimAtom* a = 0 , atomsimAtom** newrobotpart = 0 , atomsimAtom** endofpieceone = 0 , double trennverhaeltniss = 0 );
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	virtual void roboterRekombination ( int vermehrungsart , double trennverhaeltniss , atomsimRobot* partner , atomsimRobot** speicherort1 , atomsimRobot** speicherort2 , Position newpos1 , Position newpos2 );
	
};
