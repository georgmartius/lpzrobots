/************************************************************************/
/*schlangeforce.h							*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

using namespace std;

#include "schlange.h"
#include "configurable.h"

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
class SchlangeForce : public Schlange, public Configurable
{
private:

	char name[50];
	paramval gamma;
	paramval friction_ground;
	paramval factor_force;

public:

	/**
	 *constructor
	 *@param startRoboterID ID, which should be managed clearly
	 *@param welt pointer to the ODE-simulation world, which contains the whole simulation
	 *@param raum pointer to the ODE-simulation space, which contains all geometrical objects
	 *@param start_contactgroup pointer to the JointGroup, which is used for collision management
	 *@param start_Sensorzahl number of sensors of the robot
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
	 *@author Marcel Kretschmann
	 *@version beta
	 **/ 
	SchlangeForce ( int startRoboterID , dWorldID* welt , dSpaceID* raum , dJointGroupID* start_contactgroup , int start_Sensoranzahl , double start_x , double start_y , double start_z , int armanzahl , double glieder_laenge , double glieder_durchmesser , double glieder_abstand , double glieder_masse , double start_maxmotorkraft , double start_geschwindigkeitsfaktor , ausgabemodus start_ausgabeart );
	
	/**
	*Destruktor
	*@author Marcel Kretschmann
	*@version beta
	**/
	virtual ~SchlangeForce();
	
	

	
	/**
	*Reads the actual motor commands from an array, an sets all motors of the snake to this values.
	*It is an linear allocation.
	*@param motors pointer to the array, motor values are scaled to [-1,1] 
	*@param motornumber length of the motor array
	*@author Marcel Kretschmann
	*@version beta
	**/
	virtual void setMotors ( const motor* motors, int motornumber );
	

	/// returns the name of the object (with version number)
	virtual constparamkey getName() const {return name; } 

	/** The list of all parameters with there value as allocated lists.
	    @param keylist,vallist will be allocated with malloc (free it after use!)
	    @return length of the lists
	*/
	virtual int getParamList(paramkey*& keylist,paramval*& vallist) const;

	virtual paramval getParam(paramkey key) const;

	virtual bool setParam(paramkey key, paramval val);
	

};

