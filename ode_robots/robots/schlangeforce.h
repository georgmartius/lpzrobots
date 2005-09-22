/************************************************************************/
/*schlangeforce.h							*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/
#ifndef __SCHLANGEFORCE_H
#define __SCHLANGEFORCE_H

using namespace std;

#include "schlange.h"
#include "configurable.h"

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
	paramval frictionGround;
	paramval factorForce;
	paramval factorSensors;

public:

	/**
	 *constructor
	 *@param startRoboterID ID, which should be managed clearly
	 *@author Marcel Kretschmann
	 *@version beta
	 **/ 
	SchlangeForce ( int startRoboterID , const OdeHandle& odeHandle, 
			const SchlangenConf& conf );

	static SchlangenConf getDefaultConf(){
	  return Schlange::getStandartConf();
	}

	/**
	*Destruktor
	*@author Marcel Kretschmann
	*@version beta
	**/
	virtual ~SchlangeForce();
	

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
  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
   */
  virtual void doInternalStuff(const GlobalData& globalData);
	

	
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
	 *Writes the sensor values to an array in the memory.
	 *@param sensor* pointer to the array
	 *@param sensornumber length of the sensor array
	 *@return number of actually written sensors
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual int getSensors ( sensor* sensors, int sensornumber );
	

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

#endif
