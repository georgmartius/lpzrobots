/********************************************************************************/
/*roboter.h									*/
/*Basic robot-Class, designed for the robot system, designed by the author	*/
/*@author Marcel Kretschmann							*/
/*@version beta									*/
/*										*/
/********************************************************************************/

#include "roboter.h"


/***************************Hilfsfunktionen******************************/

/**
 *This funktion enables the posibility to directly access the values of a body position.
 *@param basis ODE internal robot-ID
 *@param para 0 = x, 1 = y, 2 = z
 *@author Marcel Kretschmann
 *@version final
 **/
double dBodyGetPositionAll ( dBodyID basis , int para )
{
	const dReal* pos;
	pos = dBodyGetPosition ( basis );
	switch (para)
	{
		case 1: return pos[0]; break; //X
		case 2: return pos[1]; break; //Y
		case 3: return pos[2]; break; //Z
	}
	return 0;
}

/***********************************************************************/

/**
 *constructor
 *@param startRoboterID ID, which should be managed clearly
 *@param welt pointer to the ODE-simulation world, which contains the whole simulation
 *@param raum pointer to the ODE-simulation space, which contains all geometrical objects
 *@param start_contactgroup pointer to the JointGroup, which is used for collision management
 *@param start_Sensorzahl number of sensors of the robot
 *@author Marcel Kretschmann
 *@version beta
 **/
 Roboter::Roboter ( int startRoboterID , dWorldID welt , dSpaceID raum , dJointGroupID start_contactgroup , int start_Sensoranzahl ) :
    AbstractRobot::AbstractRobot ( welt , raum , start_contactgroup )
{
	roboterID = startRoboterID;
	for ( int n = 0; n++ < start_Sensoranzahl; addSensor() );
}

/**
 *destructor
 *@author Marcel Kretschmann
 *@version beta
 **/
Roboter::~Roboter()
{
}


/*************************************************************/
/**
 *Returns the ID of the robot
 *@author Marcel Kretschmann
 *@version beta
 **/
 int Roboter::getRoboterID ()
 {
 	return roboterID;
 }

/**
 *draws all geometrical objects
 *@author Marcel Kretschmann
 *@version beta
 **/
 void Roboter::draw()
{

}

/**sets the robot to position pos, sets color to c, and creates robot if necessary
 *Only sets the color, there is no special place operation for this unspecified robot.
 *@param pos new position of the robot
 *@param c desired color for the robot in struct Color
 *@author Marcel Kretschmann
 *@version beta
 **/
void Roboter::place (Position pos, Color *c)
{
	color.r = (*c).r;
	color.g = (*c).g;
	color.b = (*c).b;
}

/**
 *This is the universal collision handling function of all robots. Each robot handles ist own collisions. There is also the posibility that the robot cancels the collision handling, but then it also returns the same value, as if it has handled the collision. So it is possible that there are special parts of the Robot, which could act whithout being influenced by other parts or geometrical objects of the simulation environment.
 *@param data
 *@param o1 first geometrical object, which has taken part in the collision
 *@param o2 second geometrical object, which has taken part in the collision
 *@return true if the collision was threated  by the robot, false if not
 *@author Marcel Kretschmann
 *@version beta
 **/
 bool Roboter::collisionCallback(void *data, dGeomID o1, dGeomID o2)
{
	//checks if one of the collision objects is part of the robot
	bool tmp_kollisionsbeteiligung = false;
	for ( int n = 0; n < getObjektAnzahl (); n++ )
	{
		if ( getObjektAt ( n ).geom == o1 || getObjektAt ( n ).geom == o2 )
		{
			tmp_kollisionsbeteiligung = true;
			break;
		}
	}

	if ( tmp_kollisionsbeteiligung == true )		
	{
		int i,n;
		const int N = 10;
		dContact contact[N];
		bool kollission = false;

		//tests, if a special collision should not be threated
		if ( Roboter::kollisionsermittlung ( o1 , o2 ) == true )
			kollission = true;

		if ( kollission == false )
		{
			n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
			if (n > 0)
				for (i=0; i<n; i++)
				{
					contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
					dContactSoftERP | dContactSoftCFM | dContactApprox1;
					contact[i].surface.mu = 0.8;
					contact[i].surface.slip1 = 0.005;
					contact[i].surface.slip2 = 0.005;
					contact[i].surface.soft_erp = 1;
					contact[i].surface.soft_cfm = 0.00001;

					dJointID c = dJointCreateContact ( (world) , contactgroup , &contact[i] );
					dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
				}
		}
		return true; //if collision was threated by this robot
	}
	else return false; //if collision was not threated by this robot
}

/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the array
 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 *@author Marcel Kretschmann
 *@version beta
 **/
 int Roboter::getSensors ( sensor* sensors, int sensornumber )
{
	sensoraktualisierung ();
	for ( int n = 0; n < sensornumber; n++ )
		getWinkelDifferenz ( n , sensors++ );
	
	return getSensorfeldGroesse (); //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden
}

/**
 *Reads the actual motor commands from an array, an sets all motors of the robot to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 *@author Marcel Kretschmann
 *@version beta
 **/
 void Roboter::setMotors ( const motor* motors, int motornumber )
{
	for ( int n = 0; n < motornumber; n ++ )
		dJointSetAMotorParam ( motorliste[n] , dParamVel , *(motors++) );
}


/**
 *Returns the number of sensors used by the robot.
 *@return number of sensors
 *@author Marcel Kretschmann
 *@version final
 **/
int Roboter::getSensorNumber()
{
	return getSensorfeldGroesse ();
}

/**
 *Returns the number of motors used by the robot.
 *@return number of motors
 *@author Marcel Kretschmann
 *@version final
 **/
 int Roboter::getMotorNumber()
{
	return motorliste.size ();
}

/**
 *Returns a list with the positionvectors of all segments of the robot
 *@param poslist list with positionvectors (of all robot segments) (free after use!)
 *@return length of the list
 *@author Marcel Kretschmann
 *@version final
 **/
int Roboter::getSegmentsPosition ( vector<Position> &poslist )
{
	const dReal* tmp;
	for ( int n = 0; n < getObjektAnzahl (); n++ )
	{
		 tmp = dBodyGetPosition ( getObjektAt ( n ).body );
		 poslist[n].x = tmp[0];
		 poslist[n].y = tmp[1];
		 poslist[n].z = tmp[2];
	}
	return getObjektAnzahl ();
}

/**
 *Returns the number of Objects, the robot consists of
 *@return int number of Objects
 *@author Marcel Kretschmann
 *@version final
 **/
 int Roboter::getObjektAnzahl ()
{
	return objektliste.size ();
}

/**
 *Returns the number of joints, linking the robot parts
 *@return int number of joints
 *@author Marcel Kretschmann
 *@version final
 **/
int Roboter::getJointAnzahl ()
{
	return jointliste.size ();
}

/**
 *Returns a special Object from the list of all Objects, the robot consits off
 *@param n The position of the Object in the list ob all Objects
 *@return Object
 *@author Marcel Kretschmann
 *@version final
 **/
Object Roboter::getObjektAt ( int n )
{
	return objektliste[n];
}

/**
 *Returns one special joint from the list of all joints, which link the robots parts
 *@param n The position of the joint in the list ob all joints belonging to the robot
 *@return dJointID
 *@author Marcel Kretschmann
 *@version final
 **/
dJointID Roboter::getJointAt ( int n )
{
	return jointliste[n];
}

/**
 *Returns one special motor joint from the list of all motors
 *@param n The position of the motor in the list ob all motors belonging to the robot
 *@return dJointID
 *@author Marcel Kretschmann
 *@version final
 **/
dJointID Roboter::getMotorAt ( int n )
{
	return motorliste[n];
}

/**
 *Calculates the difference off the actual angle and the last calculated angle of a sensor..
 *@param motor number of the motor in the motor-list
 *@param X pointer to a variable, to save the calculated difference
 *@author Marcel Kretschmann
 *@version final
 **/
void Roboter::getWinkelDifferenz ( int motor , double* X )
{
	//Eingangsdaten sind die Winkeldifferenzen eines Roboterberechnungsschrittes
	//Abfangen des Winkeldifferenzsprunges bei ueberschreiten er 2PI-Marke
	//tritt auf wenn die -PI- oder die PI-Marke ueberschritten wird	
	
	double w = sensorfeld[motor].istwinkel - sensorfeld[motor].istwinkel_alt;
	
	if ( ( w < -M_PI )
	|| ( w >  M_PI ) )
	{
		//1. Fall(PI-Marke wird ueberschritten, also annaeherung vom Positiven) -> es ergibt sich eine negative Winkeldifferenz
		if ( w > M_PI )
		{
			*X = -(2*M_PI - w);
			//dsPrint ( "%lf  %lf => %lf \n" , sensorfeld[motor].istwinkel ,  sensorfeld[motor].istwinkel_alt , -(2*M_PI-w)/M_PI );
		}
		//2. Fall(-PI-Marke wird ueberschritten, also Annaeherung vom Negativen) -> es ergibt sich eine positive Winkeldifferenz
		if ( w < -M_PI )
		{
			*X = (2*M_PI + w);
			//dsPrint ( "%lf  %lf => %lf \n" , sensorfeld[motor].istwinkel ,  sensorfeld[motor].istwinkel_alt , (2*M_PI+w)/M_PI );
		}
	}
	else
	{
		*X = w;
		//dsPrint ( "%lf  %lf = %lf \n" , sensorfeld[motor].istwinkel ,  sensorfeld[motor].istwinkel_alt , w/M_PI );
	}
}

/**
 *Adds an empy sensor to the sensorlist.
 *@author Marcel Kretschmann
 *@version final
 **/
void Roboter::addSensor ()
{
	Sensor tmpSensor;
	tmpSensor.istwinkel = 0;
	tmpSensor.istwinkel_alt = 0;
	sensorfeld.push_back ( tmpSensor );
}
	
/**
 *Removes the last sensor of the sensorlist
 *@author Marcel Kretschmann
 *@version final
 **/
void Roboter::delSensor ()
{
	sensorfeld.pop_back ( );
}
	
/**
 *Returns the number of active sensors.
 *@return number of sensors
 *@author Marcel Kretschmann
 *@version final
 **/
int Roboter::getSensorfeldGroesse ()
{
	return sensorfeld.size ();
}
	
/**
 *Updates the sensorarray.
 *@author Marcel Kretschmann
 *@version beta
 **/
void Roboter::sensoraktualisierung ( )
{
	for ( int n = 0; n < getSensorfeldGroesse (); n++ )
	{
		sensorfeld[n].istwinkel_alt = sensorfeld[n].istwinkel;
		
		sensorfeld[n].istwinkel = dJointGetAMotorAngle ( getMotorAt ( n ) , 0 );
	}
}

/**
 *Decides if some collisions of the robot should not be threated by by the collision management.
 *@param o1 Geometrieobjekt 1, dass an der Kollision beteiligt ist
 *@param o2 Geometrieobjekt 2, dass an der Kollision beteiligt ist
 *@return true, if the collision should not be threated, false else
 *@author Marcel Kretschmann
 *@version beta
 **/
bool Roboter::kollisionsermittlung ( dGeomID o1 , dGeomID o2 )
{
	return false;
}
	
/**
 *Prints some internal robot parameters. This only works in some subclasses.
 *@author Marcel Kretschmann
 *@version beta
 **/
void Roboter::getStatus ()
{

}
