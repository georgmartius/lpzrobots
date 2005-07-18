/************************************************************************/
/*schlangeforce.cpp								*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

#include "schlangeforce.h"
//#include "configurable.h"


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


SchlangeForce::SchlangeForce ( int startRoboterID , dWorldID welt , dSpaceID raum , dJointGroupID start_contactgroup , double start_x , double start_y , double start_z , int armanzahl , double glieder_laenge , double glieder_durchmesser , double glieder_abstand , double glieder_masse , double start_maxmotorkraft , double start_geschwindigkeitsfaktor , ausgabemodus start_ausgabeart ) :
Schlange::Schlange ( startRoboterID , welt , raum , start_contactgroup , start_x , start_y , start_z , armanzahl , glieder_laenge , glieder_durchmesser , glieder_abstand , glieder_masse , start_maxmotorkraft , start_geschwindigkeitsfaktor , start_ausgabeart ) 
{
        // prepare name;
        Configurable::insertCVSInfo(name, "$RCSfile$", 
		           	            "$Revision$");
	gamma=0.4;
	frictionGround=0.1;
	factorForce=4.0;
	factorSensors=5.0;

}
	
/**
 *Destruktor
 *@author Marcel Kretschmann
 *@version beta
 **/
SchlangeForce::~SchlangeForce()
{
}
	



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
 bool SchlangeForce::collisionCallback(void *data, dGeomID o1, dGeomID o2)
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
		if ( SchlangeForce::kollisionsermittlung ( o1 , o2 ) == true )
			kollission = true;

		if ( kollission == false )
		{
			n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
			if (n > 0)
				for (i=0; i<n; i++)
				{
					contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
					dContactSoftERP | dContactSoftCFM | dContactApprox1;
					contact[i].surface.mu = frictionGround;
					contact[i].surface.slip1 = 0.005;
					contact[i].surface.slip2 = 0.005;
					contact[i].surface.soft_erp = 1;
					contact[i].surface.soft_cfm = 0.00001;

					dJointID c = dJointCreateContact ( world , contactgroup , &contact[i] );
					dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
				}
		}
		return true; //if collision was threated by this robot
	}
	else return false; //if collision was not threated by this robot
}




/**
 *Reads the actual motor commands from an array, an sets all motors (forces) of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 **/
void SchlangeForce::setMotors ( const motor* motors, int motornumber )
{

  // controller output as torques 
  for (int i=0; i<motornumber/2; i++){
    dJointAddUniversalTorques( jointliste[i],factorForce* motors[2*i],
			       factorForce*motors[2*i +1]);  // motorcommand
  dJointAddUniversalTorques( jointliste[i], -(gamma)*dJointGetUniversalAngle1Rate(jointliste[i]),
			       -(gamma) *dJointGetUniversalAngle2Rate(jointliste[i]) ); // friction
  }


  /*  // controller outputs as wheel velocity
  for ( int n = 0; n < motornumber; n++ )
    if ( n % 2 == 0 )
      {
	dJointSetUniversalParam ( getJointAt(n/2) , dParamVel , *(motors++)*geschwindigkeitsfaktor );
	dJointSetUniversalParam ( jointliste[n/2] , dParamFMax , maxmotorkraft );
      }
    else
      {
	dJointSetUniversalParam ( jointliste[n/2] , dParamVel2 , *(motors++)*geschwindigkeitsfaktor );
	dJointSetUniversalParam ( jointliste[n/2] , dParamFMax2 , maxmotorkraft );
      }
  */
}	

/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the arrays

 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 *@author Marcel Kretschmann
 *@version beta
 **/
int SchlangeForce::getSensors ( sensor* sensors, int sensornumber )
{
	sensoraktualisierung ();
	for ( int n = 0; n < sensornumber; n++ )
	{
		if ( ausgabeart == angle )
			*sensors = sensorfeld[n].istwinkel;
		if ( ausgabeart == anglerate )
			getWinkelDifferenz ( n , sensors );
		*sensors *= factorSensors;
		//		*sensors = 3*tanh((1/3) * (*sensors));  // keep sensorvalues in the range [-3,3]
		sensors++;
	}
	
	return getSensorfeldGroesse (); //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden
}



/** The list of all parameters with there value as allocated lists.
    @param keylist,vallist will be allocated with malloc (free it after use!)
    @return length of the lists
*/
int SchlangeForce::getParamList(paramkey*& keylist,paramval*& vallist) const{
  int number_params=4; // don't forget to adapt number params!
  keylist=(paramkey*)malloc(sizeof(paramkey)*number_params);
  vallist=(paramval*)malloc(sizeof(paramval)*number_params);
  keylist[0]="gamma";
  keylist[1]="frictionGround";  
  keylist[2]="factorForce";  
  keylist[3]="factorSensors";  

  vallist[0]=gamma;
  vallist[1]=frictionGround;
  vallist[2]=factorForce;
  vallist[3]=factorSensors;
  return number_params;
}


paramval SchlangeForce::getParam(paramkey key) const{
  if(!key) return 0.0;
  if(strcmp(key, "gamma")==0) return gamma; 
  else if(strcmp(key, "frictionGround")==0) return frictionGround; 
  else if(strcmp(key, "factorForce")==0)    return factorForce;  	
  else if(strcmp(key, "factorSensors")==0)    return factorSensors;  	
  else  return Configurable::getParam(key) ;
}

bool SchlangeForce::setParam(paramkey key, paramval val){
  if(!key) {
    fprintf(stderr, "%s: empty Key!\n", __FILE__);
    return false;
  }
  if(strcmp(key, "gamma")==0) gamma=val;
  else if(strcmp(key, "frictionGround")==0) frictionGround=val; 
  else if(strcmp(key, "factorForce")==0)    factorForce=val;
  else if(strcmp(key, "factorSensors")==0)    factorSensors=val;
  else return Configurable::setParam(key, val);
  return true;
}
