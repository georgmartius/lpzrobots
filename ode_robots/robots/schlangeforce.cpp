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
 *@author Marcel Kretschmann
 *@version beta
 **/
SchlangeForce::SchlangeForce ( int startRoboterID , const OdeHandle& odeHandle, 
			       const SchlangenConf& conf ) 
  :Schlange ( startRoboterID , odeHandle, conf ) 
{
        // prepare name;
        Configurable::insertCVSInfo(name, "$RCSfile$", 
		           	            "$Revision$");
	gamma=0.4;
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
 void SchlangeForce::doInternalStuff(const GlobalData& global){}
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
					contact[i].surface.mu = conf.frictionGround;
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
    dJointAddUniversalTorques( jointliste[i],conf.factorForce* motors[2*i],
			       conf.factorForce*motors[2*i +1]);  // motorcommand
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
		if ( conf.ausgabeArt == angle )
			*sensors = sensorfeld[n].istwinkel/(2*M_PI);
		if ( conf.ausgabeArt == anglerate )
			getWinkelDifferenz ( n , sensors );
		*sensors *= conf.factorSensors;
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
  vallist[1]=conf.frictionGround;
  vallist[2]=conf.factorForce;
  vallist[3]=conf.factorSensors;
  return number_params;
}


paramval SchlangeForce::getParam(paramkey key) const{
  if(!key) return 0.0;
  if(strcmp(key, "gamma")==0) return gamma; 
  else if(strcmp(key, "frictionGround")==0) return conf.frictionGround; 
  else if(strcmp(key, "factorForce")==0)    return conf.factorForce;  	
  else if(strcmp(key, "factorSensors")==0)  return conf.factorSensors;  	
  else  return Configurable::getParam(key) ;
}

bool SchlangeForce::setParam(paramkey key, paramval val){
  if(!key) {
    fprintf(stderr, "%s: empty Key!\n", __FILE__);
    return false;
  }
  if(strcmp(key, "gamma")==0) gamma=val;
  else if(strcmp(key, "frictionGround")==0) conf.frictionGround = val; 
  else if(strcmp(key, "factorForce")==0)    conf.factorForce    = val;
  else if(strcmp(key, "factorSensors")==0)  conf.factorSensors  = val;
  else return Configurable::setParam(key, val);
  return true;
}
