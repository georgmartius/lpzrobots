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
 *@param start_ausgabeart angle: sensor values are the angle of the joints; anglerate: sensor values are the angle rates of the joints
 *@author Marcel Kretschmann
 *@version beta
 **/


SchlangeForce::SchlangeForce ( int startRoboterID , dWorldID* welt , dSpaceID* raum , dJointGroupID* start_contactgroup , int start_Sensoranzahl , double start_x , double start_y , double start_z , int armanzahl , double glieder_laenge , double glieder_durchmesser , double glieder_abstand , double glieder_masse , double start_maxmotorkraft , double start_geschwindigkeitsfaktor , ausgabemodus start_ausgabeart ) :
Schlange::Schlange ( startRoboterID , welt , raum , start_contactgroup , start_Sensoranzahl , start_x , start_y , start_z , armanzahl , glieder_laenge , glieder_durchmesser , glieder_abstand , glieder_masse , start_maxmotorkraft , start_geschwindigkeitsfaktor , start_ausgabeart ) 
{
        // prepare name;
        Configurable::insertCVSInfo(name, "$RCSfile$", 
		           	            "$Revision$");
	gamma=0.4;
	friction_ground=0.1;
	factor_force=4.0;
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
 *Reads the actual motor commands from an array, an sets all motors (forces) of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 **/
void SchlangeForce::setMotors ( const motor* motors, int motornumber )
{

  // controller output as torques 
  for (int i=0; i<motornumber/2; i++){
    dJointAddUniversalTorques( jointliste[i],factor_force* motors[2*i],
			       factor_force*motors[2*i +1]);  // motorcommand
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


/** The list of all parameters with there value as allocated lists.
    @param keylist,vallist will be allocated with malloc (free it after use!)
    @return length of the lists
*/
int SchlangeForce::getParamList(paramkey*& keylist,paramval*& vallist) const{
  int number_params=3; // don't forget to adapt number params!
  keylist=(paramkey*)malloc(sizeof(paramkey)*number_params);
  vallist=(paramval*)malloc(sizeof(paramval)*number_params);
  keylist[0]="gamma";
  keylist[1]="friction_ground";  
  keylist[2]="factor_force";  

  vallist[0]=gamma;
  vallist[1]=friction_ground;
  vallist[2]=factor_force;
  return number_params;
}


paramval SchlangeForce::getParam(paramkey key) const{
  if(!key) return 0.0;
  if(strcmp(key, "gamma")==0) return gamma; 
  else if(strcmp(key, "friction_ground")==0) return friction_ground; 
  else if(strcmp(key, "factor_force")==0)    return factor_force;  	
  else  return Configurable::getParam(key) ;
}

bool SchlangeForce::setParam(paramkey key, paramval val){
  if(!key) {
    fprintf(stderr, "%s: empty Key!\n", __FILE__);
    return false;
  }
  if(strcmp(key, "gamma")==0) gamma=val;
  else if(strcmp(key, "friction_ground")==0) friction_ground=val; 
  else if(strcmp(key, "factor_force")==0)    factor_force=val;
  else return Configurable::setParam(key, val);
  return true;
}
