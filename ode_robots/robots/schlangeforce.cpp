/************************************************************************/
/*schlangeforce.cpp								*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

#include "schlangeforce.h"
#include "abstractcontroller.h"


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
SchlangeForce::SchlangeForce ( int startRoboterID , dWorldID* welt , dSpaceID* raum , dJointGroupID* start_contactgroup , int start_Sensoranzahl , double start_x , double start_y , double start_z , int armanzahl , double glieder_laenge , double glieder_durchmesser , double glieder_abstand , double glieder_masse , double start_maxmotorkraft , double start_geschwindigkeitsfaktor ) :
Roboter::Roboter ( startRoboterID , welt , raum , start_contactgroup , start_Sensoranzahl )
{
        // prepare name;
        Configurable::insertCVSInfo(name, "$RCSfile$", 
		           	            "$Revision$");


	Object tmp_body;
	
	dMass masse;
	dMatrix3 R;


	gliederdurchmesser = glieder_durchmesser;
	gliederlaenge = glieder_laenge;
	schlangenarmanzahl = armanzahl;
	geschwindigkeitsfaktor = start_geschwindigkeitsfaktor;
	maxmotorkraft = start_maxmotorkraft;

	gamma=0.4;
	friction_ground=0.1;
	factor_force=4.0;
	
	//standard color of all snakes is green, if no other value is set by calling the function place
	color.r = 0;
	color.g = 1;
	color.b = 0;	

	//*************body definition**************
	
	dMassSetCappedCylinderTotal ( &masse , glieder_masse , 2 , glieder_laenge , glieder_durchmesser );
	
	dRFromAxisAndAngle ( R , 0 , 1 , 0 , M_PI/2 );//rotation of the matrix R by 90°

	for ( int n = 0; n < armanzahl; n++ )
	{
		tmp_body.body = dBodyCreate ( *world );
		objektliste.push_back ( tmp_body );
		
	
		dBodySetPosition ( (objektliste.back ()).body , start_x + (n + 0.5 )*glieder_laenge + n * glieder_abstand, start_y ,  start_z );

		dBodySetMass ( (objektliste.back ()).body , &masse );
	
		(objektliste.back ()).geom = dCreateCCylinder ( *space , glieder_durchmesser , glieder_laenge );
		dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );

		dGeomSetRotation ( (objektliste.back ()).geom , R );//includes rotation of the body
	}

	//*****************joint definition***********
	for ( int n = 0; n < armanzahl-1; n++ )
	{
		jointliste.push_back ( dJointCreateUniversal ( *world , 0 ) );
		
		dJointAttach ( jointliste.back () , objektliste[n].body , objektliste[n+1].body );
			
		dJointSetUniversalAnchor ( jointliste.back () , dBodyGetPositionAll ( objektliste[n].body , 1 ) + ( dBodyGetPositionAll ( objektliste[n+1].body , 1 ) - dBodyGetPositionAll ( objektliste[n].body , 1 ) )/2 , dBodyGetPositionAll ( objektliste[n].body , 2 ) + ( dBodyGetPositionAll ( objektliste[n+1].body , 2 ) - dBodyGetPositionAll ( objektliste[n].body , 2 ) )/2 , dBodyGetPositionAll ( objektliste[n].body , 3 ) );

		dJointSetUniversalAxis1 ( jointliste.back () , 0 , 1 , 0 );
		dJointSetUniversalAxis2 ( jointliste.back () , 0 , 0 , 1 );
	}	

	//starting sensor values
 	for ( int n = 0; n < 2*(armanzahl-1); n++ )
	{
		sensorfeld[n].sollwinkel = 0;
		sensorfeld[n].istwinkel_alt = sensorfeld[n].istwinkel;
	}

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
 *Zeichnet die Koerper-GeometrieObjekte.
 *@author Marcel Kretschmann
 *@version beta
 **/
void SchlangeForce::draw()
{
	double box [3];
	dsSetTexture (DS_WOOD);
	dsSetColor ( color.r , color.g , color.b );

	box[0] = gliederlaenge/10; box[1] = gliederdurchmesser/10; box[2] = gliederdurchmesser/10;
	for ( int n = 0; n < schlangenarmanzahl; n++ )
	{
			dsDrawCappedCylinder ( dGeomGetPosition ( getObjektAt ( n ).geom ) , dGeomGetRotation ( getObjektAt ( n ).geom ) , gliederlaenge , gliederdurchmesser );
	}
}
	
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
bool SchlangeForce::kollisionsermittlung ( dGeomID o1 , dGeomID o2 )
{
	for ( unsigned int n = 0; n < objektliste.size (); n++ )
	{
		if 
		(
		( getObjektAt ( n ).geom == o1 && getObjektAt ( n + 1 ).geom == o2 ) || ( getObjektAt ( n ).geom == o2 && getObjektAt ( n + 1 ).geom == o1 )		
		)
			return true;
	}
	
	return false;
}
	
/**Sets the snake to position pos, sets color to c, and creates snake if necessary.
 *This overwrides the function place of the class robot.
 *@param pos desired position of the snake in struct Position
 *@param c desired color for the snake in struct Color
 *@author Marcel Kretschmann
 *@version beta
 **/
void SchlangeForce::place (Position pos, Color *c)
{
	double dx , dy , dz;

	dx = pos.x - getPosition ().x;
	dy = pos.y - getPosition ().y;
	dz = pos.z - getPosition ().z;
	
	for ( int n = 0; n < getObjektAnzahl (); n++ )
		dBodySetPosition ( getObjektAt(n).body , getPosition ( n ).x + dx , getPosition ( n ).y + pos.y ,getPosition ( n ).z +  pos.z );
	
	color.r = (*c).r;
	color.g = (*c).g;
	color.b = (*c).b;
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
					contact[i].surface.mu = friction_ground;
					contact[i].surface.slip1 = 0.005;
					contact[i].surface.slip2 = 0.005;
					contact[i].surface.soft_erp = 1;
					contact[i].surface.soft_cfm = 0.00001;

					dJointID c = dJointCreateContact ( (*world) , (*contactgroup) , &contact[i] );
					dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
				}
		}
		return true; //if collision was threated by this robot
	}
	else return false; //if collision was not threated by this robot
}



/**
 *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 *@author Marcel Kretschmann
 *@version beta
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

/**
 *Returns the number of motors used by the snake.
 *@return number of motors
 *@author Marcel Kretschmann
 *@version final
 **/
 int SchlangeForce::getMotorNumber()
{
	return 2*getJointAnzahl ();
}
	
/**
 *Updates the sensorarray.
 *This overwrides the function sensoraktualisierung of the class robot
 *@author Marcel Kretschmann
 *@version beta
 **/
void SchlangeForce::sensoraktualisierung ( )
{
	for ( int n = 0; n < getSensorfeldGroesse (); n++ )
	{
		sensorfeld[n].istwinkel_alt = sensorfeld[n].istwinkel;
		
		if ( n % 2 == 0 )
			sensorfeld[n].istwinkel = dJointGetUniversalAngle1 ( getJointAt (n/2) );
		else
			sensorfeld[n].istwinkel = dJointGetUniversalAngle2 ( getJointAt (n/2) );
	}
}

/**
 *Returns the position of the snake. Here the position of the snake is the position of the first element of the snake.
 *@return Position (x,y,z)
 *@author Marcel Kretschmann
 *@version final
 **/
Position SchlangeForce::getPosition ()
{
	const dReal* tmpPos;
	Position returnPos;
	tmpPos = dBodyGetPosition ( getObjektAt(0).body );
	returnPos.x = tmpPos[0];
	returnPos.y = tmpPos[1];
	returnPos.z = tmpPos[2];

	return returnPos;
}

/**
 *Returns the position of one element of the snake.
 @param n number of the snake element
 *@return Position (x,y,z)
 *@author Marcel Kretschmann
 *@version final
 **/
Position SchlangeForce::getPosition ( int n )
{
	const dReal* tmpPos;
	Position returnPos;
	tmpPos = dBodyGetPosition ( getObjektAt (n).body );
	returnPos.x = tmpPos[0];
	returnPos.y = tmpPos[1];
	returnPos.z = tmpPos[2];

	return returnPos;
}

/**
 *Prints some internal robot parameters. Actualy it prints all sensor data of one callculation step.
 *@author Marcel Kretschmann
 *@version beta
 **/
void SchlangeForce::getStatus ()
{
	for ( int n = 0; n < getSensorfeldGroesse (); dsPrint ( "Sensor %i: %lf\n" , n , sensorfeld[n++].istwinkel ) );
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
