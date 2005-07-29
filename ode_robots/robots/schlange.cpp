/************************************************************************/
/*schlange.cpp								*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

#include "schlange.h"
#include <iostream>

/**
 *constructor
 *@param startRoboterID ID, which should be managed clearly

 *@author Marcel Kretschmann
 *@version beta
 **/
Schlange::Schlange ( int startRoboterID , const ODEHandle& odeHandle, 
		     const SchlangenConf& conf ) 
  : Roboter ( startRoboterID , odeHandle.world , odeHandle.space , odeHandle.jointGroup , 2*(conf.armAnzahl-1) )
{
  Object tmp_body;
  this->conf = conf;
	
  dMass masse;
  dMatrix3 R;	

  //*************body definition**************
	
  dMassSetCappedCylinderTotal ( &masse , conf.gliederMasse , 2 , conf.gliederLaenge , conf.gliederDurchmesser );
	
  dRFromAxisAndAngle ( R , 0 , 1 , 0 , M_PI/2 );//rotation of the matrix R by 90°

  for ( int n = 0; n < conf.armAnzahl; n++ )
    {
      tmp_body.body = dBodyCreate ( world );
      objektliste.push_back ( tmp_body );
		
	
      dBodySetPosition ( (objektliste.back ()).body , 
			 (n + 0.5 )*conf.gliederLaenge + n * conf.gliederAbstand, 0 , conf.gliederDurchmesser/2 );

      dBodySetMass ( (objektliste.back ()).body , &masse );
	
      (objektliste.back ()).geom = dCreateCCylinder ( space , conf.gliederDurchmesser , conf.gliederLaenge );
      dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );

      dGeomSetRotation ( (objektliste.back ()).geom , R );//includes rotation of the body
    }

  //*****************joint definition***********
  for ( int n = 0; n < conf.armAnzahl-1; n++ )
    {
      jointliste.push_back ( dJointCreateUniversal ( world , 0 ) );
		
      dJointAttach ( jointliste.back () , objektliste[n].body , objektliste[n+1].body );
			
      dJointSetUniversalAnchor ( jointliste.back () , dBodyGetPositionAll ( objektliste[n].body , 1 ) + ( dBodyGetPositionAll ( objektliste[n+1].body , 1 ) - dBodyGetPositionAll ( objektliste[n].body , 1 ) )/2 , dBodyGetPositionAll ( objektliste[n].body , 2 ) + ( dBodyGetPositionAll ( objektliste[n+1].body , 2 ) - dBodyGetPositionAll ( objektliste[n].body , 2 ) )/2 , dBodyGetPositionAll ( objektliste[n].body , 3 ) );

      dJointSetUniversalAxis1 ( jointliste.back () , 0 , 1 , 0 );
      dJointSetUniversalAxis2 ( jointliste.back () , 0 , 0 , 1 );

      // setting stops at universal joints		
      dJointSetUniversalParam ( jointliste.back () , dParamLoStop, -conf.maxWinkel );
      dJointSetUniversalParam ( jointliste.back () , dParamHiStop,  conf.maxWinkel );
      dJointSetUniversalParam ( jointliste.back () , dParamLoStop2,-conf.maxWinkel); 
      dJointSetUniversalParam ( jointliste.back () , dParamHiStop2, conf.maxWinkel); 

      // making stops bouncy
      dJointSetUniversalParam ( jointliste.back () , dParamBounce, 0.9 );
      dJointSetUniversalParam ( jointliste.back () , dParamBounce2, 0.9 );
    }	
}
	
/**
 *Destruktor
 *@author Marcel Kretschmann
 *@version beta
 **/
Schlange::~Schlange()
{
}

/** fix segment 0 in the sky
 */
void Schlange::fixInSky(){
  for (int i=0; i<2; i++){
    skyJoints.push_back( dJointCreateHinge ( world , 0 ) );
    dJointAttach ( skyJoints.back(), objektliste[0].body , 0 );
    dJointSetUniversalAnchor ( skyJoints.back(), 
			       dBodyGetPositionAll ( objektliste[0].body , 1 ) , 
			       dBodyGetPositionAll ( objektliste[0].body , 2 ) , 
			       dBodyGetPositionAll ( objektliste[0].body , 3 ) ); 
    if (i==0) dJointSetHingeAxis(skyJoints.back(),1,0,0);
    if (i==1) dJointSetHingeAxis(skyJoints.back(),0,1,0);
    dJointSetFixed(skyJoints.back());
  }
  /*
    jointliste.push_back( dJointCreateHinge ( world , 0 ) );
    dJointAttach ( jointliste.back() , objektliste[0].body , 0 );
    dJointSetUniversalAnchor ( jointliste.back() , 
    dBodyGetPositionAll ( objektliste[0].body , 1 ) , 
    dBodyGetPositionAll ( objektliste[0].body , 2 ) , 
    dBodyGetPositionAll ( objektliste[0].body , 3 ) ); 
    dJointSetHingeAxis(jointliste.back(),0,1,0);
    dJointSetFixed(jointliste.back());
  */
};

	
/**
 *Draws all elements of the snake.
 *@author Marcel Kretschmann
 *@version beta
 **/
void Schlange::draw()
{
  double box [3];
  dsSetTexture (DS_WOOD);
  dsSetColor ( color.r , color.g , color.b );

  box[0] = conf.gliederLaenge/10; box[1] = conf.gliederDurchmesser/10; box[2] = conf.gliederDurchmesser/10;
  for ( int n = 0; n < conf.armAnzahl; n++ )
    {
      dsDrawCappedCylinder ( dGeomGetPosition ( getObjektAt ( n ).geom ) , 
			     dGeomGetRotation ( getObjektAt ( n ).geom ) , 
			     conf.gliederLaenge , conf.gliederDurchmesser );
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
bool Schlange::kollisionsermittlung ( dGeomID o1 , dGeomID o2 )
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
 *@param c desired color for the snake in struct Color (might be NULL!)
 *@author Marcel Kretschmann
 *@version beta
 **/
void Schlange::place (Position pos, Color *c)
{
  pos.z = max(conf.gliederDurchmesser/2, pos.z);
  double dx , dy , dz;
  dx = pos.x - getPosition ().x;
  dy = pos.y - getPosition ().y;
  dz = pos.z - getPosition ().z;
  
  for ( int n = 0; n < getObjektAnzahl (); n++ )
    dBodySetPosition ( getObjektAt(n).body , getPosition ( n ).x + dx , getPosition ( n ).y + pos.y ,getPosition ( n ).z +  pos.z );
	
  if(c)
    color = (*c);
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
bool Schlange::collisionCallback(void *data, dGeomID o1, dGeomID o2)
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
      if ( Schlange::kollisionsermittlung ( o1 , o2 ) == true )
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

		dJointID c = dJointCreateContact ( world , contactgroup , &contact[i] );
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
int Schlange::getSensors ( sensor* sensors, int sensornumber )
{
  sensoraktualisierung ();
  for ( int n = 0; n < sensornumber; n++ )
    {
      if ( conf.ausgabeArt == angle )
	(*sensors++) = sensorfeld[n].istwinkel/(2*M_PI);
      if ( conf.ausgabeArt == anglerate )
	getWinkelDifferenz ( n , sensors++ );
			
    }
	
  return getSensorfeldGroesse (); //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden
}


/**
 *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 *@author Marcel Kretschmann
 *@version beta
 **/
void Schlange::setMotors ( const motor* motors, int motornumber )
{
  for ( int n = 0; n < motornumber; n++ )
    if ( n % 2 == 0 )
      {
	dJointSetUniversalParam ( getJointAt(n/2) , dParamVel , *(motors++)*conf.factorForce );
	dJointSetUniversalParam ( jointliste[n/2] , dParamFMax , conf.maxMotorKraft );
      }
    else
      {
	dJointSetUniversalParam ( jointliste[n/2] , dParamVel2 , *(motors++)*conf.factorForce );
	dJointSetUniversalParam ( jointliste[n/2] , dParamFMax2 , conf.maxMotorKraft );
      }
}	

/**
 *Returns the number of motors used by the snake.
 *@return number of motors
 *@author Marcel Kretschmann
 *@version final
 **/
int Schlange::getMotorNumber()
{
  return 2*getJointAnzahl ();
}
	
/**
 *Updates the sensorarray.
 *This overwrides the function sensoraktualisierung of the class robot
 *@author Marcel Kretschmann
 *@version beta
 **/
void Schlange::sensoraktualisierung ( )
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
Position Schlange::getPosition ()
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
Position Schlange::getPosition ( int n )
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
void Schlange::getStatus ()
{
  for ( int n = 0; n < getSensorfeldGroesse (); dsPrint ( "Sensor %i: %lf\n" , n , sensorfeld[n++].istwinkel ) );
}
