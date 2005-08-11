/************************************************************************/
/*shpererobot.cpp							*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

#include "sphererobot.h"
#include <iostream>

/**
 *constructor
 *@param startRoboterID ID, which should be managed clearly

 *@author Marcel Kretschmann
 *@version beta
 **/
Sphererobot::Sphererobot ( int startRoboterID , const ODEHandle& odeHandle, 
		     const SphererobotConf& conf )
	: Roboter ( startRoboterID , odeHandle.world , odeHandle.space , odeHandle.jointGroup , 3 )
{
  sphererobot_space = dSimpleSpaceCreate ( space );
  dSpaceSetCleanup ( sphererobot_space , 0 );
  
  Object tmp_body , tmp_body2 , tmp_body3 , tmp_body4 , tmp_body5;
  this->conf = conf;
	
  dMass mass, mass2, mass3, mass4, mass5;

  //*************body definition**************
	
  //sphere base body
  tmp_body.body = dBodyCreate ( world );
  objektliste.push_back ( tmp_body );	

  dBodySetPosition ( (objektliste.back ()).body , 0 , 0 , conf.diameter/2 );
  dMassSetSphereTotal ( &mass , conf.spheremass , conf.diameter );
  dBodySetMass ( (objektliste.back ()).body , &mass );

  (objektliste.back ()).geom = dCreateSphere ( sphererobot_space , conf.diameter );
  dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );
      
  //pendular body
  tmp_body2.body = dBodyCreate ( world );
  objektliste.push_back ( tmp_body2 );

  dBodySetPosition ( (objektliste.back ()).body , getPosition ().x , getPosition ().y , getPosition ().z + conf.diameter/4 );
  dMassSetSphereTotal ( &mass2 , conf.pendularmass , conf.pendulardiameter );
  dBodySetMass ( (objektliste.back ()).body , &mass2 );
	
  (objektliste.back ()).geom = dCreateSphere ( sphererobot_space , conf.pendulardiameter );
  dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );
  
  
  //3 conection bodies between the pendular an the sphere
  tmp_body3.body = dBodyCreate ( world );
  objektliste.push_back ( tmp_body3 );

  dBodySetPosition ( (objektliste.back ()).body , getPosition ().x , getPosition ().y , getPosition ().z + conf.diameter/4 );
  dMassSetBoxTotal ( &mass3 , conf.pendularmass/1000 , 0.2 , 0.2 , 1 );
  dBodySetMass ( (objektliste.back ()).body , &mass3 );
	
  //(objektliste.back ()).geom = dCreateSphere ( sphererobot_space , conf.pendulardiameter );
  //dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );
  
  
  tmp_body4.body = dBodyCreate ( world );
  objektliste.push_back ( tmp_body4 );

  dBodySetPosition ( (objektliste.back ()).body , getPosition ().x , getPosition ().y , getPosition ().z + conf.diameter/4 );
  dMassSetBoxTotal ( &mass4 , conf.pendularmass/1000 , 0.2 , 0.2 , 1 );
  dBodySetMass ( (objektliste.back ()).body , &mass4 );
	
  //(objektliste.back ()).geom = dCreateSphere ( sphererobot_space , conf.pendulardiameter );
  //dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );
  
  tmp_body5.body = dBodyCreate ( world );
  objektliste.push_back ( tmp_body5 );

  dBodySetPosition ( (objektliste.back ()).body , getPosition ().x , getPosition ().y , getPosition ().z + conf.diameter/4 );
  dMassSetBoxTotal ( &mass5 , conf.pendularmass/1000 , 0.2 , 0.2 , 1 );
  dBodySetMass ( (objektliste.back ()).body , &mass5 );
	
  //(objektliste.back ()).geom = dCreateSphere ( sphererobot_space , conf.pendulardiameter );
  //dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );

  //*****************joint definition***********
  dJointID tmpj1 , tmpj2 , tmpj3;
    
  tmpj1 = dJointCreateBall ( world , 0 );
  dJointAttach ( tmpj1 , getObjektAt ( 0 ).body , getObjektAt ( 2 ).body );
  dJointSetUniversalAnchor ( tmpj1 , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 0 ).body , 3 ) );
  
  tmpj2 = dJointCreateBall ( world , 0 );
  dJointAttach ( tmpj2 , getObjektAt ( 0 ).body , getObjektAt ( 3 ).body );
  dJointSetUniversalAnchor ( tmpj2 , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 0 ).body , 3 ) );
  
  tmpj3 = dJointCreateBall ( world , 0 );
  dJointAttach ( tmpj3 , getObjektAt ( 0 ).body , getObjektAt ( 4 ).body );
  dJointSetUniversalAnchor ( tmpj3 , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 0 ).body , 3 ) );
  
  
  
  jointliste.push_back ( dJointCreateSlider ( world , 0 ) );
  dJointAttach ( jointliste.back () , getObjektAt ( 2 ).body , getObjektAt ( 1 ).body );
  dJointSetSliderAxis ( jointliste.back() , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 1 ) - dBodyGetPositionAll ( getObjektAt ( 1 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 2 ) - dBodyGetPositionAll ( getObjektAt ( 1 ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 3 ) - dBodyGetPositionAll ( getObjektAt ( 1 ).body , 3 ) );
  
  jointliste.push_back ( dJointCreateSlider ( world , 0 ) );
  dJointAttach ( jointliste.back () , getObjektAt ( 3 ).body , getObjektAt ( 1 ).body );
  dJointSetSliderAxis ( jointliste.back() , dBodyGetPositionAll ( getObjektAt ( 3 ).body , 1 ) - dBodyGetPositionAll ( getObjektAt ( 1 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 3 ).body , 2 ) - dBodyGetPositionAll ( getObjektAt ( 1 ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 3 ).body , 3 ) - dBodyGetPositionAll ( getObjektAt ( 1 ).body , 3 ) );
  
  jointliste.push_back ( dJointCreateSlider ( world , 0 ) );
  dJointAttach ( jointliste.back () , getObjektAt ( 4 ).body , getObjektAt ( 1 ).body );
  dJointSetSliderAxis ( jointliste.back() , dBodyGetPositionAll ( getObjektAt ( 4 ).body , 1 ) - dBodyGetPositionAll ( getObjektAt ( 1 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 4 ).body , 2 ) - dBodyGetPositionAll ( getObjektAt ( 1 ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 4 ).body , 3 ) - dBodyGetPositionAll ( getObjektAt ( 1 ).body , 3 ) );
  
/*    jointliste.push_back ( dJointCreateUniversal ( world , 0 ) );
		
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
      dJointSetUniversalParam ( jointliste.back () , dParamBounce2, 0.9 );*/

}
	
/**
 *Destruktor
 *@author Marcel Kretschmann
 *@version beta
 **/
Sphererobot::~Sphererobot()
{
	dSpaceDestroy ( sphererobot_space );
}

/**
 *Draws all elements of the snake.
 *@author Marcel Kretschmann
 *@version beta
 **/
void Sphererobot::draw()
{
  dsSetTexture (DS_WOOD);
  dsSetColor ( color.r , color.g , color.b );

  
  dsDrawSphere ( dGeomGetPosition ( getObjektAt ( 0 ).geom ) , dGeomGetRotation ( getObjektAt ( 0 ).geom ) , conf.diameter );
  dsDrawSphere ( dGeomGetPosition ( getObjektAt ( 1 ).geom ) , dGeomGetRotation ( getObjektAt ( 1 ).geom ) , conf.pendulardiameter );

}

/**Sets the snake to position pos, sets color to c, and creates snake if necessary.
 *This overwrides the function place of the class robot.
 *@param pos desired position of the snake in struct Position
 *@param c desired color for the snake in struct Color (might be NULL!)
 *@author Marcel Kretschmann
 *@version beta
 **/
void Sphererobot::place (Position pos, Color *c)
{
  pos.z = max ( conf.diameter, pos.z );
    
  dBodySetPosition ( getObjektAt(0).body , pos.x , pos.y , pos.z );
  	
  if(c)
    color = (*c);
}

void Sphererobot::mycallback(void *data, dGeomID o1, dGeomID o2)
{
dsPrint ( "TEST\n" );
  // internal collisions
  /*Sphererobot* me = (Sphererobot*)data;  
  int i,n;  
  const int N = 10;
  dContact contact[N];  
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  for (i=0; i<n; i++){
    contact[i].surface.mode = 0;
    contact[i].surface.mu = 0;
    contact[i].surface.mu2 = 0;
//     contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
//       dContactSoftERP | dContactSoftCFM | dContactApprox1;
//     contact[i].surface.mu = 0.0;
//     contact[i].surface.slip1 = 0.005;
//     contact[i].surface.slip2 = 0.005;
//     contact[i].surface.soft_erp = 1;
//     contact[i].surface.soft_cfm = 0.00001;
    dJointID c = dJointCreateContact( me->world, me->contactgroup, &contact[i]);
    dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	      
  }*/
}

/**
 *This is the collision handling function for sphere robots.
 *This overwrides the function collisionCallback of the class robot.
 *@param data
 *@param o1 first geometrical object, which has taken part in the collision
 *@param o2 second geometrical object, which has taken part in the collision
 *@return true if the collision was threated  by the robot, false if not
 *@author Marcel Kretschmann
 *@version beta
 **/
bool Sphererobot::collisionCallback(void *data, dGeomID o1, dGeomID o2)
{
  //checks if one of the collision objects is part of the robot
  if( o1 == (dGeomID)sphererobot_space && o2 == (dGeomID)sphererobot_space)
  {
    // mycallback is called for internal collisions!
    dSpaceCollide(sphererobot_space, this, mycallback);
    return true;
  }
  else
  {
	// the rest is for collisions of some snake elements with the rest of the world
	int i,n;  
	const int N = 10;
	dContact contact[N];
	
	n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	for (i=0; i<n; i++)
	{
		contact[i].surface.mode = 0;
		contact[i].surface.mu = 0.2;
		contact[i].surface.mu2 = 0;
	// 	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	// 	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
	// 	contact[i].surface.mu = frictionGround;
	// 	contact[i].surface.slip1 = 0.005;
	// 	contact[i].surface.slip2 = 0.005;
	// 	contact[i].surface.soft_erp = 1;
	// 	contact[i].surface.soft_cfm = 0.00001;
		dJointID c = dJointCreateContact( world, contactgroup, &contact[i]);
		dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	      
	}
	return true;
    }
    return false;
}

/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the array
 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 *@author Marcel Kretschmann
 *@version beta
 **/
int Sphererobot::getSensors ( sensor* sensors, int sensornumber )
{
  sensoraktualisierung ();
  for ( int n = 0; n < sensornumber; n++ )
    {
      /*if ( conf.ausgabeArt == angle )
	(*sensors++) = sensorfeld[n].istwinkel/(2*M_PI);
      if ( conf.ausgabeArt == anglerate )
	getWinkelDifferenz ( n , sensors++ );*/
			
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
void Sphererobot::setMotors ( const motor* motors, int motornumber )
{
  /*for ( int n = 0; n < motornumber; n++ )
    if ( n % 2 == 0 )
      {
	dJointSetUniversalParam ( getJointAt(n/2) , dParamVel , *(motors++)*conf.factorForce );
	dJointSetUniversalParam ( jointliste[n/2] , dParamFMax , conf.maxMotorKraft );
      }
    else
      {
	dJointSetUniversalParam ( jointliste[n/2] , dParamVel2 , *(motors++)*conf.factorForce );
	dJointSetUniversalParam ( jointliste[n/2] , dParamFMax2 , conf.maxMotorKraft );
      }*/
}	

/**
 *Returns the number of motors used by the snake.
 *@return number of motors
 *@author Marcel Kretschmann
 *@version final
 **/
int Sphererobot::getMotorNumber()
{
  return getJointAnzahl ();
}
	
/**
 *Updates the sensorarray.
 *This overwrides the function sensoraktualisierung of the class robot
 *@author Marcel Kretschmann
 *@version beta
 **/
void Sphererobot::sensoraktualisierung ( )
{
  /*for ( int n = 0; n < getSensorfeldGroesse (); n++ )
    {
      sensorfeld[n].istwinkel_alt = sensorfeld[n].istwinkel;
		
      if ( n % 2 == 0 )
	sensorfeld[n].istwinkel = dJointGetUniversalAngle1 ( getJointAt (n/2) );
      else
	sensorfeld[n].istwinkel = dJointGetUniversalAngle2 ( getJointAt (n/2) );
    }*/
}

/**
 *Returns the position of the snake. Here the position of the snake is the position of the first element of the snake.
 *@return Position (x,y,z)
 *@author Marcel Kretschmann
 *@version final
 **/
Position Sphererobot::getPosition ()
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
 *Prints some internal robot parameters. Actualy it prints all sensor data of one callculation step.
 *@author Marcel Kretschmann
 *@version beta
 **/
void Sphererobot::getStatus ()
{
  for ( int n = 0; n < getSensorfeldGroesse (); dsPrint ( "Sensor %i: %lf\n" , n , sensorfeld[n++].istwinkel ) );
}
