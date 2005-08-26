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
  
  Object tmp_body , tmp_body2 , tmp_body3;
  this->conf = conf;
	
  dMass mass, mass2, mass3, mass4, mass5;

  //*************body definition**************
	
  //sphere base body
  tmp_body.body = dBodyCreate ( world );
  objektliste.push_back ( tmp_body );	

  dBodySetPosition ( (objektliste.back ()).body , 0 , 0 , conf.diameter/2 );
  dMassSetSphereTotal ( &mass , conf.spheremass , conf.diameter/2 );
  dBodySetMass ( (objektliste.back ()).body , &mass );

  (objektliste.back ()).geom = dCreateSphere ( sphererobot_space , conf.diameter/2 );
  dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );
      
  //pendular body
  tmp_body2.body = dBodyCreate ( world );
  objektliste.push_back ( tmp_body2 );

  dBodySetPosition ( (objektliste.back ()).body , getPosition ().x , getPosition ().y , getPosition ().z - conf.diameter/4 );
  dMassSetSphereTotal ( &mass2 , conf.pendularmass , conf.pendulardiameter/2 );
  dBodySetMass ( (objektliste.back ()).body , &mass2 );
	
  
  (objektliste.back ()).geom = dCreateSphere ( sphererobot_space , conf.pendulardiameter/2 );
  //(objektliste.back ()).geom = dCreateBox ( sphererobot_space , 0.8,0.8,0.8);
  dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );
  
  
  //first and second 3 conection bodies between the pendular an the sphere
  double x , y;
  for ( int alpha = 1; alpha < 4; alpha++ )
  {
  	x=sin ( (float) alpha*2*M_PI/3 )*conf.diameter/4; //testing values
  	y=cos ( (float) alpha*2*M_PI/3 )*conf.diameter/4;
  	
  	tmp_body3.body = dBodyCreate ( world );
  	objektliste.push_back ( tmp_body3 );
  	dBodySetPosition ( (objektliste.back ()).body , getPosition ().x + x , getPosition ().y + y , getPosition ().z - conf.diameter/3 );
  	dMassSetBoxTotal ( &mass3 , conf.slidermass*2/3 , 0.2 , 0.2 , 1 );
  	dBodySetMass ( (objektliste.back ()).body , &mass3 );
	
	tmp_body3.body = dBodyCreate ( world );
  	objektliste.push_back ( tmp_body3 );
	dBodySetPosition ( (objektliste.back ()).body , getPosition ().x + x , getPosition ().y + y , dBodyGetPositionAll ( objektliste[1].body , 3 ) );
  	dMassSetBoxTotal ( &mass3 , conf.slidermass*1/3 , 0.2 , 0.2 , 0.2 );
  	dBodySetMass ( (objektliste.back ()).body , &mass3 );
	
	//combines the 3 upper connection bodies with the pendular
	dJointID tmp = dJointCreateHinge ( world , 0 );
	dJointAttach ( tmp , getObjektAt ( 1 ).body , getObjektAt ( 2*alpha+1 ).body );
	
	dJointSetHingeAnchor ( tmp , dBodyGetPositionAll ( getObjektAt ( 2*alpha+1 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 1/*2*alpha+1*/ ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 1/*2*alpha+1*/ ).body , 3 ) );
	
	
		dJointSetHingeAxis ( tmp , dBodyGetPositionAll ( getObjektAt ( 1 ).body , 2 ) - dBodyGetPositionAll ( getObjektAt ( 2*alpha+1 ).body , 2 ) , -(dBodyGetPositionAll ( getObjektAt ( 1 ).body , 1 ) - dBodyGetPositionAll ( getObjektAt ( 2*alpha+1 ).body , 1 )) , 0 );
	
	//fixing  of the joints to one angle number: zero
/* 	dJointSetHingeParam ( tmp , dParamLoStop , -M_PI/conf.difference_angle_factor );
 	dJointSetHingeParam ( tmp , dParamHiStop , M_PI/conf.difference_angle_factor );
 	dJointSetHingeParam ( tmp , dParamLoStop , 0 );
 	dJointSetHingeParam ( tmp , dParamHiStop , 0 );*/
  }

  //*****************joint definition***********
  dJointID tmpj1 , tmpj2 , tmpj3;
  //definition of the 3 ground Ball-Joints, which connect the main sphere and the inner parts  
  tmpj1 = dJointCreateBall ( world , 0 );
  dJointAttach ( tmpj1 , getObjektAt ( 0 ).body , getObjektAt ( 2 ).body );
  dJointSetBallAnchor ( tmpj1 , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 2 ).body , 3 ) );
  
  tmpj2 = dJointCreateBall ( world , 0 );
  dJointAttach ( tmpj2 , getObjektAt ( 0 ).body , getObjektAt ( 4 ).body );
  dJointSetBallAnchor ( tmpj2 , dBodyGetPositionAll ( getObjektAt ( 4 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 4 ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 4 ).body , 3 ) );
  
  tmpj3 = dJointCreateBall ( world , 0 );
  dJointAttach ( tmpj3 , getObjektAt ( 0 ).body , getObjektAt ( 6 ).body );
  dJointSetBallAnchor ( tmpj3 , dBodyGetPositionAll ( getObjektAt ( 6 ).body , 1 ) , dBodyGetPositionAll ( getObjektAt ( 6 ).body , 2 ) , dBodyGetPositionAll ( getObjektAt ( 6 ).body , 3 ) );
  
  
  //definition of the 3 Slider-Joints, which are the controled by the robot-controler
  jointliste.push_back ( dJointCreateSlider ( world , 0 ) );
  dJointAttach ( jointliste.back () , getObjektAt ( 3 ).body , getObjektAt ( 2 ).body );
  dJointSetSliderAxis ( jointliste.back() , 0 , 0 , 1 );
  dJointSetSliderParam ( jointliste.back() , dParamLoStop , 0.2 );
  dJointSetSliderParam ( jointliste.back() , dParamHiStop , 0.45 );  
  
  jointliste.push_back ( dJointCreateSlider ( world , 0 ) );
  dJointAttach ( jointliste.back () , getObjektAt ( 5 ).body , getObjektAt ( 4 ).body );
  dJointSetSliderAxis ( jointliste.back() , 0 , 0 , 1 );
  dJointSetSliderParam ( jointliste.back() , dParamLoStop , 0.2 );
  dJointSetSliderParam ( jointliste.back() , dParamHiStop , 0.45 );
  
  jointliste.push_back ( dJointCreateSlider ( world , 0 ) );
  dJointAttach ( jointliste.back () , getObjektAt ( 7 ).body , getObjektAt ( 6 ).body );
  dJointSetSliderAxis ( jointliste.back() , 0 , 0 , 1 );
  dJointSetSliderParam ( jointliste.back() , dParamLoStop , 0.2 );
  dJointSetSliderParam ( jointliste.back() , dParamHiStop , 0.45 );
    
  for ( int n = 0; n < 3; n++ )
  {
	motorliste.push_back ( dJointCreateLMotor ( world , 0 ) );
	dJointAttach ( motorliste.back () , getObjektAt ( 2*n+2 ).body , getObjektAt ( 2*n+3 ).body );
	dJointSetLMotorNumAxes ( motorliste.back () , 3 );

	dJointSetLMotorAxis ( motorliste.back () , 0 , 1 , 0 , 0 , 1 );
	dJointSetLMotorAxis ( motorliste.back () , 2 , 2 , 1 , 0 , 0 );
	//dJointSetLMotorParam ( motorliste.back () , parameter, dReal value);
  }
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
  
  dsDrawSphere ( dGeomGetPosition ( getObjektAt ( 0 ).geom ) , dGeomGetRotation ( getObjektAt ( 0 ).geom ) , conf.diameter/2 );
  //const double box[3]={0.8,0.8,0.8};
  //dsDrawBox ( dGeomGetPosition ( getObjektAt ( 0 ).geom ) , dGeomGetRotation ( getObjektAt ( 0 ).geom ) , box );
  dsDrawSphere ( dGeomGetPosition ( getObjektAt ( 1 ).geom ) , dGeomGetRotation ( getObjektAt ( 1 ).geom ) , conf.pendulardiameter/2 );
  
  dsSetColor ( 1 , 0 , 0 );
  dsDrawCylinder ( dBodyGetPosition ( getObjektAt ( 2 ).body ) , dBodyGetRotation ( getObjektAt ( 2 ).body ) , 0.1 , 0.05 );
  dsSetColor ( 0 , 1 , 0 );
  dsDrawCylinder ( dBodyGetPosition ( getObjektAt ( 4 ).body ) , dBodyGetRotation ( getObjektAt ( 4 ).body ) , 0.1 , 0.05 );
  dsSetColor ( 0 , 0 , 1 );
  dsDrawCylinder ( dBodyGetPosition ( getObjektAt ( 6 ).body ) , dBodyGetRotation ( getObjektAt ( 6 ).body ) , 0.1 , 0.05 );
 
  dsSetColor ( 1 , 0 , 0 );
  dsDrawCylinder ( dBodyGetPosition ( getObjektAt ( 3 ).body ) , dBodyGetRotation ( getObjektAt ( 3 ).body ) , 0.05 , 0.05 );
  dsSetColor ( 0 , 1 , 0 );
  dsDrawCylinder ( dBodyGetPosition ( getObjektAt ( 5 ).body ) , dBodyGetRotation ( getObjektAt ( 5 ).body ) , 0.05 , 0.05 );
  dsSetColor ( 0 , 0 , 1 );
  dsDrawCylinder ( dBodyGetPosition ( getObjektAt ( 7 ).body ) , dBodyGetRotation ( getObjektAt ( 7 ).body ) , 0.05 , 0.05 );

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
  pos.z = max ( conf.diameter/2 , pos.z );
  
  double dx , dy , dz;
  dx = pos.x - dBodyGetPositionAll ( getObjektAt ( 0 ).body , 1 );
  dy = pos.y - dBodyGetPositionAll ( getObjektAt ( 0 ).body , 2 );
  dz = pos.z - dBodyGetPositionAll ( getObjektAt ( 0 ).body , 3 );
  
  dBodySetPosition ( getObjektAt ( 0 ).body , pos.x , pos.y , pos.z );
    
  for ( int n = 1; n < getObjektAnzahl (); n++ )
  	dBodySetPosition ( getObjektAt ( n ).body , dBodyGetPositionAll ( getObjektAt ( n ).body , 1 ) + dx , dBodyGetPositionAll ( getObjektAt ( n ).body , 2 ) + dy , dBodyGetPositionAll ( getObjektAt ( n ).body , 3 ) + dz );
  
  	
  if(c)
    color = (*c);
}
/**
 *
 */
void Sphererobot::mycallback(void *data, dGeomID o1, dGeomID o2)
{
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
	// the rest is for collisions of some sphere elements with the rest of the world
	int i,n;  
	const int N = 10;
	dContact contact[N];
	
	n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	for (i=0; i<n; i++)
	{
		contact[i].surface.mode = 0;
		contact[i].surface.mu = 0.7;
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
  //if ( sensornumber > 3 ) sensornumber = 3;
  //sensoraktualisierung ();
  for ( int n = 0; n < sensornumber; n++ )
  {
  	if ( conf.ausgabeArt == angle )
  		(*sensors++) = dJointGetSliderPosition ( getJointAt ( n ) );
  	if ( conf.ausgabeArt == anglerate )
  		(*sensors++) = dJointGetSliderPositionRate ( getJointAt ( n ) );
	
	dsPrint ( "n= %i Angle= %lf\n" , n , (*sensors++) = dJointGetSliderPosition ( getJointAt ( n ) ) );
	dsPrint ( "n= %i Anglerate= %lf\n" , n , (*sensors++) = dJointGetSliderPositionRate ( getJointAt ( n ) ) );
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
  for ( int n = 0; n < motornumber; n++ )
  {
 	dJointSetLMotorParam ( getMotorAt ( n ) , dParamVel , *(motors++)*conf.factorVelocity );
 	dJointSetLMotorParam ( getMotorAt ( n ) , dParamFMax , conf.maxMotorKraft );
  }
}	

/**
 *Returns the number of motors used by the snake.
 *@return number of motors
 *@author Marcel Kretschmann
 *@version final
 **/
int Sphererobot::getMotorNumber()
{
  return getMotorAnzahl ();
}

/**
 *Returns the number of sensors used by the robot.
 *@return number of sensors
 *@author Marcel Kretschmann
 *@version final
 **/
int Sphererobot::getSensorNumber()
{
	return getMotorNumber ();
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
  	sensorfeld[n].istwinkel = dJointGetLMotorAnglerate ( getMotorAt ( n ) );*/
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
