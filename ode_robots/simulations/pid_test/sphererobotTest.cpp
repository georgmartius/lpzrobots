/************************************************************************/
/*shpererobot.cpp							*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

#include "sphererobotTest.h"
#include "simulation.h"
#include <iostream>
#include <assert.h>

/**
 *constructor
 *@param startRoboterID ID, which should be managed clearly

 *@author Marcel Kretschmann
 *@version beta
 **/
SphererobotTest::SphererobotTest ( int startRoboterID , const OdeHandle& odeHandle, 
			   const SphererobotConf& conf )
  : Roboter ( startRoboterID , odeHandle , 1 )
{
  sphererobot_space = dSimpleSpaceCreate ( space );
  dSpaceSetCleanup ( sphererobot_space , 0 );

  this->conf = conf;
	
  dMass mass, mass2, mass3, mass4, mass5;
  Position pos(0 , 0 , conf.diameter/2);

  //*************body definition**************
  Object base;
  Object pendular;
  Object bottom[3];
  Object top[3];       
  //sphere base body
  lastJointPos[0]=0;

  base.body = dBodyCreate ( world );

  dBodySetPosition ( base.body , pos.x , pos.y , pos.z );
  dMassSetSphereTotal ( &mass , conf.spheremass * 1000 , conf.diameter/2 );
  dBodySetMass ( base.body , &mass );

  base.geom = dCreateBox ( sphererobot_space , conf.diameter,conf.diameter,conf.diameter );
  //base.geom = dCreateBox ( sphererobot_space , conf.diameter,conf.diameter,conf.diameter );
  dGeomSetBody ( base.geom , base.body );
      
  //pendular body
  pendular.body = dBodyCreate ( world );

  dBodySetPosition ( pendular.body , pos.x , pos.y , pos.z + 1 );
  dMassSetSphereTotal ( &mass2 , conf.spheremass , conf.diameter/2 );
  dBodySetMass ( pendular.body , &mass2 );
  
  pendular.geom=  dCreateSphere ( sphererobot_space , conf.diameter /2 );

  //obj.geom = dCreateBox ( sphererobot_space , 0.8,0.8,0.8);
  dGeomSetBody ( pendular.geom , pendular.body );

  //definition of the 3 Slider-Joints, which are the controled by the robot-controler
  dJointID slider = dJointCreateSlider ( world , 0 );
  dJointAttach ( slider , pendular.body , base.body );
  dJointSetSliderAxis ( slider, 0, 0, 1 );
  dJointSetSliderParam ( slider, dParamLoStop, -0.1 );
  dJointSetSliderParam ( slider, dParamHiStop, 0.1 );
  jointliste.push_back ( slider );
  motorliste.push_back ( slider );
  servo=new SliderServo(slider, -conf.sliderrange, conf.sliderrange, conf.spheremass);
  //  motorliste2.push_back ( new PID ( 500 , 0 , 20 ) );
  
  
    
//   //first and second 3 conection bodies between the pendular an the sphere
//   double x , y;
//   Position pendularPos (dBodyGetPosition(pendular.body));
//   for ( unsigned int alpha = 0; alpha < 3; alpha++ ) {
//     x=sin ( (float) alpha*2*M_PI/3 )*conf.diameter/4; //testing values
//     y=cos ( (float) alpha*2*M_PI/3 )*conf.diameter/4;

//     Position bottomPos (pendularPos.x + x , pendularPos.y + y , pendularPos.z - conf.diameter/2 );
//     bottom[alpha].body = dBodyCreate ( world );
//     dBodySetPosition ( bottom[alpha].body , bottomPos.x , bottomPos.y , bottomPos.z );
//     dMassSetBoxTotal ( &mass3 , conf.slidermass*2 , 0.2 , 0.2 , 1 );
//     dBodySetMass ( bottom[alpha].body , &mass3 );
//     bottom[alpha].geom=0;

//     x=sin ( (float) alpha*2*M_PI/3 )*conf.diameter/4; //testing values
//     y=cos ( (float) alpha*2*M_PI/3 )*conf.diameter/4;
//     Position topPos (pendularPos.x + x, pendularPos.y + y, pendularPos.z);
//     top[alpha].body = dBodyCreate ( world );
//     dBodySetPosition( top[alpha].body, topPos.x, topPos.y, topPos.z);
//     dMassSetBoxTotal( &mass3 , conf.slidermass*2 , 0.2 , 0.2 , 0.2 );
//     dBodySetMass ( top[alpha].body , &mass3 );
//     top[alpha].geom=0;

//     //combines the 3 upper connection bodies with the pendular
//     dJointID hinge = dJointCreateHinge ( world , 0 );
//     dJointAttach ( hinge , pendular.body , top[alpha].body );
	      
//     dJointSetHingeAnchor ( hinge , topPos.x, topPos.y, pendularPos.z);	
	
//     dJointSetHingeAxis ( hinge, (pendularPos.y - topPos.y) , -(pendularPos.x - topPos.x), 0 );
//     dJointSetHingeParam ( hinge, dParamLoStop, M_PI/180 * -10 );
//     dJointSetHingeParam ( hinge, dParamHiStop, M_PI/180 * 10  );
	
//     //fixing  of the joints to one angle number: zero
//     /* 	dJointSetHingeParam ( tmp , dParamLoStop , -M_PI/conf.difference_angle_factor );
// 	dJointSetHingeParam ( tmp , dParamHiStop , M_PI/conf.difference_angle_factor );
// 	dJointSetHingeParam ( tmp , dParamLoStop , 0 );
// 	dJointSetHingeParam ( tmp , dParamHiStop , 0 );*/
      
//     //***************** ball joint definition***********
// //     dJointID balljoint;
// //     //definition of the ground Ball-Joint, which connects the main sphere and the inner parts  
// //     balljoint = dJointCreateBall ( world , 0 );
// //     dJointAttach ( balljoint, base.body , bottom[alpha].body );
// //     dJointSetBallAnchor ( balljoint , bottomPos.x , bottomPos.y , bottomPos.z );

  
//     //definition of the 3 Slider-Joints, which are the controled by the robot-controler
//     dJointID slider = dJointCreateSlider ( world , 0 );
//     dJointAttach ( slider , bottom[alpha].body , top[alpha].body );
//     dJointSetSliderAxis ( slider, 0, 0, 1 );
//     dJointSetSliderParam ( slider, dParamLoStop, 0 );
//     dJointSetSliderParam ( slider, dParamHiStop, 0.5 );
//     jointliste.push_back ( slider );
//     motorliste.push_back ( slider);

  
//     dJointID lmotor;
//     lmotor = dJointCreateLMotor (world,0);
//     dJointAttach ( lmotor, bottom[alpha].body , top[alpha].body );
//     dJointSetLMotorNumAxes ( lmotor , 1 );
//     dJointSetLMotorAxis ( lmotor, 0 , 1 , 0 , 0 , 1 );
//     dJointSetSliderParam ( lmotor, dParamLoStop, 0.0 );
//     dJointSetSliderParam ( lmotor, dParamHiStop, 0.5 );
//     //    dJointSetLMotorAxis ( lmotor, 2 , 2 , 1 , 0 , 0 );
//     //dJointSetLMotorParam ( lmotor , parameter, dReal value);
//     motorliste.push_back ( lmotor);
//  }
  objektliste.push_back(base); assert(objektliste.size() == Base + 1);
  objektliste.push_back(pendular); assert(objektliste.size() == Pendular + 1);
//   objektliste.push_back(bottom[0]); assert(objektliste.size() == Pole1Bot + 1);
//   objektliste.push_back(bottom[1]); assert(objektliste.size() == Pole2Bot + 1);
//   objektliste.push_back(bottom[2]); assert(objektliste.size() == Pole3Bot + 1);
//   objektliste.push_back(top[0]); assert(objektliste.size() == Pole1Top + 1);
//   objektliste.push_back(top[1]); assert(objektliste.size() == Pole2Top + 1);
//   objektliste.push_back(top[2]); assert(objektliste.size() == Pole3Top + 1);
}
	
/**
 *Destruktor
 *@author Marcel Kretschmann
 *@version beta
 **/
SphererobotTest::~SphererobotTest()
{
  dSpaceDestroy ( sphererobot_space );
}

/**
 *Draws all elements of the snake.
 *@author Marcel Kretschmann
 *@version beta
 **/
void SphererobotTest::draw()
{
  dsSetTexture (DS_WOOD);
  dsSetColor ( color.r , color.g , color.b );
  
  //  dsDrawSphere ( dGeomGetPosition ( getObjektAt ( Base ).geom ) , 
  //  		 dGeomGetRotation ( getObjektAt ( Base ).geom ) , conf.diameter/2 );
  const double box[3]={0.8,0.8,0.8};
  dsDrawBox ( dGeomGetPosition ( getObjektAt ( Base ).geom ) , 
   	      dGeomGetRotation ( getObjektAt ( Base ).geom ) , box );

  dsSetColor ( 1 , 1 , 0 );
  dsDrawSphere ( dGeomGetPosition ( getObjektAt ( Pendular ).geom ), 
  		 dGeomGetRotation ( getObjektAt ( Pendular ).geom ) , conf.diameter/2);
  
//   for(unsigned int n = 0; n < 3; n++){
//     dsSetColor ( n==0 , n==1 , n==2 );
//     dsDrawCylinder ( dBodyGetPosition ( getObjektAt ( Pole1Bot + n ).body ) , 
// 		     dBodyGetRotation ( getObjektAt ( Pole1Bot + n ).body ) , 0.1 , 0.05 );
//     dsSetColor ( 1 , n==1 , n==2 );
//     dsDrawCylinder ( dBodyGetPosition ( getObjektAt ( Pole1Top + n ).body ) , 
// 		     dBodyGetRotation ( getObjektAt ( Pole1Top + n ).body ) , 0.05 , 0.05 );
//   }
}

/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the array
 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 *@author Marcel Kretschmann
 *@version beta
 **/
int SphererobotTest::getSensors ( sensor* sensors, int sensornumber )
{
  //if ( sensornumber > 3 ) sensornumber = 3;
  //sensoraktualisierung ();
  for ( int n = 0; n < sensornumber; n++ )
    {
    /*  if ( conf.outputtype == angle )
	(*sensors++) = dJointGetSliderPosition ( getJointAt ( n ) );
      if ( conf.outputtype == anglerate )
	(*sensors++) = dJointGetSliderPositionRate ( getJointAt ( n ) );*/
	
	
      //      dsPrint ( "n= %i Angle= %lf\n" , n , (*sensors++) = dJointGetSliderPosition ( getJointAt ( n ) ) );
      //      dsPrint ( "n= %i Anglerate= %lf\n" , n , (*sensors++) = dJointGetSliderPositionRate ( getJointAt ( n ) ) );
    }
	
  (*sensors++) = servo->get();
  (*sensors++) = servo->error;
  (*sensors++) = servo->force;
  (*sensors++) = servo->I;
  return 4; //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden
}


/**
 *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 *@author Marcel Kretschmann
 *@version beta
 **/
void SphererobotTest::setMotors ( const motor* motors, int motornumber ) {
  for ( int n = 0; n < motornumber; n++ )	
  {
    //  double force;
    //motorliste2[n]->setTargetPosition ( motors[n] );
    //   force = motorliste2[n]->Step ( dJointGetSliderPosition ( getJointAt ( n ) ) );
    // 	dJointAddSliderForce( getJointAt ( n ) , force );
    // 	cout<<"Force: "<<force<<"\n";
  }
  servo->set(motors[0]);
  

}	




/**Sets the snake to position pos, sets color to c, and creates snake if necessary.
 *This overwrides the function place of the class robot.
 *@param pos desired position of the snake in struct Position
 *@param c desired color for the snake in struct Color (might be NULL!)
 *@author Marcel Kretschmann
 *@version beta
 **/
void SphererobotTest::place (Position pos, Color *c)
{
  pos.z = max ( conf.diameter/2 , pos.z );
  
  double dx , dy , dz;
  dx = pos.x - dBodyGetPositionAll ( getObjektAt ( Base ).body , 1 );
  dy = pos.y - dBodyGetPositionAll ( getObjektAt ( Base ).body , 2 );
  dz = pos.z - dBodyGetPositionAll ( getObjektAt ( Base ).body , 3 );
  
  dBodySetPosition ( getObjektAt ( Base ).body , pos.x , pos.y , pos.z );
    
  for ( int n = 1; n < getObjektAnzahl (); n++ )
    dBodySetPosition ( getObjektAt ( n ).body , 
		       dBodyGetPositionAll ( getObjektAt ( n ).body , 1 ) + dx, 
		       dBodyGetPositionAll ( getObjektAt ( n ).body , 2 ) + dy, 
		       dBodyGetPositionAll ( getObjektAt ( n ).body , 3 ) + dz );
  
  	
  if(c)
    color = (*c);
}
/**
 *
 */
void SphererobotTest::mycallback(void *data, dGeomID o1, dGeomID o2)
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
bool SphererobotTest::collisionCallback(void *data, dGeomID o1, dGeomID o2)
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
 *Returns the number of motors used by the snake.
 *@return number of motors
 *@author Marcel Kretschmann
 *@version final
 **/
int SphererobotTest::getMotorNumber()
{
  return getMotorAnzahl ();
}

/**
 *Returns the number of sensors used by the robot.
 *@return number of sensors
 *@author Marcel Kretschmann
 *@version final
 **/
int SphererobotTest::getSensorNumber()
{
  return /*getMotorNumber ()*/4;
}
	
/**
 *Updates the sensorarray.
 *This overwrides the function sensoraktualisierung of the class robot
 *@author Marcel Kretschmann
 *@version beta
 **/
void SphererobotTest::sensoraktualisierung ( )
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
Position SphererobotTest::getPosition ()
{
  return Position(dBodyGetPosition ( getObjektAt(Base).body ));
}

/**
 *Prints some internal robot parameters. Actualy it prints all sensor data of one callculation step.
 *@author Marcel Kretschmann
 *@version beta
 **/
void SphererobotTest::getStatus ()
{
  for ( int n = 0; n < getSensorfeldGroesse (); n++){
    dsPrint ( "Sensor %i: %lf\n" , n , sensorfeld[n].istwinkel );
  }
}
