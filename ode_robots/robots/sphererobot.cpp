/************************************************************************/
/*shpererobot.cpp							*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

#include "sphererobot.h"
#include "simulation.h"
#include <iostream>
#include <assert.h>

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

  this->conf = conf;
	
  dMass mass, mass2, mass3, mass4, mass5;
  Position pos(0 , 0 , conf.diameter/2);

  //*************body definition**************
  Object base;
  Object pendular;
  Object bottom[3];
  Object top[3];       
  //sphere base body

  base.body = dBodyCreate ( world );

  dBodySetPosition ( base.body , pos.x , pos.y , pos.z );
  dMassSetSphereTotal ( &mass , conf.spheremass , conf.diameter/2 );
  dBodySetMass ( base.body , &mass );

  base.geom = dCreateSphere ( sphererobot_space , conf.diameter/2 );
  //base.geom = dCreateBox ( sphererobot_space , conf.diameter,conf.diameter,conf.diameter );
  dGeomSetBody ( base.geom , base.body );

  //pendular body
  pendular.body = dBodyCreate ( world );

  dBodySetPosition ( pendular.body , pos.x , pos.y , pos.z);
  dMassSetSphereTotal ( &mass2 , conf.pendularmass , conf.pendulardiameter/2 );
  dBodySetMass ( pendular.body , &mass2 );
  
  pendular.geom = dCreateSphere ( sphererobot_space , conf.pendulardiameter/2 );
  //obj.geom = dCreateBox ( sphererobot_space , 0.8,0.8,0.8);
  dGeomSetBody ( pendular.geom , pendular.body );
    
  //first and second 3 conection bodies between the pendular an the sphere
  double x , y;
  Position pendularPos (dBodyGetPosition(pendular.body));
  for ( unsigned int alpha = 0; alpha < 3; alpha++ ) {
    x=sin ( (float) alpha*2*M_PI/3 )*conf.diameter/4; //testing values
    y=cos ( (float) alpha*2*M_PI/3 )*conf.diameter/4;

    Position bottomPos (pendularPos.x + x , pendularPos.y + y , 
			pos.z - conf.diameter/2 + conf.diameter/5 );
    bottom[alpha].body = dBodyCreate ( world );
    dBodySetPosition ( bottom[alpha].body , bottomPos.x , bottomPos.y , bottomPos.z );
    dMassSetBoxTotal ( &mass3 , conf.slidermass , 0.2 , 0.2 , 1 );
   
    dBodySetMass ( bottom[alpha].body , &mass3 );
    bottom[alpha].geom=0;

    Position topPos (pendularPos.x + x, pendularPos.y + y, pendularPos.z);
    top[alpha].body = dBodyCreate ( world );
    dBodySetPosition( top[alpha].body, topPos.x, topPos.y, topPos.z);
    dMassSetBoxTotal( &mass3 , conf.slidermass , 0.2 , 0.2 , 0.2 );
    dBodySetMass ( top[alpha].body , &mass3 );
    top[alpha].geom=0;

    //combines the 3 upper connection bodies with the pendular
    dJointID hinge = dJointCreateHinge ( world , 0 );
    dJointAttach ( hinge , pendular.body , top[alpha].body );
	      
    dJointSetHingeAnchor ( hinge , topPos.x, topPos.y, pendularPos.z);	
	
    dJointSetHingeAxis ( hinge, (pendularPos.y - topPos.y) , -(pendularPos.x - topPos.x), 0 );
    //    dJointSetHingeParam ( hinge, dParamLoStop, -conf.hingeRange);
    //    dJointSetHingeParam ( hinge, dParamHiStop,  conf.hingeRange);
    dJointSetHingeParam  ( hinge, dParamCFM, 0.01);
	
      
    //***************** ball joint definition***********
    dJointID balljoint;
    //definition of the ground Ball-Joint, which connects the main sphere and the inner parts  
    balljoint = dJointCreateBall ( world , 0 );
    dJointAttach ( balljoint, base.body , bottom[alpha].body );
    dJointSetBallAnchor ( balljoint , bottomPos.x , bottomPos.y , bottomPos.z );
  
    //definition of the 3 Slider-Joints, which are the controled by the robot-controler
    dJointID slider = dJointCreateSlider ( world , 0 );
    dJointAttach ( slider , top[alpha].body, bottom[alpha].body );
    dJointSetSliderAxis ( slider, 0, 0, 1 );
    // the Stop parameters are messured from the initial position!
    dJointSetSliderParam ( slider, dParamLoStop, -conf.diameter*conf.sliderrange );
    dJointSetSliderParam ( slider, dParamHiStop, conf.diameter*conf.sliderrange );
//     dJointSetSliderParam ( slider, dParamBounce, 0.0 );
//     double s  = 200; // spring constant;
//     double kd =  50; // damping parameter;
//     dJointSetHingeParam  ( slider, dParamStopERP, s / (s + kd));
//     dJointSetHingeParam  ( slider, dParamStopCFM, 1 / (s + kd));
//     dJointSetHingeParam  ( slider, dParamCFM, 1 / (s + kd));

    
     jointliste.push_back ( slider );
     motorliste.push_back ( slider);
  
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
  }
  objektliste.push_back(base); assert(objektliste.size() == Base + 1);
  objektliste.push_back(pendular); assert(objektliste.size() == Pendular + 1);
  objektliste.push_back(bottom[0]); assert(objektliste.size() == Pole1Bot + 1);
  objektliste.push_back(bottom[1]); assert(objektliste.size() == Pole2Bot + 1);
  objektliste.push_back(bottom[2]); assert(objektliste.size() == Pole3Bot + 1);
  objektliste.push_back(top[0]); assert(objektliste.size() == Pole1Top + 1);
  objektliste.push_back(top[1]); assert(objektliste.size() == Pole2Top + 1);
  objektliste.push_back(top[2]); assert(objektliste.size() == Pole3Top + 1);
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
  
  dsDrawSphere ( dGeomGetPosition ( getObjektAt ( Base ).geom ) , 
  		 dGeomGetRotation ( getObjektAt ( Base ).geom ) , conf.diameter/2 );
  //   const double box[3]={0.8,0.8,0.8};
  //   dsDrawBox ( dGeomGetPosition ( getObjektAt ( Base ).geom ) , 
  // 	      dGeomGetRotation ( getObjektAt ( Base ).geom ) , box );

  dsSetColor ( 1 , 1 , 0 );
  dsDrawSphere ( dGeomGetPosition ( getObjektAt ( Pendular ).geom ), 
  		 dGeomGetRotation ( getObjektAt ( Pendular ).geom ) , conf.pendulardiameter/2 );
  
  for(unsigned int n = 0; n < 3; n++){
    dsSetColor ( n==0 , n==1 , n==2 );
    const dReal* pos1 = dBodyGetPosition ( getObjektAt ( Pole1Bot + n ).body);
    const dReal* pos2 = dBodyGetPosition ( getObjektAt ( Pole1Top + n ).body);
    dReal pos[3];
    double len=0;
    for(int i=0; i<3; i++){
      len+= (pos1[i] - pos2[i])*(pos1[i] - pos2[i]);
      pos[i] = (pos1[i] + pos2[i])/2;
    }    
    dsDrawCylinder ( pos , dBodyGetRotation ( getObjektAt ( Pole1Bot + n ).body ) , 
		     sqrt(len) , 0.05 );    
  }
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
      if ( conf.outputtype == angle )
	(*sensors++) = dJointGetSliderPosition ( getJointAt ( n ) ) / (conf.sliderrange) ;
      if ( conf.outputtype == anglerate )
	(*sensors++) = dJointGetSliderPositionRate ( getJointAt ( n ) );
	
      //      dsPrint ( "n= %i Angle= %lf\n" , n , (*sensors++) = dJointGetSliderPosition ( getJointAt ( n ) ) );
      //      dsPrint ( "n= %i Anglerate= %lf\n" , n , (*sensors++) = dJointGetSliderPositionRate ( getJointAt ( n ) ) );
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
void Sphererobot::setMotors ( const motor* motors, int motornumber ) {
  for ( int n = 0; n < motornumber; n++ ) {
    dJointAddSliderForce( getMotorAt ( n ), motors[n]*conf.maxforce);
    
    //       dJointSetLMotorParam ( getMotorAt ( n ) , dParamVel , *(motors++)*conf.factorVelocity );
    //       dJointSetLMotorParam ( getMotorAt ( n ) , dParamFMax , conf.maxMotorKraft );
  }
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
  pos.z += conf.diameter/2;
  
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
void Sphererobot::mycallback(void *data, dGeomID o1, dGeomID o2) {
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
	  contact[i].surface.mu = 1.0;
	  contact[i].surface.mu2 = 0;
	  contact[i].surface.soft_erp = 1;
	  contact[i].surface.soft_cfm = 0.001;
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
  return Position(dBodyGetPosition ( getObjektAt(Base).body ));
}

/**
 *Prints some internal robot parameters. Actualy it prints all sensor data of one callculation step.
 *@author Marcel Kretschmann
 *@version beta
 **/
void Sphererobot::getStatus ()
{
  for ( int n = 0; n < getSensorfeldGroesse (); n++){
    dsPrint ( "Sensor %i: %lf\n" , n , sensorfeld[n].istwinkel );
  }
}


/*
pid control
 double measure = dJointGetSliderPosition(getJointAt(n));
    double setpoint = 0;
    double p = 1000;
    double KP = p;
    double KI = p/4;
    double KD = p;
    double P  = (setpoint - measure) * KP;
    double I  = -measure * KI;
    double D  = (lastJointPos[n]- measure) * KD ;
    lastJointPos[n]=measure;
    double force = P + I + D;
    dJointAddSliderForce( getMotorAt ( n ), force);
*/
