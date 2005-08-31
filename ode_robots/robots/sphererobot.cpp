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

const int Sphererobot::servono;
const int Sphererobot::sensorno;


/**
 *constructor
 *@param startRoboterID ID, which should be managed clearly

 *@author Marcel Kretschmann
 *@version beta
 **/
Sphererobot::Sphererobot ( const ODEHandle& odeHandle, 
			   const SphererobotConf& conf )
  : AbstractRobot ( odeHandle, "Sphere_Robot" )
{
  sphererobot_space = dSimpleSpaceCreate ( space );
  dSpaceSetCleanup ( sphererobot_space , 0 );
  
  this->conf = conf;
	
  Position pos(0 , 0 , conf.diameter/2);

  //*************body definition**************
  dMass mass;
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
  dMassSetSphereTotal ( &mass , conf.pendularmass , conf.pendulardiameter/2 );
  dBodySetMass ( pendular.body , &mass );
  
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
    dMassSetBoxTotal ( &mass , conf.slidermass , 0.2 , 0.2 , 1 );
   
    dBodySetMass ( bottom[alpha].body , &mass );
    bottom[alpha].geom=0;

    Position topPos (pendularPos.x + x, pendularPos.y + y, pendularPos.z);
    top[alpha].body = dBodyCreate ( world );
    dBodySetPosition( top[alpha].body, topPos.x, topPos.y, topPos.z);
    dMassSetBoxTotal( &mass , conf.slidermass , 0.2 , 0.2 , 0.2 );
    dBodySetMass ( top[alpha].body , &mass );
    top[alpha].geom=0;

    //combines the 3 upper connection bodies with the pendular
    dJointID hinge = dJointCreateHinge ( world , 0 );
    dJointAttach ( hinge , pendular.body , top[alpha].body );
	      
    dJointSetHingeAnchor ( hinge , topPos.x, topPos.y, pendularPos.z);	
	
    dJointSetHingeAxis ( hinge, (pendularPos.y - topPos.y) , -(pendularPos.x - topPos.x), 0 );
    dJointSetHingeParam ( hinge, dParamLoStop, -conf.hingeRange);
    dJointSetHingeParam ( hinge, dParamHiStop,  conf.hingeRange);
    dJointSetHingeParam  ( hinge, dParamCFM, 0.1);
    dJointSetHingeParam ( hinge, dParamStopCFM, 0.1);
    dJointSetHingeParam ( hinge, dParamStopERP, 0.9);

	
      
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
    dJointSetSliderParam ( slider, dParamLoStop, -2*conf.diameter*conf.sliderrange );
    dJointSetSliderParam ( slider, dParamHiStop, 2*conf.diameter*conf.sliderrange );
    dJointSetSliderParam ( slider, dParamCFM, 0.1);
    dJointSetSliderParam ( slider, dParamStopCFM, 0.1);
    dJointSetSliderParam ( slider, dParamStopERP, 0.9);
    servo[alpha] = new SliderServo(slider, -conf.diameter*conf.sliderrange, 
				   conf.diameter*conf.sliderrange, 
				   conf.pendularmass); 
  
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
  object[Base]     = base;
  object[Pendular] = pendular; 
  object[Pole1Bot] = bottom[0];
  object[Pole2Bot] = bottom[1];
  object[Pole3Bot] = bottom[2];
  object[Pole1Top] = top[0]; 
  object[Pole2Top] = top[1]; 
  object[Pole3Top] = top[2]; 
}
	
/**
 *Destruktor
 *@author Marcel Kretschmann
 *@version beta
 **/
Sphererobot::~Sphererobot()
{
  dSpaceDestroy ( sphererobot_space );
  //todo  delete object and motors
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
  
  dsDrawSphere ( dGeomGetPosition ( object[ Base ].geom ) , 
  		 dGeomGetRotation ( object[ Base ].geom ) , conf.diameter/2 );
  //   const double box[3]={0.8,0.8,0.8};
  //   dsDrawBox ( dGeomGetPosition ( object[ Base ].geom ) , 
  // 	      dGeomGetRotation ( object[ Base ].geom ) , box );

  dsSetColor ( 1 , 1 , 0 );
  dsDrawSphere ( dGeomGetPosition ( object[ Pendular ].geom ), 
  		 dGeomGetRotation ( object[ Pendular ].geom ) , conf.pendulardiameter/2 );
  
  for(unsigned int n = 0; n < 3; n++){
    dsSetColor ( n==0 , n==1 , n==2 );
    const dReal* pos1 = dBodyGetPosition ( object[ Pole1Bot + n ].body);
    const dReal* pos2 = dBodyGetPosition ( object[ Pole1Top + n ].body);
    dReal pos[3];
    double len=0;
    for(int i=0; i<3; i++){
      len+= (pos1[i] - pos2[i])*(pos1[i] - pos2[i]);
      pos[i] = (pos1[i] + pos2[i])/2;
    }    
    dsDrawCylinder ( pos , dBodyGetRotation ( object[ Pole1Bot + n ].body ) , 
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
  int len = min(sensornumber, servono);
  for ( int n = 0; n < len; n++ ) {
    sensors[n] = servo[n]->get();
  }
  return len;
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
  int len = min(motornumber, servono);
  for ( int n = 0; n < len; n++ ) {
    servo[n]->set(motors[n]);
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
  
  Position basepos(dBodyGetPosition ( object[ Base ].body ));
  Position d = pos - basepos;
  
  dBodySetPosition ( object[ Base ].body , pos.x , pos.y , pos.z );    
  
  for ( int n = 1; n < Last; n++ ){
    Position npos (dBodyGetPosition( object[ n ].body ));
    npos = npos + d;
    dBodySetPosition ( object[ n ].body , npos.x, npos.y, npos.z );
  }
  
  if(c)
    color = (*c);
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
	  contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
	  contact[i].surface.mu = 1.0;
	  contact[i].surface.soft_erp = 0.5;
	  contact[i].surface.soft_cfm = 0.1;
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
int Sphererobot::getMotorNumber(){
  return servono;
}

/**
 *Returns the number of sensors used by the robot.
 *@return number of sensors
 *@author Marcel Kretschmann
 *@version final
 **/
int Sphererobot::getSensorNumber() {
  return sensorno;
}

/**
 *Returns the position of the snake. Here the position of the snake is the position of the first element of the snake.
 *@return Position (x,y,z)
 *@author Marcel Kretschmann
 *@version final
 **/
Position Sphererobot::getPosition () {
  return Position(dBodyGetPosition ( object[Base].body ));
}

/** returns a vector with the positions of all segments of the robot
    @param vector of positions (of all robot segments) 
    @return length of the list
*/
int Sphererobot::getSegmentsPosition(vector<Position> &poslist){
  poslist.push_back(Position(dBodyGetPosition ( object[Base].body )));
  poslist.push_back(Position(dBodyGetPosition ( object[Pendular].body )));
  return 2;
}
