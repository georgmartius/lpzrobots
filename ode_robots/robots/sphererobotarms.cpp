/************************************************************************/
/*shpererobot.cpp							*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

#include "sphererobotarms.h"
#include "simulation.h"
#include <iostream>
#include <assert.h>

const int SphererobotArms::servono;
const int SphererobotArms::sensorno;

#include "matrix.h"
using namespace matrix;

Matrix odeRto3x3RotationMatrix ( const double R[12] ) {  
  Matrix matrix(3,3);
  matrix.val(0,0)=R[0];
  matrix.val(0,1)=R[4];
  matrix.val(0,2)=R[8];
  matrix.val(1,0)=R[1];
  matrix.val(1,1)=R[5];
  matrix.val(1,2)=R[9];
  matrix.val(2,0)=R[2];
  matrix.val(2,1)=R[6];
  matrix.val(2,2)=R[10];
  return matrix;
}

// pi/2-rotation round x-axis of rotation R
void xrot ( double rotR[12], const double R[12])
{
  
  rotR[0]=R[0];
  rotR[4]=R[4];
  rotR[8]=R[8];
  
  rotR[1]= -R[2];
  rotR[5]= -R[6];
  rotR[9]= -R[10];
  
  rotR[2]=R[1];
  rotR[6]=R[5];
  rotR[10]=R[9];
  
  // whatever
  rotR[3]=R[3];
  rotR[7]=R[7];
  rotR[11]=R[11];
}

// pi/2-rotation round y-axis of rotation R
void yrot ( double rotR[12], const double R[12])
{
  rotR[0]=R[2];
  rotR[4]=R[6];
  rotR[8]=R[10];

  rotR[1]=R[1];
  rotR[5]=R[5];
  rotR[9]=R[9];
  
  rotR[2]= -R[0];
  rotR[6]= -R[4];
  rotR[10]= -R[8];
  
  // whatever
  rotR[3]=R[3];
  rotR[7]=R[7];
  rotR[11]=R[11];
}


/**
 *constructor
 *@param startRoboterID ID, which should be managed clearly

 *@author Marcel Kretschmann
 *@version beta
 **/
SphererobotArms::SphererobotArms ( const ODEHandle& odeHandle, 
			   const SphererobotConf& conf )
  : AbstractRobot ( odeHandle, "Sphere_Robot_Arms" )
{
  sphererobot_space = dSimpleSpaceCreate ( space );
  dSpaceSetCleanup ( sphererobot_space , 0 );
  
  this->conf = conf;

  this->conf.pendulardiameter = conf.diameter/15;
	
  Position pos(0 , 0 , conf.diameter/2);

  //*************body definition**************
  dMass mass;
  Object base;
  Object pendular[3];

  //sphere base body
  base.body = dBodyCreate ( world );

  dBodySetPosition ( base.body , pos.x , pos.y , pos.z );
  dMassSetSphereTotal ( &mass , conf.spheremass , conf.diameter/2 );
  dBodySetMass ( base.body , &mass );

  base.geom = dCreateSphere ( sphererobot_space , conf.diameter/2 );
  //base.geom = dCreateBox ( sphererobot_space , conf.diameter,conf.diameter,conf.diameter );
  dGeomSetBody ( base.geom , base.body );

    
  //definition of the 3 Slider-Joints, which are the controled by the robot-controler
  for ( unsigned int n = 0; n < 3; n++ ) {
    //pendular body
    pendular[n].body = dBodyCreate ( world );
    
    dBodySetPosition ( pendular[n].body , pos.x , pos.y , pos.z);
    dMassSetSphereTotal ( &mass , conf.pendularmass , conf.pendulardiameter/2 );
    dBodySetMass ( pendular[n].body , &mass );
    
    pendular[n].geom = dCreateSphere ( sphererobot_space , conf.pendulardiameter/2 );
    // obj.geom = dCreateBox ( sphererobot_space , 0.8,0.8,0.8);
    dGeomSetBody ( pendular[n].geom , pendular[n].body );
  
    dJointID slider = dJointCreateSlider ( world , 0 );
    dJointAttach ( slider , base.body, pendular[n].body );
    dJointSetSliderAxis ( slider, (n==0), (n==1), (n==2) );
    // the Stop parameters are messured from the initial position!
    dJointSetSliderParam ( slider, dParamLoStop, -1.1*conf.diameter*conf.sliderrange );
    dJointSetSliderParam ( slider, dParamHiStop, 1.1*conf.diameter*conf.sliderrange );
    dJointSetSliderParam ( slider, dParamStopCFM, 0.1);
    dJointSetSliderParam ( slider, dParamStopERP, 0.9);
    servo[n] = new SliderServo(slider, -conf.diameter*conf.sliderrange, 
			       conf.diameter*conf.sliderrange, 
			       conf.pendularmass); 
  }
  object[Base]     = base;
  object[Pendular1] = pendular[0]; 
  object[Pendular2] = pendular[1]; 
  object[Pendular3] = pendular[2]; 
}
	
/**
 *Destruktor
 *@author Marcel Kretschmann
 *@version beta
 **/
SphererobotArms::~SphererobotArms()
{
  dSpaceDestroy ( sphererobot_space );
  //todo  delete object and motors
}

/**
 *Draws all elements of the snake.
 *@author Marcel Kretschmann
 *@version beta
 **/
void SphererobotArms::draw()
{
  dsSetTexture (DS_WOOD);

  for(unsigned int n = 0; n < 3; n++){
    // draw pendular 	
    dsSetColorAlpha (n==0 , n==1 , n==2, 1); // transparency= 1  
    dsDrawSphere ( dGeomGetPosition ( object[ Pendular1 + n ].geom ), 
		   dGeomGetRotation ( object[ Pendular1 + n ].geom ) , conf.pendulardiameter/2 );
  }
  
//   // draw poles
//   for(unsigned int n = 0; n < 3; n++){
//     dsSetColor ( n==0 , n==1 , n==2 );
//     const dReal* pos1 = dBodyGetPosition ( object[ Pole1Bot + n ].body);
//     const dReal* pos2 = dBodyGetPosition ( object[ Pole1Top + n ].body);
//     dReal pos[3];
//     double len=0;
//     for(int i=0; i<3; i++){
//       len+= (pos1[i] - pos2[i])*(pos1[i] - pos2[i]);
//       pos[i] = (pos1[i] + pos2[i])/2;
//     }    
//     dsDrawCylinder ( pos , dBodyGetRotation ( object[ Pole1Bot + n ].body ) , 
// 		     sqrt(len) , 0.05 );    
//   }

  // draw blue axis
  const dReal *R = dBodyGetRotation ( object[ Base ].body);
  const dReal *pos = dGeomGetPosition ( object[ Base ].geom);
  dsDrawCylinder ( pos , R, conf.diameter , conf.diameter/100 );    

  // draw green axis
  double rotR[12];
  xrot ( rotR, R );
  dsSetColorAlpha(0,1,0,1);
  dsDrawCylinder ( pos ,rotR ,conf.diameter , conf.diameter/100 );

  // draw red axis
  yrot ( rotR, R );
  dsSetColorAlpha(1,0,0,1);
  dsDrawCylinder ( pos , rotR , conf.diameter , conf.diameter/100 );
    
  // draw sphere
  dsSetTexture (0);
  dsSetColorAlpha (color.r, color.g, color.b, 0.5); // transparency= 0.5
  
  dsDrawSphere ( dGeomGetPosition ( object[ Base ].geom ) , 
  		 dGeomGetRotation ( object[ Base ].geom ) , conf.diameter/2 );
  //   const double box[3]={0.8,0.8,0.8};
  //   dsDrawBox ( dGeomGetPosition ( object[ Base ].geom ) , 
  // 	      dGeomGetRotation ( object[ Base ].geom ) , box );

}

/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the array
 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 *@author Marcel Kretschmann
 *@version beta
 **/
int SphererobotArms::getSensors ( sensor* sensors, int sensornumber )
{  
   int len = min(sensornumber, servono);
   for ( int n = 0; n < len; n++ ) {
     sensors[n] = servo[n]->get();
   }

   Matrix A = odeRto3x3RotationMatrix(dBodyGetRotation(object[Base].body));
   return A.convertToBuffer(sensors + len , sensornumber -len) + len;
   //return len;
}

/**
 *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 *@author Marcel Kretschmann
 *@version beta
 **/
void SphererobotArms::setMotors ( const motor* motors, int motornumber ) {
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
void SphererobotArms::place (Position pos, Color *c)
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
bool SphererobotArms::collisionCallback(void *data, dGeomID o1, dGeomID o2) {
  //checks if both of the collision objects are part of the robot
  if( o1 == (dGeomID)sphererobot_space || o2 == (dGeomID)sphererobot_space) {

    // inner space collisions are not treated!

    int i,n;  
    const int N = 100;
    dContact contact[N];
    
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    for (i=0; i<n; i++) {
      if( contact[i].geom.g1 == object[Base].geom || contact[i].geom.g2 == object[Base].geom ){ 
	// only treat collisions with envelop
	contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.mu = 0.1;
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
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
      } 
    }
    return true;
  } else {
    return false;
  }
}


/**
 *Returns the number of motors used by the snake.
 *@return number of motors
 *@author Marcel Kretschmann
 *@version final
 **/
int SphererobotArms::getMotorNumber(){
  return servono;
}

/**
 *Returns the number of sensors used by the robot.
 *@return number of sensors
 *@author Marcel Kretschmann
 *@version final
 **/
int SphererobotArms::getSensorNumber() {
  return sensorno;
}

/**
 *Returns the position of the snake. Here the position of the snake is the position of the first element of the snake.
 *@return Position (x,y,z)
 *@author Marcel Kretschmann
 *@version final
 **/
Position SphererobotArms::getPosition () {
  return Position(dBodyGetPosition ( object[Base].body ));
}

/** returns a vector with the positions of all segments of the robot
    @param vector of positions (of all robot segments) 
    @return length of the list
*/
int SphererobotArms::getSegmentsPosition(vector<Position> &poslist){
  poslist.push_back(Position(dBodyGetPosition ( object[Base].body )));
  poslist.push_back(Position(dBodyGetPosition ( object[Pendular1].body )));
  poslist.push_back(Position(dBodyGetPosition ( object[Pendular2].body )));
  return 3;
}


