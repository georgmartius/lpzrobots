#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include "drawgeom.h"

#include "forcedsphere.h"

Forcedsphere::Forcedsphere(dWorldID w, dSpaceID s, dJointGroupID c, double radius /*=1*/, double max_force /*=1*/,
			   double max_linSpeed /*=5*/, double max_angSpeed /*=5*/):
 AbstractRobot::AbstractRobot(w, s, c, "Forcedsphere"){ 

  created=false;

 //  initial_pos.x=0.0;
//   initial_pos.y=0.0;
//   initial_pos.z=0.0;
	
  this->radius=radius;
  this->max_force=max_force;
  this->max_linSpeed=max_linSpeed;
  this->max_angSpeed=max_angSpeed;

  this->masse=1;

  color.r=2;
  color.g=156/255.0;
  color.b=0/255.0;

  texture = DS_WOOD;

  sensorno=4; 
  motorno=2;  

};

/** sets the textures used for body
 */
void Forcedsphere::setTextures(int body){
  
  texture = body;
  
};

/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void Forcedsphere::setMotors(const motor* motors, int motornumber){

 if (motornumber==motorno){
    dBodyAddForce(body, motors[0]*max_force, 0, 0);
    dBodyAddForce(body, 0, motors[1]*max_force, 0);
 }


};

/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int Forcedsphere::getSensors(sensor* sensors, int sensornumber){
  
  const dReal * linVel;
  const dReal * angVel;

  if (sensornumber==sensorno){
    linVel=dBodyGetLinearVel(body);
    angVel=dBodyGetAngularVel(body);
    for(int i=0; i<2; i++){
      sensors[2*i]=linVel[i];
      sensors[2*i]/=max_linSpeed;
    }
    for(int i=0; i<2; i++){
      sensors[2*i+1]=angVel[i];
      sensors[2*i+1]/=max_angSpeed;
    }
    return sensornumber;
  }

  return 0;

};

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
void Forcedsphere::place(Position pos, Color *c /*= 0*/){

  if (!c==0) {
    color=*c;
  }
  pos.z+=radius; // to put sphere on ground, not in ground
  if (!created){ 
    create(pos);
  }

};

/** returns position of robot 
    @return position robot position in struct Position  
*/
Position Forcedsphere::getPosition(){
  Position pos;
  const dReal* act_pos=dBodyGetPosition(body);
  pos.x=act_pos[0];
  pos.y=act_pos[1];
  pos.z=act_pos[2]; 
  return pos;
};

/**
 * draws sphere
 */
void Forcedsphere::draw(){

    dsSetTexture (texture);    
    dsSetColor (color.r, color.g, color.b);
    dsDrawSphere(dBodyGetPosition(body),dBodyGetRotation(body),radius );

 };


bool Forcedsphere::collisionCallback(void *data, dGeomID o1, dGeomID o2){ return false; /* ? */ }
int Forcedsphere::getSegmentsPosition(vector<Position> &poslist){return 0; }

/** creates sphere at desired position 
    @param pos struct Position with desired position
*/
void Forcedsphere::create(Position pos){
  if (created) {
    destroy();
  }

  dMass m;

  body= dBodyCreate(world);

  dMassSetSphere(&m,1,radius);
  dMassAdjust (&m,masse);
  dBodySetMass (body,&m);

  dBodySetPosition ( body, pos.x, pos.y, pos.z + radius);
  //printf("z-position=%f\n",base_z);
  geom = dCreateSphere ( space, radius );
  
  dGeomSetBody (geom, body);

  created=true;

};

/** destroys ball and space
 */
void Forcedsphere::destroy(){
 
  if (created){
    dGeomDestroy( geom );
    dBodyDestroy( body );
  }
  
  created=false;

};






