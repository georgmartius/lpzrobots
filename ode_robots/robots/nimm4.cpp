#include <assert.h>
// include drawstuff and ode stuff
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

// include simulation environmet stuff
#include "simulation.h"

// include header file
#include "nimm4.h"

// constructor:
// - size of robot, maximal used force and speed factor are adjustable
// - sphereWheels switches between spheres or wheels as wheels
//   (wheels are only drawn, collision handling is always with spheres)
Nimm4::Nimm4(const OdeHandle& odeHandle, double size/*=1.0*/, 
	     double force /*=3*/, double speed/*=15*/, bool sphereWheels /*=true*/):
  // callind AbstractRobot with name of the actual robot
  AbstractRobot::AbstractRobot(odeHandle, "Nimm4"){ 
  
  // robot is not created till now
  created=false;

  // choose color (here the color of the "Nimm Zwei" candy is used, 
  // where the name of the Nimm2 and Nimm4 robots comes from ;-)
  color.r=2;
  color.g=156/255.0;
  color.b=0/255.0;
  
  // body and wheel texture set to wood
  bodyTexture  = DS_WOOD;
  wheelTexture = DS_WOOD;
  
  // maximal used force is calculated from the forece factor and size given to the constructor
  max_force   = force*size*size;
  
  // speed and type of wheels are set
  this->speed = speed;
  this->sphereWheels = sphereWheels;

  
  height=size;  

  length=size/2.5; // length of body
  width=size/2;  // radius of body
  radius=size/6; // wheel radius
  wheelthickness=size/20; // thickness of the wheels (if wheels used, no spheres)
  cmass=8*size;  // mass of the body
  wmass=size;    // mass of the wheels
  sensorno=4;    // number of sensors
  motorno=4;     // number of motors
  segmentsno=5;  // number of segments of the robot
};

/** sets the textures used for body and wheels
 */
// here the textures can be set from outside
void Nimm4::setTextures(int body, int wheels){
  bodyTexture = body;
  wheelTexture = wheels;
}


/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void Nimm4::setMotors(const motor* motors, int motornumber){
  assert(created); // robot must exist
  // the number of controlled motors is minimum of
  // "number of motorcommands" (motornumber) and 
  // "number of motors inside the robot" (motorno)
  int len = (motornumber < motorno)? motornumber : motorno;

  // for each motor the motorcommand (between -1 and 1) multiplied with speed
  // is set and the maximal force to realize this command are set
  for (int i=0; i<len; i++){ 
    dJointSetHinge2Param(joint[i],dParamVel2, motors[i]*speed);       
    dJointSetHinge2Param (joint[i],dParamFMax2,max_force);
  }

  // another possibility is to set half of the difference between last set speed
  // and the actual desired speed as new speed; max_force is also set
  /*
    double tmp;
    int len = (motornumber < motorno)? motornumber : motorno;
    for (int i=0; i<len; i++){ 
      tmp=dJointGetHinge2Param(joint[i],dParamVel2);
      dJointSetHinge2Param(joint[i],dParamVel2,tmp + 0.5*(motors[i]*speed-tmp) );       
      dJointSetHinge2Param (joint[i],dParamFMax2,max_force);
    }
  */
};

/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int Nimm4::getSensors(sensor* sensors, int sensornumber){
  assert(created); // robot must exist

  // the number of sensors to read is the minimum of
  // "number of sensors requested" (sensornumber) and 
  // "number of sensors inside the robot" (sensorno)
  int len = (sensornumber < sensorno)? sensornumber : sensorno;

  // for each sensor the anglerate of the joint is red and scaled with 1/speed 
  for (int i=0; i<len; i++){
    sensors[i]=dJointGetHinge2Angle2Rate(joint[i]);
    sensors[i]/=speed;  //scaling
  }
  // the number of red sensors is returned 
  return len;
};

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
void Nimm4::place(Position pos, Color *c /*= 0*/){
  // if a color is given set it 
  if (!c==0) {
    color=*c;
  }
  // the position of the robot is the center of the body (without wheels)
  // to set the vehicle on the ground when the z component of the position is 0
  // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
  pos.z+=width*0.6; 
  
  // if robot does not exist reate it at the given position
  if (!created){ 
    create(pos);
  } else{
    // if robot exists set the position of the body 
    dBodySetPosition (object[0].body,pos.x, pos.y, pos.z);    
    // and set the position of the wheels
    for(int i=1; i<5; i++){
      dBodySetPosition (object[i].body, 
			pos.x + ((i-1)/2==0?-1:1)*length/2.0, 
			pos.y + ((i-1)%2==0?-1:1)*(width*0.5+wheelthickness), 
			// center of the wheels should be at height radius,
			// therefore subtract height of body (width*0.6) and add radius
			pos.z - width*0.6 +radius); 
    }
  }
};

/** returns position of robot 
    @return position robot position in struct Position  
*/
Position Nimm4::getPosition(){
  assert(created); // robot must exist
  Position pos;
  // get actual position
  const dReal* act_pos=dBodyGetPosition(object[0].body);
  pos.x=act_pos[0];
  pos.y=act_pos[1];
  // substract height of body (width*0.6) because z should be 0 if vehicle stands on the ground
  pos.z=act_pos[2]-width*0.6; 
  // return actual corrected position
  return pos;
};

/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int Nimm4::getSegmentsPosition(vector<Position> &poslist){
  assert(created); // robot must exist
  Position pos;
  // for all segments (or elements) of the body
  // get the actual position, store it in position struct
  // and push it in list
  for (int i=0; i<segmentsno; i++){
    const dReal* act_pos = dBodyGetPosition(object[i].body);
    pos.x=act_pos[0];
    pos.y=act_pos[1];
    pos.z=act_pos[2];
    poslist.push_back(pos);
  }   
  // return the number of segment positions in the list
  return segmentsno;
};  



/**
 * draws the vehicle
 */
void Nimm4::draw(){
  assert(created); // robot must exist
  dsSetColor (color.r,color.g,color.b); // set color for cylinder
  dsSetTexture (bodyTexture); // set texture for cylinder

  // draw capped cylinder (forr the body)
  // at position of the body
  // with rotation matrix of the body
  // and with length and radius(=width/2) of the body
  dsDrawCappedCylinder(dBodyGetPosition(object[0].body),
		       dBodyGetRotation(object[0].body),length, width/2 );

  dsSetColor (1,1,1); // set color for wheels
  dsSetTexture (wheelTexture); // set texture for wheels
  // for all wheels
  for (int i=1; i<5; i++) { 
    if(sphereWheels)
      // if spheres as wheels draw spheres 
      // at position of the wheels
      // with rotation matrix of the wheel
      // and with desired radius
      dsDrawSphere (dBodyGetPosition(object[i].body), 
		    dBodyGetRotation(object[i].body),radius);
    else
      // if wheels(cylinder slice) as wheels draw cylinders
      // at position of the wheels
      // with rotation matrix of the wheel
      // and with thickness (or length of the cylinder) and desired radius
      dsDrawCylinder (dBodyGetPosition(object[i].body), 
		      dBodyGetRotation(object[i].body),wheelthickness,radius);
  }
};

/** things for collision handling inside the space of the robot can be done here
 */
void Nimm4::mycallback(void *data, dGeomID o1, dGeomID o2){
  // do collisions handling for collisions between parts inside the space of the robot here
  // this has no meaning for this robot, because collsions between wheels and body are ignored
  // but if parts of the robot can move against each other this is important

  // the follwing (not active) code part can be used to check if objects which had collisions 
  // are inside the list of objects of the robot
  /*  Nimm4* me = (Nimm4*)data;  
  if(isGeomInObjectList(me->object, me->segmentsno, o1) 
     && isGeomInObjectList(me->object, me->segmentsno, o2)){
    return;
  }
  */
}

/** this function is called in each timestep. It should perform robot-internal checks, 
    like space-internal collision detection, sensor resets/update etc.
    @param GlobalData structure that contains global data from the simulation environment
*/
void Nimm4::doInternalStuff(const GlobalData& global){}

/** checks for internal collisions and treats them. 
 *  In case of a treatment return true (collision will be ignored by other objects 
 *  and the default routine)  else false (collision is passed to other objects and 
 *  (if not treated) to the default routine).
 */
bool Nimm4::collisionCallback(void *data, dGeomID o1, dGeomID o2){
  assert(created); // robot must exist

  // checks if one of the collision objects is part of thee space the robot is in 
  // and therefore part of the robot
  if( o1 == (dGeomID)car_space || o2 == (dGeomID)car_space){
    // if the space is involved check for collisions between parts inside the space
    // this has no meaning here, because collsions between wheels and body are ignored
    // but if parts of the robot can move against each other this is important
    dSpaceCollide(car_space, this, mycallback);

    bool colwithme;   // for collision with some part of the vehicle
    bool colwithbody; // for collision with the (main) body
    int i,n;  
    const int N = 10;
    dContact contact[N];
    // extract collision points between the two objects that intersect
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    // for each collision point
    for (i=0; i<n; i++){
      // collisions set to false
      colwithbody = false; 
      colwithme = false;  
      // if there is a collision with the body both bools have to be set to true
      if( contact[i].geom.g1 == object[0].geom || contact[i].geom.g2 == object[0].geom){
	colwithbody = true;
	colwithme = true;
      }
      // if there is a collision with one of the wheels only colwithme has to be set true
      if( isGeomInObjectList(object+1, segmentsno-1, contact[i].geom.g1) || 
	  isGeomInObjectList(object+1, segmentsno-1, contact[i].geom.g2)){
	colwithme = true;
      }
      if( colwithme){ // if collision set the contact parameters
	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.slip1 = 0.005;
	contact[i].surface.slip2 = 0.005;
	if(colwithbody){ // if collision with body set small friction
	  contact[i].surface.mu = 0.1; // small friction of smooth body
	  contact[i].surface.soft_erp = 0.5;
	  contact[i].surface.soft_cfm = 0.001;
	}else{  // if collision with the wheels set large friction to give wheels grip on the ground
	  contact[i].surface.mu = 1.1; //large friction
	  contact[i].surface.soft_erp = 0.9;
	  contact[i].surface.soft_cfm = 0.001;
	}
	// create a joint in the world with the properties set above
	// (the joint must be put in group "contactgroup", which is deleted 
	// after each simulation step, see ode documentation)
	dJointID c = dJointCreateContact( world, contactgroup, &contact[i]);
	// attach the intersecting objects to the joint
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	
      }
    }
    //if collision handled return true
    return true;
  }
  //if collision not handled return false
  return false;
}


/** creates vehicle at desired position 
    @param pos struct Position with desired position
*/
void Nimm4::create(Position pos){
  if (created) {  // if robot exists destroy it
    destroy();
  }
  // create car space and add it to the top level space
  car_space = dSimpleSpaceCreate (space);

  dMass m;
  // create (main) body
  object[0].body = dBodyCreate (world); // create body in the world
  dBodySetPosition (object[0].body,pos.x,pos.y,pos.z); // set body to the desired position
  dQuaternion q;
  dQFromAxisAndAngle (q,0,1,0,M_PI*0.5); // set rotation to quaternion
  dBodySetQuaternion (object[0].body,q); // rotate body (here by 90° around the y-axis)
    
  // Set the mass parameters to represent a capped cylinder of the given parameters and density, with 
  // the center of mass at (0,0,0) relative to the body  
  dMassSetCappedCylinder(&m,1,1,width/2,length); 
  dMassAdjust (&m,cmass); // set mass value to cmass (=mass of the (main) body)
  dBodySetMass (object[0].body,&m); //assign the mass to the body
  // create geom for the body in the space of the robot
  // the geom defines the outer shape of the body, it is used for the collsision detection
  // in this case the geom is a capped cylinder with radius width/2 and length length
  object[0].geom = dCreateCCylinder (car_space, width/2,length); 
  dGeomSetBody (object[0].geom, object[0].body); // geom is assigned to body

  // create wheel bodies
  for (int i=1; i<5; i++) {
    object[i].body = dBodyCreate (world);  // create body in the world
    dQuaternion q;                         
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5); // set rotation to quaternion
    dBodySetQuaternion (object[i].body,q); // rotate body (here by 90° around the x-axis)
    // Set the mass parameters to represent a sphere of the given radius and density (=1), with 
    // the center of mass at (0,0,0) relative to the body  
    dMassSetSphere (&m,1,radius);
    dMassAdjust (&m,wmass);           // set mass value to wmass (=mass of wheel )
    dBodySetMass (object[i].body,&m); //assign the mass to the body
    // create geom for wheel in the space of the robot
    // in this case the geom is a sphere with radius radius
    object[i].geom = dCreateSphere (car_space, radius);
    dGeomSetBody (object[i].geom,object[i].body); // geom is assigned to body
    // set body to the desired position 
    // (you have to find out (calculate or test) where these positions are)
    dBodySetPosition (object[i].body, 
		      pos.x + ((i-1)/2==0?-1:1)*length/2.0, 
		      pos.y + ((i-1)%2==0?-1:1)*(width*0.5+wheelthickness), 
		      pos.z-width*0.6+radius);
  }

  // generate 4 joints to connect the wheels to the body
  for (int i=0; i<4; i++) {
    joint[i] = dJointCreateHinge2 (world,0); // create joint in the world, but in no group(2.parameter=0)
    dJointAttach (joint[i],object[0].body,object[i+1].body); // attach joint to body and wheel
    const dReal *a = dBodyGetPosition (object[i+1].body);    // get position of the wheel
    dJointSetHinge2Anchor(joint[i],a[0],a[1],a[2]);          // set the joint anchor to this position
    dJointSetHinge2Axis1 (joint[i],0,0,1);   // set the joint axis
    dJointSetHinge2Axis2 (joint[i],0,1,0);
  }
  for (int i=0; i<4; i++) {
    // set stops to make sure wheels always stay in alignment
    dJointSetHinge2Param (joint[i],dParamLoStop,0);
    dJointSetHinge2Param (joint[i],dParamHiStop,0);
  }

  created=true; // robot is created
}; 


/** destroys vehicle and space
 */
void Nimm4::destroy(){
  if (created){
    dSpaceDestroy(car_space); // destroy space
    for (int i=0; i<segmentsno; i++){
      dBodyDestroy(object[i].body); // destroy bodies and geoms
      dGeomDestroy(object[i].geom);
    }
  }
  created=false; // robot does not exist (anymore)
}






