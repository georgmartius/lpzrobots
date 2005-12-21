/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   $Log$
 *   Revision 1.7  2005-12-21 00:21:13  robot7
 *   added ir-sensors
 *
 *   Revision 1.6  2005/12/12 13:44:43  martius
 *   barcodesensor is working
 *
 *   Revision 1.5  2005/11/22 15:48:19  robot3
 *   inserted raceground sensors
 *
 *   Revision 1.4  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"

#include "formel1.h"


Formel1::Formel1(const OdeHandle& odeHandle, double size/*=1.0*/, 
	     double force /*=3*/, double speed/*=15*/, bool sphereWheels /*=true*/):
  AbstractRobot::AbstractRobot(odeHandle){ 

  created=false;

  initial_pos.x=0.0;
  initial_pos.y=0.0;
  initial_pos.z=0.0;
  
  // Nimm Zwei color ;-)
  color.r=2;
  color.g=156/255.0;
  color.b=0/255.0;
  
  max_force   = force*size*size;
  this->speed = speed;
  this->sphereWheels = sphereWheels;

  height=size;  

  length=size/2.5; 
  width=size/2; 
  radius=size/6;
  wheelthickness=size/20;
  cmass=10*size;
  wmass=size;  
  sensorno=11; 
  motorno=4;  
  segmentsno=5;
};

/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void Formel1::setMotors(const motor* motors, int motornumber){
  //  double tmp;
  int len = (motornumber < motorno)? motornumber : motorno;
  for (int i=0; i<len; i++){ 
    //    tmp=dJointGetHinge2Param(joint[i],dParamVel2);
    //    dJointSetHinge2Param(joint[i],dParamVel2,tmp + 0.5*(motors[i]*speed-tmp) );       
    dJointSetHinge2Param(joint[i],dParamVel2, -motors[i]*speed);       
    dJointSetHinge2Param (joint[i],dParamFMax2,max_force);
  }
};

/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int Formel1::getSensors(sensor* sensors, int sensornumber){
  int numberOfMotors=4;
  int len=0; // no sensors are sensed yet
  // check that the array sensors is not to small
  numberOfMotors = 
    (sensornumber < numberOfMotors)? sensornumber : numberOfMotors;
  for (int i=0; i<numberOfMotors; i++){ // motor sensing
    sensors[i]=dJointGetHinge2Angle2Rate(joint[i]);
    sensors[i]/=speed;  //scaling
  }
  len += numberOfMotors; // the motors have been sensed
  list<double> trackSensors= trackSensor.get();
    for(list<double>::iterator it = trackSensors.begin();
	it!=trackSensors.end(); ++it) {
      if  (len<sensornumber) {
	sensors[len]=(*it);
	//cout << "sensed: " << (*it) << "\t";
	//sensors[len]=10+len;
	len++; // one more sensor has been sensed
      }
      //cout << "\n";
    }

  len += ray_sensor_bank.get(sensors+len, sensornumber-len);

  return len;
};

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
void Formel1::place(Position pos, Color *c /*= 0*/){
  if (!c==0) {
    color=*c;
  }
  pos.z+=width*0.6; // to put wheels on ground, not in ground
  if (!created){ 
    create(pos);
  } else{
    dBodySetPosition (object[0].body,pos.x ,pos.y           ,pos.z);    
    dBodySetPosition (object[1].body,pos.x+length/2.0 ,pos.y +width*0.5,pos.z);
    dBodySetPosition (object[2].body,pos.x+length/2.0 ,pos.y -width*0.5,pos.z);
    dBodySetPosition (object[3].body,pos.x ,pos.y +width*0.5,pos.z);
    dBodySetPosition (object[4].body,pos.x ,pos.y -width*0.5,pos.z);
  }
};

/** returns position of robot 
    @return position robot position in struct Position  
*/
Position Formel1::getPosition(){
  Position pos;
  const dReal* act_pos=dBodyGetPosition(object[0].body);
  pos.x=act_pos[0];
  pos.y=act_pos[1];
  pos.z=act_pos[2]-radius; // substract wheel radius, because vehicle stands on the ground
  return pos;
};

/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int Formel1::getSegmentsPosition(vector<Position> &poslist){
  Position pos;
  for (int i=0; i<segmentsno; i++){
    const dReal* act_pos = dBodyGetPosition(object[i].body);
    pos.x=act_pos[0];
    pos.y=act_pos[1];
    pos.z=act_pos[2];
    poslist.push_back(pos);
  }   
  return segmentsno;
};  



/**
 * draws the vehicle
 */
void Formel1::draw(){
  dsSetColor (color.r,color.g,color.b); // set color for cylinder
  dsSetTexture (DS_WOOD);
  dsDrawCappedCylinder(dBodyGetPosition(object[0].body),dBodyGetRotation(object[0].body),length, width/2 );
  dsSetColor (1,1,1); // set color for wheels
  // draw wheels
  for (int i=1; i<5; i++) { 
    if(sphereWheels)
      dsDrawSphere (dBodyGetPosition(object[i].body), dBodyGetRotation(object[i].body),radius);
    else
      dsDrawCylinder (dBodyGetPosition(object[i].body), dBodyGetRotation(object[i].body),wheelthickness,radius);
  }

  ray_sensor_bank.draw();
};

void Formel1::mycallback(void *data, dGeomID o1, dGeomID o2){
  Formel1* me = (Formel1*)data;  
  if(isGeomInObjectList(me->object, me->segmentsno, o1) && isGeomInObjectList(me->object, me->segmentsno, o2)){
    return;
  }
}

void Formel1::doInternalStuff(const GlobalData& global){
  // call update the racegroundsensor
  trackSensor.sense(global);
  ray_sensor_bank.reset();
}


bool Formel1::collisionCallback(void *data, dGeomID o1, dGeomID o2){
  //checks if one of the collision objects is part of the robot
  if( o1 == (dGeomID)car_space || o2 == (dGeomID)car_space){

    if(o1 == (dGeomID)car_space) ray_sensor_bank.sense(o2);
    if(o2 == (dGeomID)car_space) ray_sensor_bank.sense(o1);

    dSpaceCollide(car_space, this, mycallback);
    bool colwithme;  
    bool colwithbody;  
    int i,n;  
    const int N = 10;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    for (i=0; i<n; i++){
      colwithbody = false;
      colwithme = false;  
      if( contact[i].geom.g1 == object[0].geom || contact[i].geom.g2 == object[0].geom){
	colwithbody = true;
	colwithme = true;
	// fprintf(stderr,"col with body\n");
      }
      
      if( isGeomInObjectList(object+1, segmentsno-1, contact[i].geom.g1) || 
	  isGeomInObjectList(object+1, segmentsno-1, contact[i].geom.g2)){
	colwithme = true;
	// fprintf(stderr,"col with wheels\n");
      }
      if( colwithme){
	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.slip1 = 0.005;
	contact[i].surface.slip2 = 0.005;
	if(colwithbody){
	  contact[i].surface.mu = 0.1; // small friction of smooth body
	  contact[i].surface.soft_erp = 0.5;
	  contact[i].surface.soft_cfm = 0.001;
	}else{
	  contact[i].surface.mu = 1.1; //large friction
	  contact[i].surface.soft_erp = 0.9;
	  contact[i].surface.soft_cfm = 0.001;
	}
	dJointID c = dJointCreateContact( world, contactgroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	
      }
    }
    return true;
  }
  return false;
}


/** creates vehicle at desired position 
    @param pos struct Position with desired position
*/
void Formel1::create(Position pos){
  if (created) {
    destroy();
  }
  // create car space and add it to the top level space
  car_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (car_space,0);
  
  dMass m;
  // cylinder
  object[0].body = dBodyCreate (world);
  dBodySetPosition (object[0].body,pos.x,pos.y,pos.z);
  dQuaternion q;
  dQFromAxisAndAngle (q,0,1,0,M_PI*0.5);
  dBodySetQuaternion (object[0].body,q);
    
  dMassSetCappedCylinder(&m,1,1,width/2,length);
  dMassAdjust (&m,cmass);
  dBodySetMass (object[0].body,&m);
  object[0].geom = dCreateCCylinder (car_space, width/2,length);
  dGeomSetBody (object[0].geom, object[0].body);

  // wheel bodies
  for (int i=1; i<5; i++) {
    object[i].body = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
    dBodySetQuaternion (object[i].body,q);
    dMassSetSphere (&m,1,radius);
    dMassAdjust (&m,wmass);
    dBodySetMass (object[i].body,&m);
    object[i].geom = dCreateSphere (car_space, radius);
    dGeomSetBody (object[i].geom,object[i].body);
    dBodySetPosition (object[i].body, 
		      pos.x + ((i-1)/2==0?-1:1)*length/2.0, 
		      pos.y + ((i-1)%2==0?-1:1)*(width*0.5+wheelthickness), 
		      pos.z-width*0.6+radius);
  }


  for (int i=0; i<4; i++) {
    joint[i] = dJointCreateHinge2 (world,0);
    dJointAttach (joint[i],object[0].body,object[i+1].body);
    const dReal *a = dBodyGetPosition (object[i+1].body);
    dJointSetHinge2Anchor(joint[i],a[0],a[1],a[2]);
    dJointSetHinge2Axis1 (joint[i],0,0,1);
    dJointSetHinge2Axis2 (joint[i],0,1,0);
  }
  for (int i=0; i<4; i++) {
    // set stops to make sure wheels always stay in alignment
    dJointSetHinge2Param (joint[i],dParamLoStop,0);
    dJointSetHinge2Param (joint[i],dParamHiStop,0);
  }

  // initialise racegroundsensors
  trackSensor.init(object[0].body);


  // set up ir-sensors
  IRSensor *p_ir_sensor = NULL;
  

  dMatrix3 rot;
  dRFromAxisAndAngle(rot, 0.0, 0.0, 1.0, 0.0);

  ray_sensor_bank.init(car_space, RaySensor::drawAll);


  dRFromAxisAndAngle(rot, 1.0, 0.0, 0.0, 0.0);
  p_ir_sensor = new IRSensor();
  ray_sensor_bank.registerSensor(p_ir_sensor,
				 object[0].body,
				 Position(0.0, 0.0, 0.5),
				 rot,
				 2.0);

  dRFromAxisAndAngle(rot, 1.0, 0.0, 0.0, - M_PI / 8.0);
  p_ir_sensor = new IRSensor();
  ray_sensor_bank.registerSensor(p_ir_sensor,
				 object[0].body,
				 Position(0.0, 0.3, 0.5),
				 rot,
				 2.0);


  dRFromAxisAndAngle(rot, 1.0, 0.0, 0.0, + M_PI / 8.0);
  p_ir_sensor = new IRSensor();
  ray_sensor_bank.registerSensor(p_ir_sensor,
				 object[0].body,
				 Position(0.0, -0.3, 0.5),
				 rot,
				 2.0);


  dRFromAxisAndAngle(rot, 1.0, 0.0, 0.0, - M_PI / 2.0);
  p_ir_sensor = new IRSensor();
  ray_sensor_bank.registerSensor(p_ir_sensor,
				 object[0].body,
				 Position(0.0, 0.0, 0.5),
				 rot,
				 2.0);


  dRFromAxisAndAngle(rot, 1.0, 0.0, 0.0, + M_PI / 2.0);
  p_ir_sensor = new IRSensor();
  ray_sensor_bank.registerSensor(p_ir_sensor,
				 object[0].body,
				 Position(0.0, 0.0, 0.5),
				 rot,
				 2.0);

  created=true;
}; 


/** destroys vehicle and space
 */
void Formel1::destroy(){
  if (created){
    dSpaceDestroy(car_space);
    for (int i=0; i<segmentsno; i++){
      dBodyDestroy(object[i].body);
      dGeomDestroy(object[i].geom);
    }
  }
  created=false;
}






