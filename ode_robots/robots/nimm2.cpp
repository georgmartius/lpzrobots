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
 *   Revision 1.21.4.2  2005-11-15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.21.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.21  2005/11/08 11:35:56  martius
 *   removed check for sensorbank because rays are disabled now
 *
 *   Revision 1.20  2005/11/04 14:43:27  martius
 *   added GPL
 *
 *                                                                 *
 ***************************************************************************/

#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <assert.h>

#include "simulation.h"
#include "drawgeom.h"

#include "nimm2.h"
#include "irsensor.h"


Nimm2::Nimm2(const OdeHandle& odehandle, const Nimm2Conf& conf):
  OdeRobot(odehandle, "Nimm2"), conf(conf) { 

  created=false;
  
  //Nimm2 color ;-)
  color.r=2;
  color.g=156/255.0;
  color.b=0/255.0;
  
  bodyTexture  = DS_WOOD;
  wheelTexture = DS_WOOD;

  max_force   = conf.force*conf.size*conf.size;

  height=conf.size;

  width=conf.size/2; 
  radius=conf.size/4+conf.size/600;
  wheelthickness=conf.size/20;
  cmass=8*conf.size;  
  wmass=conf.size;  
  if(conf.singleMotor){
    sensorno=1; 
    motorno=1;  
  } else {
    sensorno=2; 
    motorno=2;  
  }
  segmentsno=3;

  if (conf.cigarMode){
    length=conf.size*2.0;         // long body
    wheeloffset= -length/4;  // wheels at the end of the cylinder, and the opposite endas the bumper
    number_bumpers=2;        // if wheels not at center only one bumper
    cmass=4*conf.size;
    max_force   = 2*conf.force*conf.size*conf.size;
  }
  else{
    length=conf.size/3;    // short body 
    wheeloffset=0.0;  // wheels at center of body
    number_bumpers=2; // if wheels at center 2 bumpers (one at each end)
  }

  sensorno+= conf.irFront * 2;
};


/** sets the textures used for body and wheels
 */
void Nimm2::setTextures(int body, int wheels){
  bodyTexture = body;
  wheelTexture = wheels;
}


/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void Nimm2::setMotors(const motor* motors, int motornumber){
  assert(created);
  assert(motornumber == motorno);
  if(conf.singleMotor){
    dJointSetHinge2Param (joint[0],dParamVel2, motors[0]*conf.speed);       
    dJointSetHinge2Param (joint[0],dParamFMax2,max_force);
    dJointSetHinge2Param (joint[1],dParamVel2, motors[0]*conf.speed);       
    dJointSetHinge2Param (joint[1],dParamFMax2,max_force);    
  } else {
    for (int i=0; i<2; i++){ 
      dJointSetHinge2Param (joint[i],dParamVel2, motors[i]*conf.speed);       
      dJointSetHinge2Param (joint[i],dParamFMax2,max_force);
    }
  }
};

/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int Nimm2::getSensors(sensor* sensors, int sensornumber){
  assert(created);
  
  int len = conf.singleMotor ? 1 : 2;
  for (int i=0; i<len; i++){
    sensors[i]=dJointGetHinge2Angle2Rate(joint[i]);
    sensors[i]/=conf.speed;  //scaling
  }
  len += irSensorBank.get(sensors+len, sensornumber-len);
  return len;
};

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
void Nimm2::place(Position pos, Color *c /*= 0*/){

  if (!c==0) {
    color=*c;
  }
  pos.z+=radius; // to put wheels on ground, not in ground
  if (!created){ 
    create(pos);
  }
  else{
    dBodySetPosition (object[1].body,pos.x ,pos.y +width*0.5,pos.z);
    dBodySetPosition (object[2].body,pos.x ,pos.y -width*0.5,pos.z);
    dBodySetPosition (object[0].body,pos.x ,pos.y           ,pos.z);    
  }
};


/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int Nimm2::getSegmentsPosition(vector<Position> &poslist){
  assert(created);
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

void Nimm2::draw(){
  assert(created);
  // draw body
  dsSetColor (color.r,color.g,color.b); // set color for body and bumper
  dsSetTexture (bodyTexture);
  drawGeom(object[0].geom,0,0);

  // draw bumper
  if (conf.bumper){
    for (int i=0; i<number_bumpers; i++){
      drawGeom(bumper[i].transform,0,0);
    }
  }

  // draw wheels
  dsSetColor (1,1,1); // set color for wheels
  dsSetTexture (wheelTexture);
  for (int i=1; i<3; i++) { 
    if(conf.sphereWheels)
      drawGeom(object[i].geom,0,0);
    else
      dsDrawCylinder (dBodyGetPosition(object[i].body), dBodyGetRotation(object[i].body),wheelthickness,radius);
  }
  
  irSensorBank.draw();
};

void Nimm2::mycallback(void *data, dGeomID o1, dGeomID o2){
  // Nimm2* me = (Nimm2*)data;  
  // o1 and o2 are member of the space

  // we ignore the collisions
}

bool Nimm2::collisionCallback(void *data, dGeomID o1, dGeomID o2){
  //checks if one of the collision objects is part of the robot
  bool colwithme = false;  
  if( o1 == (dGeomID)car_space || o2 == (dGeomID)car_space ){
    if(o1 == (dGeomID)car_space) irSensorBank.sense(o2);
    if(o2 == (dGeomID)car_space) irSensorBank.sense(o1);

    bool colwithbody;  
    int i,n;  
    const int N = 10;
    dContact contact[N];
    //    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    for (i=0; i<n; i++){
      colwithme = true; // there is at least one collision with some part of the robot (not sensors)
      colwithbody = false;
      if( contact[i].geom.g1 == object[0].geom || contact[i].geom.g2 == object[0].geom ||
	  contact[i].geom.g1 == bumper[0].transform || contact[i].geom.g2 == bumper[0].transform ||
	  contact[i].geom.g1 == bumper[1].transform || contact[i].geom.g2 == bumper[1].transform ){
	
	colwithbody = true;
	//fprintf(stderr,"col with body\n");
      }
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.slip1 = 0.005;
      contact[i].surface.slip2 = 0.005;
      if(colwithbody){
	contact[i].surface.mu = 0.1; // small friction of smooth body
	contact[i].surface.soft_erp = 0.9;
	contact[i].surface.soft_cfm = 0.001;
      }else{
	contact[i].surface.mu = 1.1; //large friction
	contact[i].surface.soft_erp = 0.9;
	contact[i].surface.soft_cfm = 0.001;
      }
      dJointID c = dJointCreateContact( world, contactgroup, &contact[i]);
      dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
    }        
  }
  return colwithme;
}

void Nimm2::doInternalStuff(const GlobalData& globalData){
  // dSpaceCollide(car_space, this, mycallback); // checks collisions in the car_space only (not needed)
  irSensorBank.reset();
}

/** creates vehicle at desired position 
    @param pos struct Position with desired position
*/
void Nimm2::create(Position pos){
  if (created) {
    destroy();
  }
  // create car space and add it to the top level space
  car_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (car_space,0);

  dMass m;
  // cylinder
  object[0].body = dBodyCreate (world);
  dQuaternion q;
  dQFromAxisAndAngle (q,0,1,0,M_PI*0.5);
  dBodySetQuaternion (object[0].body,q);
  dBodySetPosition (object[0].body, pos.x, pos.y, pos.z);
    
  dMassSetCappedCylinder(&m,1,1,width/2,length);
  dMassAdjust (&m,cmass);
  dBodySetMass (object[0].body,&m);
  object[0].geom = dCreateCCylinder (car_space, width/2,length);
  dGeomSetBody (object[0].geom, object[0].body);

  irSensorBank.init(car_space, RaySensor::drawAll);

  // bumper
  if (conf.bumper){
    dMatrix3 R;
    for (int i=0; i<number_bumpers; i++){
      bumper[i].transform = dCreateGeomTransform(car_space);
      dGeomTransformSetInfo(bumper[i].transform, 1);
      dGeomTransformSetCleanup(bumper[i].transform, 1);
      bumper[i].geom = dCreateCCylinder (0, width/4,2*radius+width/2);
      dGeomTransformSetGeom (bumper[i].transform, bumper[i].geom);

      dRFromEulerAngles(R, M_PI/2,0,0);
      dGeomSetRotation (bumper[i].geom, R);
      dGeomSetPosition (bumper[i].geom, 0, 0, + pow(-1.0,i)*(length/2) );
      dGeomSetBody (bumper[i].transform, object[0].body);
    }
  }
  // wheel bodies
  for (int i=1; i<3; i++) {
    object[i].body = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
    dBodySetQuaternion (object[i].body,q);
    dMassSetSphere (&m,1,radius);
    dMassAdjust (&m,wmass);
    dBodySetMass (object[i].body,&m);
    object[i].geom = dCreateSphere (car_space, radius);
    dGeomSetBody (object[i].geom,object[i].body);
  }
  dBodySetPosition (object[1].body, pos.x+wheeloffset, pos.y + width*0.5+wheelthickness, pos.z);
  dBodySetPosition (object[2].body, pos.x+wheeloffset, pos.y - width*0.5-wheelthickness, pos.z);


  for (int i=0; i<2; i++) {
    joint[i] = dJointCreateHinge2 (world,0);
    dJointAttach (joint[i],object[0].body,object[i+1].body);
    const dReal *a = dBodyGetPosition (object[i+1].body);
    dJointSetHinge2Anchor(joint[i],a[0],a[1],a[2]);
    dJointSetHinge2Axis1 (joint[i],0,0,1);
    dJointSetHinge2Axis2 (joint[i],0,-1,0);
  }
  for (int i=0; i<2; i++) {
    // set stops to make sure wheels always stay in alignment
    dJointSetHinge2Param (joint[i],dParamLoStop,0);
    dJointSetHinge2Param (joint[i],dParamHiStop,0);
  }
//   // create car space and add it to the top level space
//   car_space = dSimpleSpaceCreate (space);
//   dSpaceSetCleanup (car_space,0);
//   for (int i=0; i<3; i++){
//     dSpaceAdd (car_space,object[i].geom);
//   }

  if (conf.irFront){
    for(int i=-1; i<2; i+=2){
      IRSensor* sensor = new IRSensor();
      dMatrix3 R;      
      dRFromEulerAngles(R, i*M_PI/10,0,0);      
      irSensorBank.registerSensor(sensor, object[0].body, Position(0,i*width/10,length/2 + width/2 - width/60 ), R, 2);
    }
  }
  // TODO Back , Side

  created=true;
}; 


/** destroys vehicle and space
 */
void Nimm2::destroy(){
  if (created){
    for (int i=0; i<segmentsno; i++){
      dBodyDestroy(object[i].body);
      dGeomDestroy(object[i].geom);     
    }
    // Todo: delete bumpers
    dSpaceDestroy(car_space);
  }
  created=false;
}






