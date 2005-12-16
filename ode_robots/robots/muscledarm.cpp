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
 *   Revision 1.1.4.4  2005-12-16 16:36:04  fhesse
 *   manual control via keyboard
 *   setMotors via dJointAddSliderForce
 *
 *   Revision 1.1.4.3  2005/11/24 16:15:57  fhesse
 *   moved from main branch, sensors improved
 *
 *   Revision 1.3  2005/11/17 16:29:24  fhesse
 *   initial version
 *
 *   Revision 1.2  2005/11/15 12:35:19  fhesse
 *   muscles drawn as muscles, sphere drawn on tip of lower arm
 *
 *   Revision 1.1  2005/11/11 15:37:06  fhesse
 *   preinitial version
 *                                                                 *
 *                                                                         *
 ***************************************************************************/

#include <iostream>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include "drawgeom.h"
#include "mathutils.h"

#include "muscledarm.h"

#include <assert.h>

MuscledArm::MuscledArm(const OdeHandle& odeHandle, const MuscledArmConf& conf):
  OdeRobot(odeHandle), conf(conf){ 

  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile$",
			      "$Revision$");
  created=false;

  factorMotors=0.1;
  factorSensors=4.0;//20.0;
  damping=1;

  /*
  initial_pos.x=0.0;
  initial_pos.y=0.0;
  initial_pos.z=0.0;
  */

  sensorno=0;
  if (conf.jointAngleSensors)
    sensorno+=2;
  if (conf.jointAngleRateSensors)
    sensorno+=2;
  if (conf.MuscleLengthSensors)
    sensorno+=6;

  motorno=6;  
  print=0;

 
  color.r=1;
  color.g=1;
  color.b=0;
  mainTexture=DS_WOOD;

  max_l=0;
  max_r=0;
  min_l=10; 
  min_r=10;

  for (int i=0; i<NUMParts; i++){
    old_dist[i].x=0;
    old_dist[i].y=0;
    old_dist[i].z=0;
  }

  for (int i=0; i<6; i++){
    force_[i]=0;
  }

};

/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void MuscledArm::setMotors(const motor* motors, int motornumber){
  

  /*
    // old execution of motorvalues
  const dReal *p1;
  const dReal *p2;
  for (int i=mainMuscle11; i<smallMuscle42; i+=2){
    p1 = dBodyGetPosition (object[i].body);
    p2 = dBodyGetPosition (object[i+1].body);
    
    Position dist;
    //distance between slider joint elements
    dist.x=p2[0]-p1[0];
    dist.y=p2[1]-p1[1];
    dist.z=p2[2]-p1[2];
    
    Position force;
    //calculating motor force
    force=dist*factorMotors*motors[(int)(i*0.5)-1];
    
    dBodyAddForce (object[i].body, force.x, force.y, force.z);
    dBodyAddForce (object[i+1].body, -force.x, -force.y, -force.z);
    */

  
  if (!conf.manualMode) { 
    // mode where attached controller is used

    /* 
    // execution of motor values without slider sensors factor
    for (int i=sliderJointM1; i<=sliderJointS4; i++){
    // starting with motor[0]
    dJointAddSliderForce(joint[i], dJointGetSliderPosition (joint[i]) * factorMotors * motors[i-sliderJointM1]);
    }
    */
    
    // execution of motor values with slider sensors factor
    double force;
    for(int i=sliderJointM1; i<= sliderJointM2; i++){
      force=factorMotors * motors[i-sliderJointM1] * dJointGetSliderPosition (joint[i]) * 5 / (3*SIDE);
      dJointAddSliderForce(joint[i], force);
    }
    for(int i=sliderJointS1; i<= sliderJointS2; i++){
      force=factorMotors * motors[i-sliderJointM1] * dJointGetSliderPosition (joint[i]) * 4 / (SIDE);
      dJointAddSliderForce(joint[i], force);
    }
    for(int i=sliderJointS3; i<= sliderJointS4; i++){
      force=factorMotors * motors[i-sliderJointM1] * dJointGetSliderPosition (joint[i]) / (0.5*SIDE);
      dJointAddSliderForce(joint[i], force);
    }
  }else{
    // manual control (see command in main.cpp)
    for(int i=sliderJointM1; i<= sliderJointS4; i++){
      dJointAddSliderForce(joint[i], force_[i-sliderJointM1]);
    }
  }
};


/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int MuscledArm::getSensors(sensor* sensors, int sensornumber){
  int sens_values = (sensornumber < sensorno)? sensornumber : sensorno;
  int written=0;
  
  if ((conf.jointAngleSensors) && ((sens_values-2)>-1)){
    /* hinge joint between upper arm and fixed body: hingeJointFUA (=1) */
    /* hinge joint between 2 main arms: hingeJointUALA (=2) */
    for (int i=hingeJointFUA; i<hingeJointUALA+1; i++){
      sensors[written]=factorSensors*dJointGetHingeAngle(joint[i]);
      written++;
    }
  }
  if ((conf.jointAngleRateSensors) && ((sens_values -written -2)>-1)){
    for (int i=hingeJointFUA; i<hingeJointUALA+1; i++){
      sensors[written]=factorSensors*dJointGetHingeAngleRate(joint[i]);
      written++;
    }
  }
  if ((conf.MuscleLengthSensors) && ((sens_values -written -6)>-1)){
    for(int i=sliderJointM1; i<= sliderJointM2; i++){
      sensors[written] = dJointGetSliderPosition (joint[i]) * 5 / (3*SIDE);
      written++;
    }
    for(int i=sliderJointS1; i<= sliderJointS2; i++){
      sensors[written] = dJointGetSliderPosition (joint[i]) * 4 / (SIDE);
      written++;
    }
    for(int i=sliderJointS3; i<= sliderJointS4; i++){
      sensors[written] = dJointGetSliderPosition (joint[i]) / (0.5*SIDE);
      written++;
    }

//     double mlen;
//     const dReal *p1;
//     const dReal *p2;
//     for (int i=mainMuscle11; i<smallMuscle42; i+=2){

//       p1 = dBodyGetPosition (object[i].body);
//       p2 = dBodyGetPosition (object[i+1].body);
//       mlen=0;
//       //distance between slider joint elements
//       mlen+=(p2[0]-p1[0])*(p2[0]-p1[0]);
//       mlen+=(p2[1]-p1[1])*(p2[1]-p1[1]);
//       mlen+=(p2[2]-p1[2])*(p2[2]-p1[2]);
//       mlen=pow(mlen, 1.0/3.0);
//       sensors[written]=mlen;
//       written++;
  }
  return written;
};

/** sets the vehicle to position pos, sets color to c, and create robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
void MuscledArm::place(Position pos, Color *c /*= 0*/){
  
 if (!c==0) {
    color=*c;
  }

  create(pos);
  /*
  if (!created){ 
    create(pos);
  }
  else{
    dBodySetPosition (object[1].body,pos.x ,pos.y +width*0.5,pos.z);
    dBodySetPosition (object[2].body,pos.x ,pos.y -width*0.5,pos.z);
    dBodySetPosition (object[0].body,pos.x ,pos.y           ,pos.z);    
  }
  */
};

// /** returns position of robot 
//     @return position robot position in struct Position  
// */
// Position MuscledArm::getPosition(){
//   Position pos;
  
//   //difference between center of arm and tip of arm 
//   dReal s[3];
//   s[0]=0;
//   s[1]=0;
//   s[2]=-SIDE*2;
//   const double* R=dBodyGetRotation(object[lowerArm].body);

//   //rotation of difference vector
//   pos.x=s[0]*R[0]+s[1]*R[1]+s[2]*R[2];
//   pos.y=s[0]*R[4]+s[1]*R[5]+s[2]*R[6];
//   pos.z=s[0]*R[8]+s[1]*R[9]+s[2]*R[10];
//   // adding (rotated) difference vector to actual position of arm center
//   // -> leading to actual position of the tip of the arm
//   pos = pos + Position(dBodyGetPosition(object[lowerArm].body));

//   return pos;
// };

/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int MuscledArm::getSegmentsPosition(vector<Position> &poslist){
  assert(created);
  for (int i=0; i<NUMParts; i++){
    Position pos (dBodyGetPosition(object[i].body));
    poslist.push_back(pos);
  }   
  return NUMParts;
};  


/**
 * draws the arm
 */
void MuscledArm::draw(){

dsSetTexture (mainTexture);
  dsSetColor (color.r,color.g,color.b); // fixedBody
  drawGeom(object[0].geom, 0, 0);
  dsSetColor (1,1,0); // upperArm 
  drawGeom(object[1].geom, 0, 0);
  dsSetColor (1,1,0); // lowerArm
  drawGeom(object[2].geom, 0, 0);
  dsSetColor (1,0,0); // hand
  drawGeom(object[hand].geom, 0, 0);

  // draw existing geoms
   for(int i = 0; i<6; i++){
     dsSetColor (1,1,0);
     if (force_[i]>0) dsSetColor (1,0,0);
     if (force_[i]<0) dsSetColor (0,0,1);
     drawGeom(object[mainMuscle11+i*2].geom, 0, 0);    
     drawGeom(object[mainMuscle11+i*2+1].geom, 0, 0);    
   }

  // draw sphere/hand (only drawing no body)
  //  if (conf.drawSphere){  
//     //difference between center of arm and center of sphere=-halbe hoehe der arm-box 
//     dReal s[3];
//     s[0]=0;
//     s[1]=0;
//     s[2]=-SIDE*2;
//     const double* R=dBodyGetRotation(object[lowerArm].body);
//     dReal new_pos[3];
//     //rotation of difference vector
//     new_pos[0]=s[0]*R[0]+s[1]*R[1]+s[2]*R[2];
//     new_pos[1]=s[0]*R[4]+s[1]*R[5]+s[2]*R[6];
//     new_pos[2]=s[0]*R[8]+s[1]*R[9]+s[2]*R[10];
//     // adding (rotated) difference vector to actual position of arm center
//     // -> leading to actual position of sphere
//     new_pos[0]+=dBodyGetPositionAll(object[lowerArm].body,1);
//     new_pos[1]+=dBodyGetPositionAll(object[lowerArm].body,2);
//     new_pos[2]+=dBodyGetPositionAll(object[lowerArm].body,3);
//     dsSetColor(1,0,0);    
//     dsDrawSphere (dBodyGetPosition(object[hand]), dBodyGetRotation(object[lowerArm].body) ,SIDE*0.2);
//   }

  // draw (nice :-) muscles
  if (conf.includeMuscles && conf.drawMuscles) {
    for(int j=mainMuscle11; j<mainMuscle22; j+=2){
      const dReal* pos1 = dBodyGetPosition (object[j] .body);
      const dReal* pos2 = dBodyGetPosition (object[j+1] .body);
      dReal pos[3];
      double len=0;
      for(int i=0; i<3; i++){
	len+= (pos1[i] - pos2[i])*(pos1[i] - pos2[i]);
	pos[i] = (pos1[i] + pos2[i])/2;
      }    
      if(j==mainMuscle11){
	if (sqrt(len)<min_l) min_l=sqrt(len);
	if (sqrt(len)>max_l) max_l=sqrt(len);
      }
      if(j==mainMuscle21){
	if (sqrt(len)<min_r) min_r=sqrt(len);
	if (sqrt(len)>max_r) max_r=sqrt(len);
      }
      
      dsSetColor (0.8,0,0);
      dsDrawCappedCylinder ( pos , dBodyGetRotation ( object[j].body ) , 
			     sqrt(len) ,0.032*( -2.0*sqrt(len)+1) +0.02);    
      
      dsDrawCappedCylinder ( pos , dBodyGetRotation ( object[j].body ) , 
			     sqrt(len) +SIDE*1.8 ,0.02 );    
    }
    for(int j=smallMuscle11; j<smallMuscle42; j+=2){
      const dReal* pos1 = dBodyGetPosition (object[j] .body);
      const dReal* pos2 = dBodyGetPosition (object[j+1] .body);
      dReal pos[3];
      double len=0;
      double sign=0;
      for(int i=0; i<3; i++){
	sign+= (pos1[i] - pos2[i]);
	len+= (pos1[i] - pos2[i])*(pos1[i] - pos2[i]);
	pos[i] = (pos1[i] + pos2[i])/2;
      }    
      dsSetColor (0.8,0,0);
      dsDrawCappedCylinder ( pos , dBodyGetRotation ( object[j].body ) , 
			     0.5*sqrt(len) ,0.05*( -sqrt(len)+0.3 ));    
      if (sign>0){
	dsDrawCappedCylinder ( pos , dBodyGetRotation ( object[j].body ) , 
			       0.8*sqrt(len) +SIDE*0.45 ,0.005 );    
      }
      else{
  	dsDrawCappedCylinder ( pos , dBodyGetRotation ( object[j].body ) , 
			       SIDE*0.45 ,0.005 ); 
      }
  
  
    }

  }


   hand_follower.draw();
};

void MuscledArm::setTextures(int texture){
  mainTexture = texture;
}



void MuscledArm::doInternalStuff(const GlobalData& globalData){
  if (conf.includeMuscles){
//     const dReal ksM1 = 0.5;	// spring constant between mainMuscle11 and mainMuscle12
//     const dReal ksM2 = 0.2;	// spring constant between mainMuscle21 and mainMuscle22
//     const dReal ksS1 = 0.5;	// spring constant between smallMuscle11 and smallMuscle12
//     const dReal ksS2 = 0.2;	// spring constant between smallMuscle21 and smallMuscle22
//     const dReal ksS3 = 0.5;	// spring constant between smallMuscle31 and smallMuscle32
//     const dReal ksS4 = 0.2;	// spring constant between smallMuscle41 and smallMuscle42

    double k[6];
    k[0] = 0.5;	// spring constant between mainMuscle11 and mainMuscle12
    k[1] = 0.2;	// spring constant between mainMuscle21 and mainMuscle22
    k[2] = 0.5;	// spring constant between smallMuscle11 and smallMuscle12
    k[3] = 0.2;	// spring constant between smallMuscle21 and smallMuscle22
    k[4] = 0.5;	// spring constant between smallMuscle31 and smallMuscle32
    k[5] = 0.2;	// spring constant between smallMuscle41 and smallMuscle42


    
    // add a spring force to keep the bodies together, otherwise they will
    // fly apart along the slider axis.
    
    //mainMuscle11  =  3
    //smallMuscle42 = 15
    const dReal *p1;
    const dReal *p2;
    for (int i=mainMuscle11; i<smallMuscle42; i+=2){
      p1 = dBodyGetPosition (object[i].body);
      p2 = dBodyGetPosition (object[i+1].body);

      Position dist;
      //distance between slider joint elements
      dist.x=p2[0]-p1[0];
      dist.y=p2[1]-p1[1];
      dist.z=p2[2]-p1[2];
      
      Position force;
      //calculating force
      force=dist*k[(int)(i*0.5)-1];

      //damping
      force=force + (dist - old_dist[i] )*damping;

      dBodyAddForce (object[i].body, force.x, force.y, force.z);
      dBodyAddForce (object[i+1].body, -force.x, -force.y, -force.z);
      old_dist[i]=dist;
    }
  }

  if(print==1){
    dVector3 res;  
     for (int i=hingeJointFUA; i<sliderJointM1; i++){
       dJointGetHingeAnchor (joint[i], res);
       //dJointSetHingeAnchor (joint[hingeJointFM2], 4.7*SIDE, 1.0*SIDE, 1.25*SIDE);
       std::cout<<"dJointSetHingeAnchor(joint["<<getJointName(i)<<"], "<<res[0]<<", "<<res[1]<<", "<<res[2]<<"); \n";

       dJointGetHingeAxis (joint[i], res);
       //dJointSetHingeAxis (joint[hingeJointFM2], 0, 0, 1);
       std::cout<<"dJointSetHingeAxis(joint["<<getJointName(i)<<"], "<<res[0]<<", "<<res[1]<<", "<<res[2]<<"); \n";
       std::cout<<" \n";
     }
     for (int i=sliderJointM1; i<sliderJointS4+1; i++){
       dJointGetSliderAxis (joint[i], res);
       //dJointSetSliderAxis (joint[sliderJointM2],1,0,0);
       std::cout<<"dJointSetSliderAxis(joint["<<getJointName(i)<<"], "<<res[0]<<", "<<res[1]<<", "<<res[2]<<"); \n";
       std::cout<<" \n";
     }

     std::cout<<"dReal q[4]; \n";
     std::cout<<"Position pos; \n";
     for (int i=0; i<NUMParts; i++){
       std::cout<<"// "<<getPartName(i)<<"\n";
       const dReal* R=dBodyGetQuaternion(object[i].body);
       std::cout<<"q[0]="<<R[0]<<";  q[1]="<<R[1]<<";  q[2]="<<R[2]<<";  q[3]="<<R[3]<<";\n";
       const dReal* P=dBodyGetPosition(object[i].body);
       std::cout<<"pos.x="<<P[0]<<";  pos.y="<<P[1]<<";  pos.z="<<P[2]<<"; \n";
     }
     print=0;
   }
}

void MuscledArm::mycallback(void *data, dGeomID o1, dGeomID o2){
  MuscledArm* me = (MuscledArm*)data;  
 
  if (// collision between fixed body and upper arm
      ((o1 == me->object[fixedBody].geom) && (o2 == me->object[upperArm].geom)) 
      || ((o2 == me->object[fixedBody].geom) && (o1 == me->object[upperArm].geom))
      // collision between upper arm and lower arm
      || ((o1 == me->object[upperArm].geom) && (o2 == me->object[lowerArm].geom)) 
      || ((o2 == me->object[upperArm].geom) && (o1 == me->object[lowerArm].geom))
      // collision between fixed body and lower arm
      || ((o1 == me->object[fixedBody].geom) && (o2 == me->object[lowerArm].geom)) 
      || ((o2 == me->object[fixedBody].geom) && (o1 == me->object[lowerArm].geom))
      ){
   
    int i,n;  
    const int N = 10;
    dContact contact[N];
    
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = 0.01;
      contact[i].surface.soft_erp = 1;
      contact[i].surface.soft_cfm = 0.00001;
      dJointID c = dJointCreateContact( me->world, me->contactgroup, &contact[i]);
      dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
    }
  }

}


bool MuscledArm::collisionCallback(void *data, dGeomID o1, dGeomID o2){

  //checks if both of the collision objects are part of the robot
  if( o1 == (dGeomID)arm_space || o2 == (dGeomID)arm_space) {
    
    // treat inner collisions in mycallback  => now down with joint stops
    //dSpaceCollide(arm_space, this, mycallback);

    int i,n;  
    const int N = 10;
    dContact contact[N];
    
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    for (i=0; i<n; i++) {
      if( contact[i].geom.g1 == object[fixedBody].geom || contact[i].geom.g2 == object[fixedBody].geom 
 	  || contact[i].geom.g1 == object[upperArm].geom || contact[i].geom.g2 == object[upperArm].geom 
 	  || contact[i].geom.g1 == object[lowerArm].geom || contact[i].geom.g2 == object[lowerArm].geom){ 
 	// only treat collisions with fixed body, upper arm or lower arm
 	contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
 	contact[i].surface.mu = 0.01;
 	contact[i].surface.soft_erp = 1;
 	contact[i].surface.soft_cfm = 0.00001;

	dJointID c = dJointCreateContact( world, contactgroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
      } 
    }
    return true;
  } else {
    return false;
  }
 
}


/** creates vehicle at desired position 
    @param pos struct Position with desired position
*/
void MuscledArm::create(Position pos){
  if (created) {
    destroy();
  }

  //todo: add dependence from pos
  
  // create arm space and add it to the top level space
  arm_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (arm_space,0);

  dMass f_m;
  dMassSetBox (&f_m,1,SIDE,SIDE,SIDE);
  dMassAdjust (&f_m,MASS);
  object[fixedBody].body = dBodyCreate (world);
  dBodySetMass (object[fixedBody].body,&f_m);
  

  if(conf.strained){
    dQuaternion q1;
    dQFromAxisAndAngle (q1,1,0,0,0);
    dBodySetPosition (object[fixedBody].body,5.2*SIDE,0.25*SIDE,1.25*SIDE);
    dBodySetQuaternion (object[fixedBody].body,q1);
    object[fixedBody].geom = dCreateBox (arm_space,SIDE,SIDE*1.7f,SIDE);
    dGeomSetBody (object[fixedBody].geom, object[fixedBody].body);

    BodyCreate(upperArm, f_m, 3.2*SIDE-0.01,0.25*SIDE,1.25*SIDE, 0,1,0,M_PI*0.5);
    //here
    BodyCreate(lowerArm, f_m, 1.5*SIDE,1.25*SIDE,1.25*SIDE, 1,0,0,M_PI*0.5);
    //BodyCreate(lowerArm, f_m, 1.5*SIDE,1.25*SIDE,1.25*SIDE, 0,0,0,M_PI*0.5);
    if (conf.includeMuscles) {
      BodyCreate(mainMuscle22, f_m, 2.7*SIDE,1.0*SIDE,1.25*SIDE, 0,1,0,M_PI*0.5);
      BodyCreate(mainMuscle21, f_m, 3.7*SIDE,1.0*SIDE,1.25*SIDE, 0,1,0,M_PI*0.5);
      BodyCreate(mainMuscle12, f_m, 2.7*SIDE,-0.5*SIDE,1.25*SIDE, 0,1,0,M_PI*0.5);
      BodyCreate(mainMuscle11, f_m, 3.7*SIDE,-0.5*SIDE,1.25*SIDE, 0,1,0,M_PI*0.5);

      BodyCreate(smallMuscle11, f_m, 4.37*SIDE,-0.25*SIDE,1.25*SIDE, -1,-1,0,M_PI/2);
      BodyCreate(smallMuscle12, f_m, 4.1*SIDE,0.0*SIDE,1.25*SIDE, -1,-1,0,M_PI/2);
      BodyCreate(smallMuscle21, f_m, 4.37*SIDE,0.75*SIDE,1.25*SIDE, -1,1,0,M_PI/2);
      BodyCreate(smallMuscle22, f_m, 4.1*SIDE,0.5*SIDE,1.25*SIDE, -1,1,0,M_PI/2);
      BodyCreate(smallMuscle32, f_m, 2.00*SIDE,-0.25*SIDE,1.25*SIDE, -1,1,0,M_PI/2);
      BodyCreate(smallMuscle31, f_m, 2.3*SIDE,0.0*SIDE,1.25*SIDE, -1,1,0,M_PI/2);
      BodyCreate(smallMuscle42, f_m, 2.00*SIDE,0.75*SIDE,1.25*SIDE, -1,-1,0,M_PI/2);
      BodyCreate(smallMuscle41, f_m, 2.3*SIDE,0.5*SIDE,1.25*SIDE, -1,-1,0,M_PI/2);
    } 


    /* hinge joint between 2 main arms */
    joint[hingeJointUALA] = dJointCreateHinge (world,0);
    dJointAttach (joint[hingeJointUALA],object[upperArm].body,object[lowerArm].body);
    dJointSetHingeAnchor (joint[hingeJointUALA], 1.7*SIDE, 0.25*SIDE, 1.25*SIDE);
    dJointSetHingeAxis (joint[hingeJointUALA], 0, 0, 1);
    dJointSetHingeParam(joint[hingeJointUALA], dParamLoStop,-M_PI/2);
    dJointSetHingeParam(joint[hingeJointUALA], dParamHiStop, M_PI/2); 

    /* hinge joint between upper arm and fixed body */
    joint[hingeJointFUA] = dJointCreateHinge (world,0);
    dJointAttach (joint[hingeJointFUA],object[fixedBody].body, object[upperArm].body);
    dJointSetHingeAnchor (joint[hingeJointFUA], 4.7*SIDE, 0.25*SIDE, 1.25*SIDE);
    dJointSetHingeAxis (joint[hingeJointFUA], 0, 0, 1);
    dJointSetHingeParam(joint[hingeJointUALA], dParamLoStop,-M_PI/2);
    dJointSetHingeParam(joint[hingeJointUALA], dParamHiStop, M_PI/2); 

    if (conf.includeMuscles) {
      /* hinge joint between muscle 3 and fixed body */
      joint[hingeJointFM2] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointFM2],object[fixedBody].body,object[mainMuscle21].body);
      dJointSetHingeAnchor (joint[hingeJointFM2], 4.7*SIDE, 1.0*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointFM2], 0, 0, 1);
	
      /* hinge joint between muscle 5 and fixed body */
      joint[hingeJointFM1] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointFM1],object[fixedBody].body,object[mainMuscle11].body);
      dJointSetHingeAnchor (joint[hingeJointFM1], 4.7*SIDE, -0.5*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointFM1], 0, 0, 1);
	
      /* hinge joint between muscle 2 and arm 1 */
      joint[hingeJointLAM2] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointLAM2],object[lowerArm].body, object[mainMuscle22].body);
      dJointSetHingeAnchor (joint[hingeJointLAM2], 1.7*SIDE, 1.0*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointLAM2], 0, 0, 1);
	
      /* hinge joint between muscle 4 and arm 1 */
      joint[hingeJointLAM1] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointLAM1],object[lowerArm].body, object[mainMuscle12].body);
      dJointSetHingeAnchor (joint[hingeJointLAM1], 1.7*SIDE, -0.5*SIDE,1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointLAM1], 0, 0, 1);
	
      /* hinge joint between muscle 6 and fixed body */
      joint[hingeJointFS1] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointFS1],object[fixedBody].body, object[smallMuscle11].body);
      dJointSetHingeAnchor (joint[hingeJointFS1], 4.7*SIDE, -0.5*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointFS1], 0, 0, 1);
	
	
      /* hinge joint between muscle 7 and arm 0 */
      joint[hingeJointUAS1] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointUAS1],object[upperArm].body, object[smallMuscle12].body);
      dJointSetHingeAnchor (joint[hingeJointUAS1], 3.7*SIDE, 0.15*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointUAS1], 0, 0, 1);
	
      /* hinge joint between muscle 8 and fixed */
      joint[hingeJointFS2] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointFS2],object[fixedBody].body, object[smallMuscle21].body);
      dJointSetHingeAnchor (joint[hingeJointFS2], 4.7*SIDE, 1.0*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointFS2], 0, 0, 1);
	
      /* hinge joint between muscle 9 and arm 0 */
      joint[hingeJointUAS2] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointUAS2],object[upperArm].body, object[smallMuscle22].body);
      dJointSetHingeAnchor (joint[hingeJointUAS2], 3.7*SIDE, 0.35*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointUAS2], 0, 0, 1);
	
      /* hinge joint between muscle 10 and arm 1 */
      joint[hingeJointLAS3] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointLAS3],object[lowerArm].body, object[smallMuscle32].body);
      dJointSetHingeAnchor (joint[hingeJointLAS3], 1.7*SIDE, -0.5*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointLAS3], 0, 0, 1);
	
      /* hinge joint between muscle 11 and arm 0 */
      joint[hingeJointUAS3] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointUAS3],object[upperArm].body, object[smallMuscle31].body);
      dJointSetHingeAnchor (joint[hingeJointUAS3], 2.7*SIDE, 0.15*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointUAS3], 0, 0, 1);
	
      /* hinge joint between muscle 12 and arm 1 */
      joint[hingeJointLAS4] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointLAS4], object[lowerArm].body, object[smallMuscle42].body);
      dJointSetHingeAnchor (joint[hingeJointLAS4], 1.7*SIDE, 1.0*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointLAS4], 0, 0, 1);
	
      /* hinge joint between muscle 13 and arm 0 */
      joint[hingeJointUAS4] = dJointCreateHinge (world,0);
      dJointAttach (joint[hingeJointUAS4],object[upperArm].body, object[smallMuscle41].body);
      dJointSetHingeAnchor (joint[hingeJointUAS4], 2.7*SIDE, 0.35*SIDE, 1.25*SIDE);
      dJointSetHingeAxis (joint[hingeJointUAS4], 0, 0, 1);
    }
  
    /* fixed body is fixed to the world */
    joint[fixedJoint] = dJointCreateFixed (world, 0);
    dJointAttach (joint[fixedJoint],object[fixedBody].body,0);
    dJointSetFixed (joint[fixedJoint]);
  
    if (conf.includeMuscles) {
      /* slider joint between mainMuscle21 and mainMuscle22 */
      joint[sliderJointM2] = dJointCreateSlider (world,0);
      dJointAttach (joint[sliderJointM2], object[mainMuscle22].body, object[mainMuscle21].body);
      dJointSetSliderAxis (joint[sliderJointM2],1,0,0);
	
      /* slider joint between mainMuscle11 and mainMuscle12 */
      joint[sliderJointM1] = dJointCreateSlider (world,0);
      dJointAttach (joint[sliderJointM1], object[mainMuscle12].body, object[mainMuscle11].body);
      dJointSetSliderAxis (joint[sliderJointM1],1,0,0);
	
      /* slider joint between smallMuscle11 and smallMuscle12 */
      joint[sliderJointS1] = dJointCreateSlider (world,0);
      dJointAttach (joint[sliderJointS1], object[smallMuscle11].body, 
		    object[smallMuscle12].body);
      dJointSetSliderAxis (joint[sliderJointS1],1,-1,0);
	
      /* slider joint between smallMuscle21 and smallMuscle22 */
      joint[sliderJointS2] = dJointCreateSlider (world,0);
      dJointAttach (joint[sliderJointS2], object[smallMuscle21].body, 
		    object[smallMuscle22].body);
      dJointSetSliderAxis (joint[sliderJointS2],1,1,0);
	
      /* slider joint between smallMuscle31 and smallMuscle32 */
      joint[sliderJointS3] = dJointCreateSlider (world,0);
      dJointAttach (joint[sliderJointS3], object[smallMuscle32].body, 
		    object[smallMuscle31].body);
      dJointSetSliderAxis (joint[sliderJointS3],1,1,0); //or (-1, -1 ,0)
	
      /* slider joint between smallMuscle41 and smallMuscle42 */
      joint[sliderJointS4] = dJointCreateSlider (world,0);
      dJointAttach (joint[sliderJointS4], object[smallMuscle42].body, 
		    object[smallMuscle41].body);
      dJointSetSliderAxis (joint[sliderJointS4],1,-1,0);
    }

 }else{
    dReal q[4];
    q[0]=1;  q[1]=-8.38228e-24;  q[2]=-5.73441e-24;  q[3]=7.56852e-21;
    dBodySetQuaternion (object[fixedBody].body,q);

    Position pos;
    pos.x=1.04;  pos.y=0.05;  pos.z=0.25;
    dBodySetPosition (object[fixedBody].body,pos.x,pos.y,pos.z);

    object[fixedBody].geom = dCreateBox (arm_space,SIDE,SIDE*1.7f,SIDE);
    dGeomSetBody (object[fixedBody].geom, object[fixedBody].body);

    for(int n=upperArm; n<=smallMuscle42; n++) {

      if (n==upperArm){
	pos.x=0.64245;  pos.y=-0.0369706;  pos.z=0.25;
	q[0]=0.699971;  q[1]=-0.100201;  q[2]=0.699971;  q[3]=0.100201;
	object[n].geom = dCreateBox (arm_space,SIDE*0.2f,SIDE*0.2f,SIDE*3.0f);
      }
      if (n==lowerArm){
	pos.x=0.219915;  pos.y=0.025933;  pos.z=0.25;
	q[0]=0.676795;  q[1]=0.676795;  q[2]=0.204812;  q[3]=0.204812;
	object[n].geom = dCreateBox (arm_space,SIDE*0.2f,SIDE*0.2f,SIDE*4.0f);
      }
      if (n== mainMuscle11){
	pos.x=0.747942;  pos.y=-0.155801;  pos.z=0.25;
	q[0]=0.700052;  q[1]=-0.0996381;  q[2]=0.700052;  q[3]=0.0996381;
	object[n].geom = dCreateBox (arm_space,SIDE*0.2f,SIDE*0.2f,SIDE*2.0f);
      }
      if (n==mainMuscle12){
	pos.x=0.639324;  pos.y=-0.18736;  pos.z=0.25;
	q[0]=0.700052;  q[1]=-0.0996381;  q[2]=0.700052;  q[3]=0.0996381;
	object[n].geom = dCreateBox (arm_space,SIDE*0.2f,SIDE*0.2f,SIDE*2.0f);
      }
      if (n==mainMuscle21){
	pos.x=0.7481;  pos.y=0.143659;  pos.z=0.25;
	q[0]=0.699911;  q[1]=-0.100621;  q[2]=0.699911;  q[3]=0.100621;
	object[n].geom = dCreateBox (arm_space,SIDE*0.2f,SIDE*0.2f,SIDE*2.0f);
      }
      if (n==mainMuscle22){
	pos.x=0.472827;  pos.y=0.0628416;  pos.z=0.25;
	q[0]=0.699911;  q[1]=-0.100621;  q[2]=0.699911;  q[3]=0.100621;
	object[n].geom = dCreateBox (arm_space,SIDE*0.2f,SIDE*0.2f,SIDE*2.0f);
      }
      if (n==smallMuscle11){
	pos.x=0.867069;  pos.y=-0.0607949;  pos.z=0.25;
	q[0]=0.704982;  q[1]=-0.459764;  q[2]=-0.537231;  q[3]=0.0547768;
	object[n].geom = dCreateBox (arm_space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      }
      if (n==smallMuscle12){
	pos.x=0.837317;  pos.y=-0.0425895;  pos.z=0.25;
	q[0]=0.704982;  q[1]=-0.459764;  q[2]=-0.537231;  q[3]=0.0547768;
	object[n].geom = dCreateBox (arm_space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      }
      if (n==smallMuscle21){
	pos.x=0.882527;  pos.y=0.140394;  pos.z=0.25;
	q[0]=0.704977;  q[1]=-0.537274;  q[2]=0.459714;  q[3]=0.0548432;
	object[n].geom = dCreateBox (arm_space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      }
      if (n==smallMuscle22){
	pos.x=0.816819;  pos.y=0.055098;  pos.z=0.25;
	q[0]=0.704977;  q[1]=-0.537274;  q[2]=0.459714;  q[3]=0.0548432;
	object[n].geom = dCreateBox (arm_space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      }
      if (n==smallMuscle31){
	pos.x=0.500997;  pos.y=-0.141568;  pos.z=0.25;
	q[0]=0.691404;  q[1]=-0.593682;  q[2]=0.384111;  q[3]=0.148189;
	object[n].geom = dCreateBox (arm_space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      }
      if (n==smallMuscle32){
	pos.x=0.481504;  pos.y=-0.172963;  pos.z=0.25;
	q[0]=0.691404;  q[1]=-0.593682;  q[2]=0.384111;  q[3]=0.148189;
	object[n].geom = dCreateBox (arm_space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      }
      if (n==smallMuscle41){
	pos.x=0.465184;  pos.y=-0.0484272;  pos.z=0.25;
	q[0]=0.69142;  q[1]=-0.384174;  q[2]=-0.593641;  q[3]=0.148116;
	object[n].geom = dCreateBox (arm_space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      }
      if (n==smallMuscle42){
	pos.x=0.356144;  pos.y=-0.0145328;  pos.z=0.25;
	q[0]=0.69142;  q[1]=-0.384174;  q[2]=-0.593641;  q[3]=0.148116;
	object[n].geom = dCreateBox (arm_space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      }

      object[n].body = dBodyCreate (world);
      dBodySetMass (object[n].body,&f_m);
      dBodySetPosition (object[n].body,pos.x,pos.y,pos.z);
      dBodySetQuaternion (object[n].body,q);
      dGeomSetBody (object[n].geom, object[n].body);
    }
    /*enum joints {
      0 = fixedJoint, 
      1 = hingeJointFUA, 
      2 = hingeJointUALA, 
      3 = hingeJointFM1, 
      4 = hingeJointFM2, 
      5 = hingeJointFS1, 
      6 = hingeJointFS2, 
      7 = hingeJointUAS1, 
      8 =hingeJointUAS2, 
      9 = hingeJointUAS3, 
      10 = hingeJointUAS4, 
      11 = hingeJointLAM1, 
      12 =hingeJointLAM2, 
      13 = hingeJointLAS3, 
      14 = hingeJointLAS4, 
      15 = sliderJointM1, 
      16 = sliderJointM2, 
      17 = sliderJointS1, 
      18 = sliderJointS2, 
      19 = sliderJointS3, 
      20 = sliderJointS4};
    */

    /* fixed body is fixed to the world */
    joint[fixedJoint] = dJointCreateFixed (world, 0);
    dJointAttach (joint[fixedJoint],object[fixedBody].body,0);
    dJointSetFixed (joint[fixedJoint]);

    for (int n=hingeJointFUA; n<sliderJointM1; n++){
      joint[n] = dJointCreateHinge (world,0);
      if (n==1){    /* hinge joint between upper arm and fixed body */
	dJointAttach (joint[hingeJointFUA],object[fixedBody].body, object[upperArm].body);
	dJointSetHingeAnchor(joint[1], 0.94, 0.05, 0.25);
	dJointSetHingeAxis(joint[1], -3.73313e-23, -1.26876e-23, 1);
	// set stops
	dJointSetHingeParam(joint[hingeJointFUA], dParamLoStop,-M_PI/4);
	dJointSetHingeParam(joint[hingeJointFUA], dParamHiStop, M_PI/2); 
      }
      if (n==2){    /* hinge joint between 2 main arms */
	dJointAttach (joint[hingeJointUALA],object[upperArm].body,object[lowerArm].body);
	dJointSetHingeAnchor(joint[2], 0.364096, -0.11833, 0.25);
	dJointSetHingeAxis(joint[2], -1.32215e-16, -3.36141e-17, 1);
	// set stops
	dJointSetHingeParam(joint[hingeJointUALA], dParamLoStop,-M_PI/4);
	dJointSetHingeParam(joint[hingeJointUALA], dParamHiStop, M_PI/2); 
      }

      if (n==3){      /* hinge joint between muscle 5 and fixed body */
	dJointAttach (joint[hingeJointFM1],object[fixedBody].body,object[mainMuscle11].body);
	dJointSetHingeAnchor(joint[3], 0.94, -0.1, 0.25);
	dJointSetHingeAxis(joint[3], -3.73313e-23, -1.26876e-23, 1);
      }
      if (n==4){      /* hinge joint between muscle 3 and fixed body */
	dJointAttach (joint[hingeJointFM2],object[fixedBody].body,object[mainMuscle21].body);
	dJointSetHingeAnchor(joint[4], 0.94, 0.2, 0.25);
	dJointSetHingeAxis(joint[4], -3.73313e-23, -1.26876e-23, 1);
      }
      if (n==5){      /* hinge joint between muscle 6 and fixed body */
	dJointAttach (joint[hingeJointFS1],object[fixedBody].body, object[smallMuscle11].body);
	dJointSetHingeAnchor(joint[5], 0.94, -0.1, 0.25);
	dJointSetHingeAxis(joint[5], -3.73313e-23, -1.26876e-23, 1);
      }
      if (n==6){      /* hinge joint between muscle 8 and fixed */
	dJointAttach (joint[hingeJointFS2],object[fixedBody].body, object[smallMuscle21].body);    
	dJointSetHingeAnchor(joint[6], 0.94, 0.2, 0.25);
	dJointSetHingeAxis(joint[6], -3.73313e-23, -1.26876e-23, 1);
      }
      if (n==7){      /* hinge joint between muscle 7 and arm 0 */
	dJointAttach (joint[hingeJointUAS1],object[upperArm].body, object[smallMuscle12].body);
	dJointSetHingeAnchor(joint[7], 0.753643, -0.0253068, 0.25);
	dJointSetHingeAxis(joint[7], -1.32215e-16, -3.36141e-17, 1);
      }
      if (n==8){      /* hinge joint between muscle 9 and arm 0 */
	dJointAttach (joint[hingeJointUAS2],object[upperArm].body, object[smallMuscle22].body);
	dJointSetHingeAnchor(joint[8], 0.742421, 0.0130867, 0.25);
	dJointSetHingeAxis(joint[8], -1.32215e-16, -3.36141e-17, 1);
      }
      if (n==9){      /* hinge joint between muscle 11 and arm 0 */
	dJointAttach (joint[hingeJointUAS3],object[upperArm].body, object[smallMuscle31].body);
	dJointSetHingeAnchor(joint[9], 0.561675, -0.0814169, 0.25);
	dJointSetHingeAxis(joint[9], -1.32215e-16, -3.36141e-17, 1);
      }
      if (n==10){      /* hinge joint between muscle 13 and arm 0 */
	dJointAttach (joint[hingeJointUAS4],object[upperArm].body, object[smallMuscle41].body);
	dJointSetHingeAnchor(joint[10], 0.550453, -0.0430233, 0.25);
	dJointSetHingeAxis(joint[10], -1.32215e-16, -3.36141e-17, 1);
      }
      if (n==11){    /* hinge joint between muscle 4 and arm 1 */
	dJointAttach (joint[hingeJointLAM1],object[lowerArm].body, object[mainMuscle12].body);
	dJointSetHingeAnchor(joint[11], 0.447266, -0.243161, 0.25);
	dJointSetHingeAxis(joint[11], 7.38062e-18, 5.28601e-17, 1);
      }
      if (n==12){      /* hinge joint between muscle 2 and arm 1 */
	dJointAttach (joint[hingeJointLAM2],object[lowerArm].body, object[mainMuscle22].body);
	dJointSetHingeAnchor(joint[12], 0.280927, 0.00650108, 0.25);
	dJointSetHingeAxis(joint[12], 7.38062e-18, 5.28601e-17, 1);
      }
      if (n==13){    /* hinge joint between muscle 10 and arm 1 */
	dJointAttach (joint[hingeJointLAS3],object[lowerArm].body, object[smallMuscle32].body);
	dJointSetHingeAnchor(joint[13], 0.447266, -0.243161, 0.25);
	dJointSetHingeAxis(joint[13], 7.38062e-18, 5.28601e-17, 1);
      }
      if (n==14){      /* hinge joint between muscle 12 and arm 1 */
	dJointAttach (joint[hingeJointLAS4], object[lowerArm].body, object[smallMuscle42].body);
	dJointSetHingeAnchor(joint[14], 0.280927, 0.00650108, 0.25);
	dJointSetHingeAxis(joint[14], 7.38062e-18, 5.28601e-17, 1);
      }
    }

    for (int n=sliderJointM1; n<sliderJointS4+1; n++){
      joint[n] = dJointCreateSlider (world,0);
      if (n==15){      /* slider joint between mainMuscle11 and mainMuscle12 */
	dJointAttach (joint[sliderJointM1], object[mainMuscle12].body, object[mainMuscle11].body);
	dJointSetSliderAxis(joint[15], 0.960289, 0.279007, -6.245e-17);
      }
      if (n==16){      /* slider joint between mainMuscle21 and mainMuscle22 */
	dJointAttach (joint[sliderJointM2], object[mainMuscle22].body, object[mainMuscle21].body);
	dJointSetSliderAxis(joint[16], 0.959502, 0.281703, -1.30267e-16);
      }
      if (n==17){      /* slider joint between smallMuscle11 and smallMuscle12 */
	dJointAttach (joint[sliderJointS1], object[smallMuscle11].body, object[smallMuscle12].body);
	dJointSetSliderAxis(joint[17], 0.807844, -0.589396, 1.22027e-16);
      }
      if (n==18){      /* slider joint between smallMuscle21 and smallMuscle22 */
	dJointAttach (joint[sliderJointS2], object[smallMuscle21].body, object[smallMuscle22].body);
	dJointSetSliderAxis(joint[18], 0.589244, 0.807955, -1.2265e-16);
      }
      if (n==19){      /* slider joint between smallMuscle31 and smallMuscle32 */
	dJointAttach (joint[sliderJointS3], object[smallMuscle32].body, object[smallMuscle31].body);
	dJointSetSliderAxis(joint[19], 0.355199, 0.934791, 1.86401e-16); // or -0.355199, -0.934791, 0
      }
      if (n==20){      /* slider joint between smallMuscle41 and smallMuscle42 */
	dJointAttach (joint[sliderJointS4], object[smallMuscle42].body, object[smallMuscle41].body);
	dJointSetSliderAxis(joint[20], 0.934716, -0.355397, -5.64869e-17);
      }
    }
    // append the hand :-) 
    dMass hand_m;
    dMassSetSphere (&hand_m,1,SIDE/5);
    dMassAdjust (&hand_m,MASS/20);

    object[hand].body = dBodyCreate (world);
    dBodySetMass (object[hand].body,&hand_m);
    //difference between center of arm and center of sphere=-halbe hoehe der arm-box 
    Position s(0,0, -SIDE*2);
    Matrix R = odeRto3x3RotationMatrix( dBodyGetRotation(object[lowerArm].body));
    Position newpos = multMatrixPosition(R,s);
    newpos = newpos + Position(dBodyGetPosition(object[lowerArm].body));
    dBodySetPosition (object[hand].body,newpos.x, newpos.y, newpos.z);    
    object[hand].geom = dCreateSphere (arm_space,SIDE/5);
    dGeomSetBody (object[hand].geom, object[hand].body);

    joint[fixedJointHand] = dJointCreateFixed (world, 0);
    dJointAttach (joint[fixedJointHand],object[lowerArm].body, object[hand].body);
    dJointSetFixed (joint[fixedJointHand]);
        
  }
  hand_follower.init(10, object[hand].body);
  created=true;
}; 


void MuscledArm::BodyCreate(int n, dMass m, dReal x, dReal y, dReal z, 
			    dReal qx, dReal qy, dReal qz, dReal qangle)
{
  object[n].body = dBodyCreate (world);
  dBodySetMass (object[n].body,&m);
  dBodySetPosition (object[n].body,x,y,z);
  dQuaternion q[n];
  dQFromAxisAndAngle (q[n],qx,qy,qz,qangle);
  dBodySetQuaternion (object[n].body,q[n]);

  if (qx == 0) 
  {
    if (n == upperArm) {
      object[n].geom = dCreateBox (arm_space,SIDE*0.2f,SIDE*0.2f,SIDE*3.0f);
      dGeomSetBody (object[n].geom, object[n].body);
    } 
    else {
      object[n].geom = dCreateBox (arm_space,SIDE*0.2f,SIDE*0.2f,SIDE*2.0f);
      dGeomSetBody (object[n].geom, object[n].body);
    }
  } 
  else {
    if (n == lowerArm) {
      object[n].geom = dCreateBox (arm_space,SIDE*0.2f,SIDE*0.2f,SIDE*4.0f);
      dGeomSetBody (object[n].geom, object[n].body);  
    } 
    else {
      object[n].geom = dCreateBox (arm_space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      dGeomSetBody (object[n].geom, object[n].body);
    }
  }
}



/** destroys vehicle and space
 */
void MuscledArm::destroy(){
  if (created){
    for (int i=0; i<NUMParts; i++){
      dBodyDestroy(object[i].body);
      dGeomDestroy(object[i].geom);     
    }
    dSpaceDestroy(arm_space);
  }
  created=false;
}


double MuscledArm::dBodyGetPositionAll ( dBodyID basis , int para ){
    dReal Dpos[3];
    const dReal* pos = Dpos;

    pos = dBodyGetPosition ( basis );

    switch (para)
    {
        case 1: return pos[0]; break;
        case 2: return pos[1]; break;
        case 3: return pos[2]; break;	  
    }
    return 0;    
}

double MuscledArm::dGeomGetPositionAll ( dGeomID basis , int para ){
    dReal Dpos[3];
    const dReal* pos = Dpos;

    pos = dGeomGetPosition ( basis );

    switch (para)
    {
        case 1: return pos[0]; break;
        case 2: return pos[1]; break;
        case 3: return pos[2]; break;	  
    }
    return 0;    
}

Object MuscledArm::getMainObject() const {
  return object[hand];  
}

Configurable::paramlist MuscledArm::getParamList() const{
  paramlist list;
  list.push_back(pair<paramkey, paramval> (string("factorMotors"), factorMotors));
  list.push_back(pair<paramkey, paramval> (string("factorSensors"), factorSensors));
  list.push_back(pair<paramkey, paramval> (string("damping"), damping));
  list.push_back(pair<paramkey, paramval> (string("print"), print));
  return list;
}

Configurable::paramval MuscledArm::getParam(const paramkey& key) const{
  if(key == "factorMotors") return factorMotors; 
  else if(key == "factorSensors") return factorSensors; 
  else if(key == "damping") return damping; 
  else if(key == "print") return print; 
  else  return Configurable::getParam(key) ;
}

bool MuscledArm::setParam(const paramkey& key, paramval val){
  if(key == "factorMotors") factorMotors=val;
  else if(key == "factorSensors") factorSensors = val; 
  else if(key == "damping") damping = val; 
  else if(key == "print") print = val; 
  else return Configurable::setParam(key, val);
  return true;
}

std::string MuscledArm::getJointName(int j) const{
  assert( (j>-1) && (j<=NUMJoints) );
  std::string name;
  switch (j)
    {
    case  0: return "fixedJoint"; break;
    case  1: return "hingeJointFUA"; break; 
    case  2: return "hingeJointUALA"; break; 
    case  3: return "hingeJointFM1"; break;
    case  4: return "hingeJointFM2"; break; 
    case  5: return "hingeJointFS1"; break;
    case  6: return "hingeJointFS2"; break;
    case  7: return "hingeJointUAS1"; break;
    case  8: return "hingeJointUAS2"; break;
    case  9: return "hingeJointUAS3"; break;
    case 10: return "hingeJointUAS4"; break;
    case 11: return "hingeJointLAM1"; break;
    case 12: return "hingeJointLAM2"; break;
    case 13: return "hingeJointLAS3"; break;
    case 14: return "hingeJointLAS4"; break;
    case 15: return "sliderJointM1"; break;
    case 16: return "sliderJointM2"; break;
    case 17: return "sliderJointS1"; break; 
    case 18: return "sliderJointS2"; break;
    case 19: return "sliderJointS3"; break;
    case 20: return "sliderJointS4"; break;
    case 21: return "fixedJointHand"; break;
    case 22: return "NUMJoints"; break;
    }  
}

std::string MuscledArm::getPartName(int j) const{
  assert( (j>-1) && (j<=NUMParts) );
  std::string name;
  switch (j)
    {
    case  0: return "fixedBody"; break;
    case  1: return "upperArm"; break; 
    case  2: return "lowerArm"; break; 
    case  3: return "mainMuscle11"; break;
    case  4: return "mainMuscle12"; break; 
    case  5: return "mainMuscle21"; break;
    case  6: return "mainMuscle22"; break;
    case  7: return "smallMuscle11"; break;
    case  8: return "smallMuscle12"; break;
    case  9: return "smallMuscle21"; break;
    case 10: return "smallMuscle22"; break;
    case 11: return "smallMuscle31"; break;
    case 12: return "smallMuscle32"; break;
    case 13: return "smallMuscle41"; break;
    case 14: return "smallMuscle42"; break;
    case 15: return "hand"; break;
    case 16: return "NUMParts"; break;
    }  
}
