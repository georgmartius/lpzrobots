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
 *   Revision 1.2  2005-11-15 12:35:19  fhesse
 *   muscles drawn as muscles, sphere drawn on tip of lower arm
 *
 *   Revision 1.1  2005/11/11 15:37:06  fhesse
 *   preinitial version
 *                                                                 *
 *                                                                         *
 ***************************************************************************/

#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include "drawgeom.h"

#include "muscledarm.h"

#include <assert.h>

MuscledArm::MuscledArm(const OdeHandle& odeHandle, const MuscledArmConf& conf):
  AbstractRobot::AbstractRobot(odeHandle), conf(conf){ 


  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile$",
			      "$Revision$");

  created=false;


  //  factorMotors=1.0;
  //  factorSensors=20.0;
  //  avgMotor=0.3;
  //  maxMotorKraft=1;

  //  gelenkabstand =0.2;
  //  SOCKEL_LAENGE= 0.4;
  //  SOCKEL_BREITE= 0.1;
  //  SOCKEL_HOEHE =0.4;
  //  SOCKEL_MASSE =1;

  //  ARMDICKE=0.2;
  //  ARMLAENGE = 1.2;
  //  ARMABSTAND= 0.03;
  //  ARMMASSE = 0.001;

  /*
  initial_pos.x=0.0;
  initial_pos.y=0.0;
  initial_pos.z=0.0;
  */


  sensorno=2;
  motorno=1;  


 
  color.r=1;
  color.g=1;
  color.b=0;
  mainTexture=DS_WOOD;

  max_l=0;
  max_r=0;
  min_l=10; 
  min_r=10;
};

/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void MuscledArm::setMotors(const motor* motors, int motornumber){
};


/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int MuscledArm::getSensors(sensor* sensors, int sensornumber){
  assert(sensornumber==2);

  /* hinge joint between upper arm and fixed body: hingeJointUALA (=1) */
  /* hinge joint between 2 main arms: hingeJointUALA (=2) */
  for (int i=hingeJointFUA; i<=hingeJointUALA; i++){
    sensors[i]=dJointGetHingeAngleRate (joint[i]);
  }
  return 2;

};

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
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

/** returns position of robot 
    @return position robot position in struct Position  
*/
Position MuscledArm::getPosition(){
  Position pos;
  
  //difference between center of arm and tip of arm 
  dReal s[3];
  s[0]=0;
  s[1]=0;
  s[2]=-SIDE*2;
  const double* R=dBodyGetRotation(object[lowerArm].body);
  dReal new_pos[3];
  //rotation of difference vector
  pos.x=s[0]*R[0]+s[1]*R[1]+s[2]*R[2];
  pos.y=s[0]*R[4]+s[1]*R[5]+s[2]*R[6];
  pos.z=s[0]*R[8]+s[1]*R[9]+s[2]*R[10];
  // adding (rotated) difference vector to actual position of arm center
  // -> leading to actual position of the tip of the arm
  pos.x+=dBodyGetPositionAll(object[lowerArm].body,1);
  pos.y+=dBodyGetPositionAll(object[lowerArm].body,2);
  pos.z+=dBodyGetPositionAll(object[lowerArm].body,3);

  return pos;
};

/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int MuscledArm::getSegmentsPosition(vector<Position> &poslist){
  return segmentsno;
};  


/**
 * draws the vehicle
 */
void MuscledArm::draw(){

  dsSetTexture (mainTexture);
  dsSetColor (color.r,color.g,color.b); // fixedBody
  drawGeom(object[0].geom, 0, 0);
  dsSetColor (1,1,0); // upperArm 
  drawGeom(object[1].geom, 0, 0);
  dsSetColor (0,1,1); // lowerArm
  drawGeom(object[2].geom, 0, 0);

  if (conf.drawSphere){  
    //difference between center of arm and center of sphere=-halbe hoehe der arm-box 
    dReal s[3];
    s[0]=0;
    s[1]=0;
    s[2]=-SIDE*2;
    const double* R=dBodyGetRotation(object[lowerArm].body);
    dReal new_pos[3];
    //rotation of difference vector
    new_pos[0]=s[0]*R[0]+s[1]*R[1]+s[2]*R[2];
    new_pos[1]=s[0]*R[4]+s[1]*R[5]+s[2]*R[6];
    new_pos[2]=s[0]*R[8]+s[1]*R[9]+s[2]*R[10];
    // adding (rotated) difference vector to actual position of arm center
    // -> leading to actual position of sphere
    new_pos[0]+=dBodyGetPositionAll(object[lowerArm].body,1);
    new_pos[1]+=dBodyGetPositionAll(object[lowerArm].body,2);
    new_pos[2]+=dBodyGetPositionAll(object[lowerArm].body,3);
    dReal sides2[3] = {SIDE*0.2f,SIDE*0.2f,SIDE*2.0f};
    dsSetColor(1,0,0);
    dsDrawSphere (new_pos, dBodyGetRotation(object[lowerArm].body) ,SIDE*0.2);
  }

  if (conf.includeMuscles && conf.drawMuscles) {
    // displays mainMuscles
    //dsSetColor (1,0,0);
//     dsSetColor (1,1,0);
//     drawGeom(object[mainMuscle11].geom, 0, 0);
//     //dsSetColor (0,1,0);
//     dsSetColor (0,1,1);
//     drawGeom(object[mainMuscle12].geom, 0, 0);
    //dsSetColor (0,0,1);
//     dsSetColor (1,0,1);
//     drawGeom(object[mainMuscle21].geom, 0, 0);
//     //dsSetColor (0,1,0);
//     dsSetColor (0,1,1);
//     drawGeom(object[mainMuscle22].geom, 0, 0);
//     // displays smallMuscles
//     dsSetColor (0,1,1);
//     drawGeom(object[smallMuscle11].geom, 0, 0);
//     dsSetColor (1,0,1);
//     drawGeom(object[smallMuscle12].geom, 0, 0);
//     dsSetColor (0,1,1);
//     drawGeom(object[smallMuscle21].geom, 0, 0);
//     dsSetColor (1,1,0);
//     drawGeom(object[smallMuscle22].geom, 0, 0);
//     dsSetColor (0,1,1);
//     drawGeom(object[smallMuscle31].geom, 0, 0);
//     dsSetColor (1,1,0);
//     drawGeom(object[smallMuscle32].geom, 0, 0);
//     dsSetColor (0,1,1);
//     drawGeom(object[smallMuscle41].geom, 0, 0);
//     dsSetColor (1,1,0);
//     drawGeom(object[smallMuscle42].geom, 0, 0);

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
      for(int i=0; i<3; i++){
	len+= (pos1[i] - pos2[i])*(pos1[i] - pos2[i]);
	pos[i] = (pos1[i] + pos2[i])/2;
      }    
      dsSetColor (0.8,0,0);
      dsDrawCappedCylinder ( pos , dBodyGetRotation ( object[j].body ) , 
			     0.5*sqrt(len) ,0.05*( -sqrt(len)+0.3 ));    
      dsDrawCappedCylinder ( pos , dBodyGetRotation ( object[j].body ) , 
			     0.8*sqrt(len) +SIDE*0.45 ,0.005 );    
     
    }

  }


};

void MuscledArm::setTextures(int texture){
  mainTexture = texture;
}


void MuscledArm::mycallback(void *data, dGeomID o1, dGeomID o2){
  MuscledArm* me = (MuscledArm*)data;  
  if(isGeomInObjectList(me->object, me->segmentsno, o1) && isGeomInObjectList(me->object, me->segmentsno, o2)){
    return;
  }
}

void MuscledArm::doInternalStuff(const GlobalData& globalData){
  if (conf.includeMuscles){
    const dReal ksM1 = 0.5;	// spring constant between mainMuscle11 and mainMuscle12
    const dReal ksM2 = 0.2;	// spring constant between mainMuscle21 and mainMuscle22
    const dReal ksS1 = 0.5;	// spring constant between smallMuscle11 and smallMuscle12
    const dReal ksS2 = 0.2;	// spring constant between smallMuscle21 and smallMuscle22
    const dReal ksS3 = 0.5;	// spring constant between smallMuscle31 and smallMuscle32
    const dReal ksS4 = 0.2;	// spring constant between smallMuscle41 and smallMuscle42


    
    // add a spring force to keep the bodies together, otherwise they will
    // fly apart along the slider axis.
    
    // spring force to mainMuscle1
    const dReal *pM12 = dBodyGetPosition (object[mainMuscle12].body);
    const dReal *pM11 = dBodyGetPosition (object[mainMuscle11].body);
    dBodyAddForce (object[mainMuscle12].body,
		   ksM1*(pM11[0]-pM12[0]),ksM1*(pM11[1]-pM12[1]),ksM1*(pM11[2]-pM12[2]));
    dBodyAddForce (object[mainMuscle11].body,
		   ksM1*(pM12[0]-pM11[0]),ksM1*(pM12[1]-pM11[1]),ksM1*(pM12[2]-pM11[2]));

    // spring force to mainMuscle2
    const dReal *pM22 = dBodyGetPosition (object[mainMuscle22].body);
    const dReal *pM21 = dBodyGetPosition (object[mainMuscle21].body);
    dBodyAddForce (object[mainMuscle22].body,
		   ksM2*(pM21[0]-pM22[0]),ksM2*(pM21[1]-pM22[1]), ksM2*(pM21[2]-pM22[2]));
    dBodyAddForce (object[mainMuscle21].body,
		   ksM2*(pM22[0]-pM21[0]),ksM2*(pM22[1]-pM21[1]), ksM2*(pM22[2]-pM21[2]));


    // spring force to smallMuscle1 		   
    const dReal *pS12 = dBodyGetPosition (object[smallMuscle12].body);
    const dReal *pS11 = dBodyGetPosition (object[smallMuscle11].body);
    dBodyAddForce (object[smallMuscle12].body,
		   ksS1*(pS11[0]-pS12[0]),ksS1*(pS11[1]-pS12[1]),ksS1*(pS11[2]-pS12[2]));
    dBodyAddForce (object[smallMuscle11].body,
		   ksS1*(pS12[0]-pS11[0]),ksS1*(pS12[1]-pS11[1]),ksS1*(pS12[2]-pS11[2]));
    
    // spring force to smallMuscle2 
    const dReal *pS22 = dBodyGetPosition (object[smallMuscle22].body);
    const dReal *pS21 = dBodyGetPosition (object[smallMuscle21].body);
    dBodyAddForce (object[smallMuscle22].body,
		   ksS2*(pS21[0]-pS22[0]),ksS2*(pS21[1]-pS22[1]), ksS2*(pS21[2]-pS22[2]));
    dBodyAddForce (object[smallMuscle21].body,
		   ksS2*(pS22[0]-pS21[0]),ksS2*(pS22[1]-pS21[1]), ksS2*(pS22[2]-pS21[2]));
 		   
    // spring force to smallMuscle3 		   
    const dReal *pS32 = dBodyGetPosition (object[smallMuscle32].body);
    const dReal *pS31 = dBodyGetPosition (object[smallMuscle31].body);
    dBodyAddForce (object[smallMuscle32].body,
		   ksS3*(pS31[0]-pS32[0]),ksS3*(pS31[1]-pS32[1]),ksS3*(pS31[2]-pS32[2]));
    dBodyAddForce (object[smallMuscle31].body,
		   ksS3*(pS32[0]-pS31[0]),ksS3*(pS32[1]-pS31[1]),ksS3*(pS32[2]-pS31[2]));
    
    // sprinf force to smallMuscle4
    const dReal *pS42 = dBodyGetPosition (object[smallMuscle42].body);
    const dReal *pS41 = dBodyGetPosition (object[smallMuscle41].body);
    dBodyAddForce (object[smallMuscle42].body,
		   ksS4*(pS41[0]-pS42[0]),ksS4*(pS41[1]-pS42[1]), ksS4*(pS41[2]-pS42[2]));
    dBodyAddForce (object[smallMuscle41].body,
		   ksS4*(pS42[0]-pS41[0]),ksS4*(pS42[1]-pS41[1]), ksS4*(pS42[2]-pS41[2])); 		    		   

  }
}

bool MuscledArm::collisionCallback(void *data, dGeomID o1, dGeomID o2){

  int i, numc;
  int contPts = 1;
  dContact contact[contPts];			// up to 4 contacts per box-box

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);

  return true; // todo: remove this and make collision things work

  if( o1 == (dGeomID)arm_space || o2 == (dGeomID)arm_space){
    if (b1 && b2 && dAreConnected(b1,b2)) 
      return true;
    if (b1 == object[mainMuscle11].body && b2 == object[mainMuscle12].body)
      return true;
    if (b1 == object[mainMuscle12].body && b2 == object[mainMuscle11].body)
      return true;
    if (b1 == object[mainMuscle21].body && b2 == object[mainMuscle22].body)
      return true;
    if (b1 == object[mainMuscle22].body && b2 == object[mainMuscle21].body)
      return true;
    /*  */
    if (b1 == object[smallMuscle11].body && b2 == object[smallMuscle12].body)
      return true;
    if (b1 == object[smallMuscle12].body && b2 == object[smallMuscle11].body)
      return true;
    if (b1 == object[smallMuscle21].body && b2 == object[smallMuscle22].body)
      return true;
    if (b1 == object[smallMuscle22].body && b2 == object[smallMuscle21].body)
      return true;
    if (b1 == object[smallMuscle31].body && b2 == object[smallMuscle32].body)
      return true;
    if (b1 == object[smallMuscle32].body && b2 == object[smallMuscle31].body)
      return true;
    if (b1 == object[smallMuscle41].body && b2 == object[smallMuscle42].body)
      return true;
    if (b1 == object[smallMuscle42].body && b2 == object[smallMuscle41].body)
      return true;
    /*  */
    
    int g1 = (b1 == object[lowerArm].body || b1 == object[upperArm].body);
    int g2 = (b2 == object[lowerArm].body || b2 == object[upperArm].body);
    if (!(g1 ^ g2)) return true;
    
    
    for (i=0; i<contPts; i++) {
      /* dContactBounce to define surface elasticity
       * dContactSoftCFM to get the surface's contact normal "softness" 
       *  constraint force mixing parameter.
       * dContactSoftERP to get the surface's contact normal "softness" 
       *  error reduction parameter. 
       * dContactApprox1 to 
       * dContactSlip1 to get the surface's first-order slip in friction 
       * direction 1
       * dContactSlip2 toGet the surface's first-order slip in friction 
       * direction 2
       */
      //      contact[i].surface.mode = 0;
   
      // surface.mu = 0.0 (frictionless) to dInfinity (no slipping)
      //      contact[i].surface.mu = 1;
      // surface.mu2 = 0.0 to dInfinity - friction in direction 2
      //      contact[i].surface.mu2 = 0;
      // surface.bounce = 0 (inelastic) to 1 (elastic)
      //      contact[i].surface.bounce = 0.1;
      // surface.bounce_vel - incoming velocity for bounce
      //      contact[i].surface.bounce_vel = 0.1;
      // surface.soft_cfm = 1E-9 to 1
      //      contact[i].surface.soft_cfm = 0.0001;
      // surface.soft_erp = 0.1 to 0.8
      //      contact[i].surface.soft_erp = 0.3;
    }
    if ((numc = dCollide (o1,o2,contPts,&contact[0].geom,sizeof(dContact)))) {
      for (i=0; i<numc; i++) {
	dJointID c = dJointCreateContact (world,contactgroup,contact+i);
	dJointAttach (c,b1,b2);
      }
    }
    return true;
  }
  return false;
 
}


/** creates vehicle at desired position 
    @param pos struct Position with desired position
*/
void MuscledArm::create(Position pos){
  if (created) {
    destroy();
  }
  // create arm space and add it to the top level space
  arm_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (arm_space,0);

  dMass f_m;
  dMassSetBox (&f_m,1,SIDE,SIDE,SIDE);
  dMassAdjust (&f_m,MASS);

  object[fixedBody].body = dBodyCreate (world);
  dBodySetMass (object[fixedBody].body,&f_m);
  dQuaternion q1;
  dQFromAxisAndAngle (q1,1,0,0,0);
  dBodySetPosition (object[fixedBody].body,5.2*SIDE,0.25*SIDE,1.25*SIDE);
  dBodySetQuaternion (object[fixedBody].body,q1);
  object[fixedBody].geom = dCreateBox (arm_space,SIDE,SIDE*1.7f,SIDE);
  dGeomSetBody (object[fixedBody].geom, object[fixedBody].body);


  BodyCreate(upperArm, f_m, 3.2*SIDE,0.25*SIDE,1.25*SIDE, 0,1,0,M_PI*0.5);
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

  /* hinge joint between upper arm and fixed body */
  joint[hingeJointFUA] = dJointCreateHinge (world,0);
  dJointAttach (joint[hingeJointFUA],object[fixedBody].body, object[upperArm].body);
  dJointSetHingeAnchor (joint[hingeJointFUA], 4.7*SIDE, 0.25*SIDE, 1.25*SIDE);
  dJointSetHingeAxis (joint[hingeJointFUA], 0, 0, 1);

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
    dJointSetSliderAxis (joint[sliderJointS3],-1,-1,0);
	
    /* slider joint between smallMuscle41 and smallMuscle42 */
    joint[sliderJointS4] = dJointCreateSlider (world,0);
    dJointAttach (joint[sliderJointS4], object[smallMuscle42].body, 
		  object[smallMuscle41].body);
    dJointSetSliderAxis (joint[sliderJointS4],1,-1,0);
  }
  
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
      object[n].geom = dCreateBox (space,SIDE*0.2f,SIDE*0.2f,SIDE*3.0f);
      dGeomSetBody (object[n].geom, object[n].body);
    } 
    else {
      object[n].geom = dCreateBox (space,SIDE*0.2f,SIDE*0.2f,SIDE*2.0f);
      dGeomSetBody (object[n].geom, object[n].body);
    }
  } 
  else {
    if (n == lowerArm) {
      object[n].geom = dCreateBox (space,SIDE*0.2f,SIDE*0.2f,SIDE*4.0f);
      dGeomSetBody (object[n].geom, object[n].body);  
    } 
    else {
      object[n].geom = dCreateBox (space,SIDE*0.1f,SIDE*0.1f,SIDE*0.5f);
      dGeomSetBody (object[n].geom, object[n].body);
    }
  }
}


/** destroys vehicle and space
 */
void MuscledArm::destroy(){
  if (created){
    for (int i=0; i<NUM; i++){
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

Configurable::paramlist MuscledArm::getParamList() const{
  paramlist list;
  return list;
}

Configurable::paramval MuscledArm::getParam(const paramkey& key) const{
  return Configurable::getParam(key) ;
}

bool MuscledArm::setParam(const paramkey& key, paramval val){
  return true;
}




