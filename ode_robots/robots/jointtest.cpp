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
 *   Revision 1.1  2005-06-22 15:38:05  fhesse
 *   sensors and motor values are wheel velocities
 *
 *                                                                 *
 ***************************************************************************/


#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"

#include "jointtest.h"
#include <iostream>

JointTest::JointTest(dWorldID *w, dSpaceID *s, dJointGroupID *c):
  AbstractRobot::AbstractRobot(w, s, c){

  created=false;

  initial_pos.x=0.0;
  initial_pos.y=0.0;
  initial_pos.z=0.0;
  
  color.r=1;
  color.g=0;
  color.b=1;
    
  max_force=0.5;

  glieder_masse=0.4;
  glieder_durchmesser=0.2;
  glieder_laenge=1.0;

  sensorno=2; 
  motorno=2;  
  segmentsno=2;

  t=0;
  positiv=false;
};

/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void JointTest::setMotors(const motor* motors, int motornumber){

  double tmp;
  t++;

  /*    tmp=dJointGetHinge2Param(joints[1],dParamVel);
    dJointSetHinge2Param(joints[1],dParamVel,tmp + 0.1*(motors[0]*20.0-tmp) );
    dJointSetHinge2Param (joints[1],dParamFMax,1);

    tmp=dJointGetHinge2Param(joints[1],dParamVel2);
    dJointSetHinge2Param(joints[1],dParamVel2,tmp + 0.1*(motors[1]*20.0-tmp) );
    dJointSetHinge2Param (joints[1],dParamFMax2,1);
  */

  
  dJointSetUniversalParam(joints[1], dParamVel,  0.5*motors[0] );
  dJointSetUniversalParam(joints[1], dParamFMax, 1);

  dJointSetUniversalParam(joints[1], dParamVel2,  0.5*motors[1] );
  dJointSetUniversalParam(joints[1], dParamFMax2, 1);
  
  
  /*
  dJointSetUniversalParam(joints[1],dParamVel,0.5*sin((double)t/40.0) );
  dJointSetUniversalParam (joints[1],dParamFMax,10);

  dJointSetUniversalParam(joints[1],dParamVel2,0.5*cos((double)t/40.0) );
  dJointSetUniversalParam (joints[1],dParamFMax2,10);
  */


  //  dJointSetHinge2Param(joints[1],dParamVel,-0.5*sin((double)t/40.0));
  //  dJointSetHinge2Param (joints[1],dParamFMax,1);

  //dJointSetHinge2Param(joints[1],dParamVel2,0.5*sin((double)t/40.0) );
  //dJointSetHinge2Param (joints[1],dParamFMax2,1);


    //dJointAddUniversalTorques(joints[1], sin((double)t/40.0), sin((double)t/40.0));

  //dBodyAddTorque(segments[0].body, 0.05*sin((double)t/40), 0, 0);

    /*
    if (t%10==0){
      if (positiv) {
	dJointAddUniversalTorques(joints[1], 0, 5);
	positiv=false;
      }else{
	dJointAddUniversalTorques(joints[1], 0, -5);      
	positiv=true;
      }
    }
    */

  /*
  double tmp;
  int len = (motornumber < motorno)? motornumber : motorno;
  for (int i=0; i<len; i++){ 
    tmp=dJointGetHinge2Param(joint[i],dParamVel2);
    dJointSetHinge2Param(joint[i],dParamVel2,tmp + 0.5*(motors[i]*20.0-tmp) );
    dJointSetHinge2Param (joint[i],dParamFMax2,max_force);
  }
  */
};

/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int JointTest::getSensors(sensor* sensors, int sensornumber){
  int len = (sensornumber < sensorno)? sensornumber : sensorno;
  sensors[0]=dJointGetUniversalAngle1Rate(joints[1]);
  sensors[1]=dJointGetUniversalAngle2Rate(joints[1]);


  // sensors[0]=dJointGetHinge2Angle1Rate(joints[1]);
  //sensors[1]=dJointGetHinge2Angle2Rate(joints[1]);

  return len;
  /*
  int len = (sensornumber < sensorno)? sensornumber : sensorno;
  for (int i=0; i<len; i++){
    sensors[i]=dJointGetHinge2Angle2Rate(joint[i]);
    sensors[i]*=0.05;  //scaling
  }
  return len;
  */
};

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
void JointTest::place(Position pos, Color *c /*= 0*/){
  if (!c==0) {
    color=*c;
  }
  pos.z+=glieder_durchmesser; // to put wheels on ground, not in ground
  if (!created){ 
    create(pos);
  }
  else{
    /*
    dBodySetPosition (object[1].body,pos.x ,pos.y +width*0.5,pos.z);
    dBodySetPosition (object[2].body,pos.x ,pos.y -width*0.5,pos.z);
    dBodySetPosition (object[0].body,pos.x ,pos.y           ,pos.z);    
    */
  }
};

  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects and the default routine)
   *  else false (collision is passed to other objects and (if not treated) to the default routine).
   */
bool JointTest::collisionCallback(void *data, dGeomID o1, dGeomID o2){

    for ( int n = 0; n < (armanzahl-1); n++ )
      {
	if( ( segments[n].geom == o1 && segments[n + 1].geom == o2 ) 
	    || ( segments[n].geom == o2 && segments[n + 1].geom == o1 ) ){
	  return true;
	}
      }
  return false;
}


/** returns position of robot 
    @return position robot position in struct Position  
*/
Position JointTest::getPosition(){
  Position pos;
  const dReal* act_pos=dBodyGetPosition(segments[0].body);
  pos.x=act_pos[0];
  pos.y=act_pos[1];
  pos.z=act_pos[2]-glieder_durchmesser; // substract wheel radius, because vehicle stands on the ground
  return pos;

  /*
  Position pos;
  const dReal* act_pos=dBodyGetPosition(object[0].body);
  pos.x=act_pos[0];
  pos.y=act_pos[1];
  pos.z=act_pos[2]-radius; // substract wheel radius, because vehicle stands on the ground
  return pos;
  */
};

/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int JointTest::getSegmentsPosition(vector<Position> &poslist){
  return segmentsno;
  /*
  Position pos;
  for (int i=0; i<segmentsno; i++){
    const dReal* act_pos = dBodyGetPosition(object[i].body);
    pos.x=act_pos[0];
    pos.y=act_pos[1];
    pos.z=act_pos[2];
    poslist.push_back(pos);
  }   
  return segmentsno;
  */
};  



/**
 * draws the vehicle
 */
void JointTest::draw(){
  dsSetColor (color.r,color.g,color.b); 
  dsDrawCappedCylinder(  dGeomGetPosition ( segments[0].geom ) , dGeomGetRotation ( segments[0].geom ) , 
			 glieder_laenge , glieder_durchmesser );
  dsDrawCappedCylinder(  dGeomGetPosition ( segments[1].geom ) , dGeomGetRotation ( segments[1].geom ) , 
			 glieder_laenge , glieder_durchmesser );

  /*
  dsSetColor (color.r,color.g,color.b); // set color for cylinder
  dsSetTexture (DS_WOOD);
  dsDrawCappedCylinder(dBodyGetPosition(object[0].body),dBodyGetRotation(object[0].body),length, width/2 );
  dsSetColor (1,1,1); // set color for wheels
  // draw wheels
  for (int i=1; i<3; i++) { 
    dsDrawCylinder (dBodyGetPosition(object[i].body), dBodyGetRotation(object[i].body),0.02f,radius);
  }
  */
};



/** creates vehicle at desired position 
    @param pos struct Position with desired position
*/
void JointTest::create(Position pos){
  if (created) {
    destroy();
  }
  dMass masse;
  dMassSetCappedCylinder ( &masse , glieder_masse , 3, glieder_durchmesser, glieder_laenge  );

  dMatrix3 R;//Matrix fuer Koerper-Rotationen
  dRFromAxisAndAngle ( R , 0 , 1 , 0 , PI/2 );//hier drehung um 90Â° um die y-Achse

  for (int i=0; i<armanzahl; i++){
    segments[i].body=dBodyCreate ( *world);
    dBodySetMass ( segments[i].body , &masse );
    segments[i].geom=dCreateCCylinder ( *space , glieder_durchmesser , glieder_laenge );
    dGeomSetBody ( segments[i].geom , segments[i].body );
    dGeomSetPosition(segments[i].geom,pos.x +i*(glieder_laenge+glieder_laenge/10), pos.y, pos.z+2.0*glieder_laenge);
    dGeomSetRotation ( segments[i].geom, R );
  }

  /*  segments[0].body=dBodyCreate ( welt );
  dBodySetMass ( segments[0].body , &masse );
  segments[0].geom=dCreateCCylinder ( raum , glieder_durchmesser , glieder_laenge );
  dGeomSetBody ( segments[0].geom , segments[0].body );
  dGeomSetPosition(segments[0].geom, 5,0,1+glieder_durchmesser);
  dGeomSetRotation ( segments[0].geom, R );

  segments[1].body=dBodyCreate ( welt );
  dBodySetMass ( segments[1].body , &masse );
  segments[1].geom=dCreateCCylinder ( raum , glieder_durchmesser , glieder_laenge );
  dGeomSetBody ( segments[1].geom , segments[1].body );
  dGeomSetPosition(segments[1].geom, 5+glieder_laenge+glieder_laenge/10,0,1+glieder_durchmesser);
  dGeomSetRotation ( segments[1].geom, R );
  */
  /*
  joints[0]= ( dJointCreateHinge ( *world , 0 ) );
  dJointAttach ( joints[0] , segments[0].body , 0 );
  dJointSetUniversalAnchor ( joints[0] , 
			     dBodyGetPositionAll ( segments[0].body , 1 ) , 
			     dBodyGetPositionAll ( segments[0].body , 2 ) , 
			     dBodyGetPositionAll ( segments[0].body , 3 ) ); 
  dJointSetHingeAxis(joints[0],1,0,0);
  dJointSetFixed(joints[0]);

  joints[2]= ( dJointCreateHinge ( *world , 0 ) );
  dJointAttach ( joints[2] , segments[0].body , 0 );
  dJointSetUniversalAnchor ( joints[2] , 
			     dBodyGetPositionAll ( segments[0].body , 1 ) , 
			     dBodyGetPositionAll ( segments[0].body , 2 ) , 
			     dBodyGetPositionAll ( segments[0].body , 3 ) ); 
  dJointSetHingeAxis(joints[2],0,1,0);
  dJointSetFixed(joints[2]);
  */


  // UniversalJoint
  joints[1]= ( dJointCreateUniversal ( *world , 0 ) );
  dJointAttach ( joints[1] , segments[0].body , segments[1].body );
			
  dJointSetUniversalAnchor ( joints[1] , 
			     dBodyGetPositionAll ( segments[0].body , 1 ) 
			     + ( dBodyGetPositionAll ( segments[1].body , 1 ) 
				 - dBodyGetPositionAll ( segments[0].body , 1 ) )/2 , 
			     dBodyGetPositionAll ( segments[0].body , 2 )  
			     /*+ ( dBodyGetPositionAll ( segments[1].body , 2 ) 
			       - dBodyGetPositionAll ( segments[0].body , 2 )  )/2*/ , 
			     dBodyGetPositionAll ( segments[0].body , 3 ) ); 
  dJointSetUniversalAxis1 ( joints[1] , 0 , 1 , 0 ); 
  dJointSetUniversalAxis2 ( joints[1] , 0 , 0 , 1 ); 

  dJointSetUniversalParam (joints[1], dParamFudgeFactor, 0.1);
  dJointSetUniversalParam (joints[1], dParamBounce, 0.5);

  dJointSetUniversalParam (joints[1], dParamLoStop, -M_PI/4); 
  dJointSetUniversalParam (joints[1], dParamHiStop,  M_PI/4); 

  dJointSetUniversalParam (joints[1], dParamLoStop2, -M_PI/4); 
  dJointSetUniversalParam (joints[1], dParamHiStop2,  M_PI/4); 
  


  /*
  // Hinge2Joint
  joints[1]= ( dJointCreateHinge2 ( *world , 0 ) );
  dJointAttach ( joints[1] , segments[0].body , segments[1].body );
			
  dJointSetHinge2Anchor ( joints[1] , 
			  dBodyGetPositionAll ( segments[0].body , 1 ) 
			  + ( dBodyGetPositionAll ( segments[1].body , 1 ) 
			      - dBodyGetPositionAll ( segments[0].body , 1 ) )/2 , 
			  dBodyGetPositionAll ( segments[0].body , 2 ) 
			  /*+ ( dBodyGetPositionAll ( segments[1].body , 2 ) 
			    - dBodyGetPositionAll ( segments[0].body , 2 )  )/2* / , 
			  dBodyGetPositionAll ( segments[0].body , 3 ) );
  dJointSetHinge2Axis1 ( joints[1] , 0 , 1 , 0 );
  dJointSetHinge2Axis2 ( joints[1] , 0 , 0 , 1 );



  dJointSetHinge2Param (joints[1], dParamLoStop, -M_PI/8);
  dJointSetHinge2Param (joints[1], dParamHiStop,  M_PI/8);

  // Hi and Low stop nur an erster Achse möglich!
  //dJointSetHinge2Param (joints[1], dParamLoStop2, -M_PI/8);
  //dJointSetHinge2Param (joints[1], dParamHiStop2,  M_PI/8);
  */

  created=true;
}; 


/** destroys vehicle and space
 */
void JointTest::destroy(){
  if (created){
    for (int i=0; i<segmentsno; i++){
      dBodyDestroy(segments[i].body);
      dGeomDestroy(segments[i].geom);
    }
  }
  created=false;

  /*
  if (created){
    dSpaceDestroy(car_space);
    for (int i=0; i<segmentsno; i++){
      dBodyDestroy(object[i].body);
      dGeomDestroy(object[i].geom);
    }
  }
  created=false;
  */
}


/**
 *vereinfachte Objektpositions-Einzelermittlungsfunktion
 *@author Marcel Kretschmann
 *@version
 **/
double JointTest::dBodyGetPositionAll ( dBodyID basis , int para )
{
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





