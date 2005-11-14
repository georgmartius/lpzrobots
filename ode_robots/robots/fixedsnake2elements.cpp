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
 *   Revision 1.3.4.1  2005-11-14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.3  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.2  2005/07/18 14:47:32  martius
 *   world, space, contactgroup are not pointers anymore.
 *
 *   Revision 1.1  2005/06/27 13:15:58  fhesse
 *   from JointTest, for usage in simulation running_through_limit_cycles
 *
 *   Revision 1.3  2005/06/27 09:31:26  fhesse
 *   few things tested, velocity as sensor- and motorvalues still works fine
 *
 *   Revision 1.2  2005/06/24 13:33:40  fhesse
 *   a lot tested and added
 *
 *   Revision 1.1  2005/06/22 15:38:05  fhesse
 *   sensors and motor values are wheel velocities
 *
 *                                                                 *
 ***************************************************************************/


#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"

#include "fixedsnake2elements.h"
#include <iostream>

FixedSnake2Elements::FixedSnake2Elements(const OdeHandle& odeHandle)
  : OdeRobot(odeHandle){

  created=false;

  initial_pos.x=0.0;
  initial_pos.y=0.0;
  initial_pos.z=0.0;
  
  color.r=1;
  color.g=0;
  color.b=1;
    
  max_force=0.5;

  glieder_masse=0.1;
  glieder_durchmesser=0.2;
  glieder_laenge=1.0;

  segmentsno=2;
  sensorno=2*(segmentsno-1); 
  motorno=sensorno;  


  t=0;
  positiv=false;

  segments.resize(segmentsno);
  joints.resize( 2*(segmentsno-1)  + 2); // +2 for attaching to sky
  old_sensorvalues.resize( sensorno );
  mean_sensorvalues.resize( sensorno );

  for (int i=0; i<sensorno; i++){
    old_sensorvalues[i]=0.0;
    mean_sensorvalues[i]=0.0;
  }

  gamma=0.35;

};

/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void FixedSnake2Elements::setMotors(const motor* motors, int motornumber){


  t++;
  motor m[sensorno];
  for (int i=0; i<sensorno; i++){
    m[i]=motors[i];
  }

  /*// sin as motorcommand
  m[0]=sin((double)t/40.0);
  m[1]=cos((double)t/40.0);
  m[2]=-sin((double)t/40.0);
  m[3]=-cos((double)t/40.0);
  */

  //setMotorsHinge2Velocity(m); //-> use hinge2 angelrate in getSensors

  //setMotorsUniversalVelocity(m);  // -> use universal joint angle rate in getSensors
  

  // controller output as torques -> use universal joint angle acceleration in getSensors
    //dJointAddUniversalTorques(joints[0], 0.5*m[0],0.5*m[1]);
  for (int i=0; i<sensorno/2; i++){
    dJointAddUniversalTorques( joints[i], 1.0*m[2*i],1.0*m[2*i +1]);  // motorcommand
    dJointAddUniversalTorques( joints[i], -(2.0*gamma)*dJointGetUniversalAngle1Rate(joints[i]),
			       -(2.0*gamma)*dJointGetUniversalAngle2Rate(joints[i]) );  // friction
  }
  





  //dBodyAddTorque(segments[0].body, 0.05*sin((double)t/40), 0, 0);

};

/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int FixedSnake2Elements::getSensors(sensor* sensors, int sensornumber){
  int len = (sensornumber < sensorno)? sensornumber : sensorno;

  /*  // universal joint angle 
  sensors[0]=dJointGetUniversalAngle1(joints[0]);
  sensors[1]=dJointGetUniversalAngle2(joints[0]);
  */

  
  // universal joint angle rate
  // tanh keeps the controller from being corrupted by to large sensorvalues, e.g. when the snake drops down
  for (int i=0; i<sensorno; i++){    
    if ((i%2)==0){
      sensors[i]=tanh( dJointGetUniversalAngle1Rate(joints[i/2]) );
    }
    else{
      sensors[i]=tanh( dJointGetUniversalAngle2Rate(joints[i/2]) );
    }
  }

  
  

  /*// universal joint angle acceleration 
    //sensors[0]=dJointGetUniversalAngle1(joints[0])-old_sensorvalues[0];
    //sensors[1]=dJointGetUniversalAngle2(joints[0])-old_sensorvalues[1];
    //old_sensorvalues[0]=dJointGetUniversalAngle1Rate(joints[0]);
    //old_sensorvalues[1]=dJointGetUniversalAngle2Rate(joints[0]);
  for (int i=0; i<sensorno; i++){
    if ((i%2)==0){
      mean_sensorvalues[i] = 0.2*( (dJointGetUniversalAngle1Rate(joints[i/2])-old_sensorvalues[i]) -mean_sensorvalues[i] );
      sensors[i] = mean_sensorvalues[i];
      old_sensorvalues[i] = dJointGetUniversalAngle1Rate(joints[i/2]);
    }
    else{
      mean_sensorvalues[i] = 0.2*( (dJointGetUniversalAngle2Rate(joints[i/2])-old_sensorvalues[i]) -mean_sensorvalues[i]);
      sensors[i] = mean_sensorvalues[i];
      old_sensorvalues[i] = dJointGetUniversalAngle2Rate(joints[i/2]);
    }
  }
  */


  /*//hinge2 angelrate
  sensors[0]=dJointGetHinge2Angle1Rate(joints[0]);
  sensors[1]=dJointGetHinge2Angle2Rate(joints[0]);
  */

  return len;
};

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
void FixedSnake2Elements::place(Position pos, Color *c /*= 0*/){
  if (!c==0) {
    color=*c;
  }
  pos.z+=glieder_durchmesser; // to put wheels on ground, not in ground
  if (!created){ 
    create(pos);
  }
  else{
    for (int i=0; i<segmentsno; i++){
      dGeomSetPosition(segments[i].geom,pos.x +i*(glieder_laenge+glieder_laenge/10), pos.y, pos.z);
    }
  }
};

  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects and the default routine)
   *  else false (collision is passed to other objects and (if not treated) to the default routine).
   */
void FixedSnake2Elements::doInternalStuff(const GlobalData& global){}
bool FixedSnake2Elements::collisionCallback(void *data, dGeomID o1, dGeomID o2){

    for ( int n = 0; n < (segmentsno-1); n++ )
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
Position FixedSnake2Elements::getPosition(){
  Position pos;
  const dReal* act_pos=dBodyGetPosition(segments[0].body);
  pos.x=act_pos[0];
  pos.y=act_pos[1];
  pos.z=act_pos[2]-glieder_durchmesser; // substract wheel radius, because vehicle stands on the ground
  return pos;
};

/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int FixedSnake2Elements::getSegmentsPosition(vector<Position> &poslist){
  Position pos;
  for (int i=0; i<segmentsno; i++){
    const dReal* act_pos = dBodyGetPosition(segments[i].body);
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
void FixedSnake2Elements::draw(){

  dsSetColor (color.r,color.g,color.b); 
  dsSetTexture (DS_WOOD);
  for (int i=0; i<segmentsno; i++){
    dsDrawCappedCylinder(  dGeomGetPosition ( segments[i].geom ) , dGeomGetRotation ( segments[i].geom ) , 
			   glieder_laenge , glieder_durchmesser );
  }
};



/** creates vehicle at desired position 
    @param pos struct Position with desired position
*/
void FixedSnake2Elements::create(Position pos){
  if (created) {
    destroy();
  }
  dMass masse;
  dMassSetCappedCylinder ( &masse , glieder_masse , 3, glieder_durchmesser, glieder_laenge  );

  dMatrix3 R;//Matrix fuer Koerper-Rotationen
  dRFromAxisAndAngle ( R , 0 , 1 , 0 , PI/2 );//hier drehung um 90Â° um die y-Achse

  for (int i=0; i<segmentsno; i++){
    segments[i].body=dBodyCreate ( world);
    dBodySetMass ( segments[i].body , &masse );
    segments[i].geom=dCreateCCylinder ( space , glieder_durchmesser , glieder_laenge );
    dGeomSetBody ( segments[i].geom , segments[i].body );
    dGeomSetPosition(segments[i].geom,pos.x +i*(glieder_laenge+glieder_laenge/10), pos.y, 
		     pos.z+glieder_durchmesser/*+2.0*glieder_laenge*/);
    dGeomSetRotation ( segments[i].geom, R );
  }


  useUniversalJoints();

  // useHinge2Joints();

  fixInSky();  // fix segment 0 in the sky (in initial Position)

  created=true;
}; 


/** destroys vehicle and space
 */
void FixedSnake2Elements::destroy(){
  if (created){
    for (int i=0; i<segmentsno; i++){
      dBodyDestroy(segments[i].body);
      dGeomDestroy(segments[i].geom);
    }
  }
  created=false;
}


/**
 *vereinfachte Objektpositions-Einzelermittlungsfunktion
 *@author Marcel Kretschmann
 *@version
 **/
double FixedSnake2Elements::dBodyGetPositionAll ( dBodyID basis , int para )
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

/** fix segment 0 in the sky
 */
void FixedSnake2Elements::fixInSky(){
  joints[ 2*(segmentsno-1) ]= ( dJointCreateHinge ( world , 0 ) );
  dJointAttach ( joints[ 2*(segmentsno-1) ] , segments[0].body , 0 );
  dJointSetUniversalAnchor ( joints[ 2*(segmentsno-1) ] , 
			     dBodyGetPositionAll ( segments[0].body , 1 ) , 
			     dBodyGetPositionAll ( segments[0].body , 2 ) , 
			     dBodyGetPositionAll ( segments[0].body , 3 ) ); 
  dJointSetHingeAxis(joints[ 2*(segmentsno-1) ],1,0,0);
  dJointSetFixed(joints[ 2*(segmentsno-1) ]);
  joints[ 2*(segmentsno-1) +1]= ( dJointCreateHinge ( world , 0 ) );
  dJointAttach ( joints[2*(segmentsno-1) +1] , segments[0].body , 0 );
  dJointSetUniversalAnchor ( joints[2*(segmentsno-1) +1] , 
			     dBodyGetPositionAll ( segments[0].body , 1 ) , 
			     dBodyGetPositionAll ( segments[0].body , 2 ) , 
			     dBodyGetPositionAll ( segments[0].body , 3 ) ); 
  dJointSetHingeAxis(joints[2*(segmentsno-1) +1],0,1,0);
  dJointSetFixed(joints[2*(segmentsno-1) +1]);
};



void FixedSnake2Elements::useUniversalJoints(){
  
  for (int i=0; i<segmentsno-1; i++){
    joints[i]= ( dJointCreateUniversal ( world , 0 ) );
    dJointAttach ( joints[i] , segments[i].body , segments[i+1].body );
			
    dJointSetUniversalAnchor ( joints[i] , 
			       dBodyGetPositionAll ( segments[i].body , 1 ) 
			       + ( dBodyGetPositionAll ( segments[i+1].body , 1 ) 
				   - dBodyGetPositionAll ( segments[i].body , 1 ) )/2 , 
			       dBodyGetPositionAll ( segments[i].body , 2 ), 
			       dBodyGetPositionAll ( segments[i].body , 3 ) ); 
    dJointSetUniversalAxis1 ( joints[i] , 0 , 1 , 0 ); 
    dJointSetUniversalAxis2 ( joints[i] , 0 , 0 , 1 ); 
    
    dJointSetUniversalParam (joints[i], dParamFudgeFactor, 0.1);
    dJointSetUniversalParam (joints[i], dParamBounce, 0.1);

    dJointSetUniversalParam (joints[i], dParamLoStop, -M_PI/4); 
    dJointSetUniversalParam (joints[i], dParamHiStop,  M_PI/4); 

    dJointSetUniversalParam (joints[i], dParamLoStop2, -M_PI/4); 
    dJointSetUniversalParam (joints[i], dParamHiStop2,  M_PI/4); 

  }
};


void FixedSnake2Elements::useHinge2Joints(){  // TODO: adapt to mot then to segments
  joints[0]= ( dJointCreateHinge2 ( world , 0 ) );
  dJointAttach ( joints[0] , segments[0].body , segments[1].body );
			
  dJointSetHinge2Anchor ( joints[0] , 
			  dBodyGetPositionAll ( segments[0].body , 1 ) 
			  + ( dBodyGetPositionAll ( segments[1].body , 1 ) 
			      - dBodyGetPositionAll ( segments[0].body , 1 ) )/2 , 
			  dBodyGetPositionAll ( segments[0].body , 2 ) 
			  /* + ( dBodyGetPositionAll ( segments[1].body , 2 ) 
			    - dBodyGetPositionAll ( segments[0].body , 2 )  )/2 */ , 
			  dBodyGetPositionAll ( segments[0].body , 3 ) );
  dJointSetHinge2Axis1 ( joints[0] , 0 , 1 , 0 );
  dJointSetHinge2Axis2 ( joints[0] , 0 , 0 , 1 );



  dJointSetHinge2Param (joints[0], dParamLoStop, -M_PI/8);
  dJointSetHinge2Param (joints[0], dParamHiStop,  M_PI/8);

  // Hi and Low stop nur an erster Achse möglich!
  //dJointSetHinge2Param (joints[0], dParamLoStop2, -M_PI/8);
  //dJointSetHinge2Param (joints[0], dParamHiStop2,  M_PI/8);
}



void FixedSnake2Elements::setMotorsHinge2Velocity(const motor*motors){// TODO: adapt to mot then to segments
  double tmp;
  tmp=dJointGetHinge2Param(joints[0],dParamVel);
  dJointSetHinge2Param(joints[0],dParamVel,tmp + 0.1*(motors[0]*20.0-tmp) );
  dJointSetHinge2Param (joints[0],dParamFMax,1);

  tmp=dJointGetHinge2Param(joints[0],dParamVel2);
  dJointSetHinge2Param(joints[0],dParamVel2,tmp + 0.1*(motors[1]*20.0-tmp) );
  dJointSetHinge2Param (joints[0],dParamFMax2,1);
}


                                           
void FixedSnake2Elements::setMotorsUniversalVelocity(const motor* motors){
  for (int i=0; i<sensorno; i++){
    if ((i%2)==0){
      dJointSetUniversalParam(joints[i/2], dParamVel,  0.5*motors[i] );
      dJointSetUniversalParam(joints[i/2], dParamFMax, 2);
    }
    else{
      dJointSetUniversalParam(joints[i/2], dParamVel2,  0.5*motors[i] );
      dJointSetUniversalParam(joints[i/2], dParamFMax2, 2);
    }
  }
};
