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
 *   Revision 1.9  2005-11-09 13:26:31  martius
 *   added factorSensors
 *
 *   Revision 1.8  2005/10/06 17:14:24  martius
 *   switched to stl lists
 *
 *   Revision 1.7  2005/09/22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.6  2005/08/31 17:18:15  fhesse
 *   setTextures added, Mass is now sphere (not box anymore)
 *
 *   Revision 1.5  2005/08/29 06:41:22  martius
 *   kosmetik
 *
 *   Revision 1.4  2005/08/03 20:35:28  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2005/07/27 13:23:09  martius
 *   new color and position construction
 *
 *   Revision 1.2  2005/07/26 17:04:21  martius
 *   lives in its own space now
 *
 *   Revision 1.1  2005/07/21 12:17:04  fhesse
 *   new hurling snake, todo: add collision space, clean up, comment
 *
 *         
 *                                                                 *
 ***************************************************************************/

#include "hurlingsnake.h"

#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include "simulation.h"

#include <iostream>

/**
 * Constructor
 * @param w world in which robot should be created
 * @param s space in which robot should be created
 * @param c contactgroup for collision treatment
 */
HurlingSnake::HurlingSnake(const OdeHandle& odeHandle):
  AbstractRobot(odeHandle){

  // prepare name;
  name.clear();
  Configurable::insertCVSInfo(name, "$RCSfile$", 
			      "$Revision$");

  factorForce=3.0;
  factorSensor=20.0;
  frictionGround=0.3;

  created=false;

  color.r=1;
  color.g=1;
  color.b=0.0;
  
  bodyTexture  = DS_WOOD;

  NUM= 10;		/* number of spheres */
  SIDE= 0.2;		/* side length of a box */
  MASS= 1.0;		/* mass of a sphere*/
  RADIUS= 0.1732f;	/* sphere radius */

  sensorno = 2;
  motorno  = 2;

  for (int i=0; i<3; i++){
    old_position[i]=0.0;
  }
};

/** sets the textures used for body and wheels
 */
void HurlingSnake::setTextures(int body){
  bodyTexture = body;
}

 
/// draws the robot
void HurlingSnake::draw(){
  dsSetTexture (bodyTexture);
  for (int i=0; i<NUM; i++) {
    dsSetColor (color.r, color.g, color.b);
    if (i==NUM-1)  dsSetColor (1, 0, 0);
    dsDrawSphere(dBodyGetPosition(object[i].body), dBodyGetRotation(object[i].body),RADIUS);
  }
}


/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
void HurlingSnake::place(Position pos , Color *c /*= 0*/){
  if (!c==0) {
    color=*c;
  }
  if (!created){ 
    create(pos);
  }
  else{
    for (int i=0; i<NUM; i++) {
      double k = 1.3*i*SIDE;  // SIDE larger then RADIUS
                              // 1.3*SIDE added in x and y direction
                              // -> difference between spheres centers larger then 2*RADIUS
      dBodySetPosition (object[i].body,pos.x+k,pos.y+k,pos.z+/*k+*/RADIUS);
    }
  };
}


void HurlingSnake::doInternalStuff(const GlobalData& global){
  // mycallback is called for internal collisions!
  dSpaceCollide(snake_space, this, mycallback);
}

void HurlingSnake::mycallback(void *data, dGeomID o1, dGeomID o2){
  // internal collisions
  HurlingSnake* me = (HurlingSnake*)data;  
  int i,n;  
  const int N = 10;
  dContact contact[N];  
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  for (i=0; i<n; i++){
    contact[i].surface.mode = 0;
    contact[i].surface.mu = 0;
    contact[i].surface.mu2 = 0;
//     contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
//       dContactSoftERP | dContactSoftCFM | dContactApprox1;
//     contact[i].surface.mu = 0.0;
//     contact[i].surface.slip1 = 0.005;
//     contact[i].surface.slip2 = 0.005;
//     contact[i].surface.soft_erp = 1;
//     contact[i].surface.soft_cfm = 0.00001;
    dJointID c = dJointCreateContact( me->world, me->contactgroup, &contact[i]);
    dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	      
  }
}
bool HurlingSnake::collisionCallback(void *data, dGeomID o1, dGeomID o2){
  //checks if one of the collision objects is part of the robot
  if( o1 == (dGeomID)snake_space || o2 == (dGeomID)snake_space){

    // the rest is for collisions of some snake elements with the rest of the world
    int i,n;  
    const int N = 10;
    dContact contact[N];

    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    for (i=0; i<n; i++){
           contact[i].surface.mode = 0;
           contact[i].surface.mu = frictionGround;
           contact[i].surface.mu2 = 0;
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
  return false;
}

  

/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] 
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int HurlingSnake::getSensors(sensor* sensors, int sensornumber){
  int len = (sensornumber < sensorno)? sensornumber : sensorno;
  
  const dReal* pos = dBodyGetPosition ( object[NUM-1].body );      //read actual position
  sensors[0]=(pos[0]-old_position[0])*factorSensor;     // calculate change of position during timestep	
  sensors[1]=(pos[1]-old_position[1])*factorSensor;
  //  sensors[2]=(pos[2]-old_position[2])*factorSensor;
  for (int i=0; i<3; i++){
    old_position[i]=pos[i];
  }
  return len;
}


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
void HurlingSnake::setMotors(const motor* motors, int motornumber){
  //  dBodyAddForce (object[NUM-1].body,motors[0]*factorForce,motors[1]*factorForce,motors[2]*factorForce);
  dBodyAddForce (object[NUM-1].body,motors[0]*factorForce,motors[1]*factorForce,0);
}


  /** returns number of sensors
   */
  int HurlingSnake::getSensorNumber(){
    return sensorno;
  }

  /** returns number of motors
   */
  int HurlingSnake::getMotorNumber(){
    return motorno;
  }

  /** returns position of robot 
      @param pos vector of desired position (x,y,z)
  */
Position HurlingSnake::getPosition(){
  Position pos;
  const dReal* act_pos= dBodyGetPosition ( object[NUM-1].body );      //read actual position
  pos.x=act_pos[0];
  pos.y=act_pos[1];
  pos.z=act_pos[2]-RADIUS; 
  return pos;
  
};

  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
  int HurlingSnake::getSegmentsPosition(vector<Position> &poslist){
    Position pos;
    for (int i=0; i<NUM; i++){
      const dReal* act_pos = dBodyGetPosition(object[i].body);
      pos.x=act_pos[0];
      pos.y=act_pos[1];
      pos.z=act_pos[2];
      poslist.push_back(pos);
    }   
    return NUM;
  }



  void HurlingSnake::create(Position pos){   
    if (created){
      destroy();
    }
    // create snake space and add it to the top level space
    snake_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (snake_space,0);

    dMass m;
    for (int i=0; i<NUM; i++) {
      object[i].body = dBodyCreate (world);
      double k = 1.3*i*SIDE;
      //double k = 2*i*RADIUS+0.1;
      dBodySetPosition (object[i].body,pos.x+k,pos.y+k,pos.z+/*k+*/RADIUS);
      dMassSetSphere (&m,1,SIDE);
      dMassAdjust (&m,MASS);
      dBodySetMass (object[i].body,&m);
      object[i].geom = dCreateSphere (snake_space,RADIUS);
      dGeomSetBody (object[i].geom,object[i].body);
    }
    for (int i=0; i<(NUM-1); i++) {
      joint[i] = dJointCreateBall (world,0);
      dJointAttach (joint[i],object[i].body,object[i+1].body);
      double k = 1.3*(i+0.5)*SIDE;
      dJointSetBallAnchor (joint[i],pos.x+k,pos.y+k,pos.z+/*k+*/RADIUS);
    }

    created=true;
  }


  /** destroys robot
   */
  void HurlingSnake::destroy(){
    if (created){
      dSpaceDestroy(snake_space);
      for (int i=0; i<NUM; i++){
	dBodyDestroy(object[i].body);
	dGeomDestroy(object[i].geom);
      }
    }
    created=false;
  }



Configurable::paramlist HurlingSnake::getParamList() const{
  paramlist list;
  list.push_back( pair<paramkey, paramval> (string("factorForce"), factorForce));
  list.push_back( pair<paramkey, paramval> (string("factorSensor"), factorSensor));
  list.push_back( pair<paramkey, paramval> (string("frictionGround"), frictionGround));
  list.push_back( pair<paramkey, paramval> (string("place"), 0));
  return list;
}


Configurable::paramval HurlingSnake::getParam(const paramkey& key) const{
  if(key == "factorForce") return factorForce; 
  else if(key == "factorSensor") return factorSensor; 
  else if(key == "frictionGround") return frictionGround; 
  else  return Configurable::getParam(key) ;
}


bool HurlingSnake::setParam(const paramkey& key, paramval val){
  if(key == "factorForce") factorForce=val;
  else if(key == "factorSensor") factorSensor=val; 
  else if(key == "frictionGround") frictionGround=val; 
  else if(key == "place") {
    Position p(0,0,3);
    place(p) ; 
  }
  else return Configurable::setParam(key, val);
  return true;
}



 
  
