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
 *   Revision 1.4  2005-08-03 20:35:28  martius
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
HurlingSnake::HurlingSnake(dWorldID w, dSpaceID s, dJointGroupID c):
  AbstractRobot(w, s, c){

  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile$", 
		           	            "$Revision$");

  factorForce=5.0;
  frictionGround=0.2;

  created=false;

  color.r=1;
  color.g=1;
  color.b=0.0;

  NUM= 10;		/* number of boxes */
  SIDE= 0.2;		/* side length of a box */
  MASS= 1.0;		/* mass of a box */
  RADIUS= 0.1732f;	/* sphere radius */

  sensorno = 2;
  motorno  = 2;

  for (int i=0; i<3; i++){
    old_position[i]=0.0;
  }
};
 
/// draws the robot
void HurlingSnake::draw(){
  dsSetTexture (DS_WOOD);
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
      double k = 1.3*i*SIDE;
      dBodySetPosition (object[i].body,pos.x+k,pos.y+k,pos.z+/*k+*/RADIUS);
    }
  };
}


// bool HurlingSnake::collisionCallback(void *data, dGeomID o1, dGeomID o2){

//   if ( isGeomInObjectList(object, NUM, o1)  ||  isGeomInObjectList(object, NUM, o1)  ){
//     int n;
//     dContact contact[10];
//     n=dCollide (o1,o2,10,&contact->geom,sizeof(dContact)); 
//     for( int i=0; i<n; i++){
//       contact[i].surface.mode = 0;
//       contact[i].surface.mu = frictionGround;
//       contact[i].surface.mu2 = 0;
      
//       dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
//       dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
//       //printf("contact created\n");
//     }
//     return true;
//   }
//   return false;
// }

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
    // mycallback is called for internal collisions!
    dSpaceCollide(snake_space, this, mycallback);

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
  
  const dReal* pos = pos;
  pos = dBodyGetPosition ( object[NUM-1].body );      //read actual position
  sensors[0]=(pos[0]-old_position[0])*3.0;     // calculate change of position during timestep	
  sensors[1]=(pos[1]-old_position[1])*3.0;
  //  sensors[2]=(pos[2]-old_position[2])*3.0;
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
      dBodySetPosition (object[i].body,pos.x+k,pos.y+k,pos.z+/*k+*/RADIUS);
      dMassSetBox (&m,1,SIDE,SIDE,SIDE);
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



/** The list of all parameters with there value as allocated lists.
    @param keylist,vallist will be allocated with malloc (free it after use!)
    @return length of the lists
*/
int HurlingSnake::getParamList(paramkey*& keylist,paramval*& vallist) const{
  int number_params=2; // don't forget to adapt number params!
  keylist=(paramkey*)malloc(sizeof(paramkey)*number_params);
  vallist=(paramval*)malloc(sizeof(paramval)*number_params);
  keylist[0]="factorForce";
  keylist[1]="frictionGround";  

  vallist[0]=factorForce;
  vallist[1]=frictionGround;
  return number_params;
}


paramval HurlingSnake::getParam(paramkey key) const{
  if(!key) return 0.0;
  if(strcmp(key, "factorForce")==0) return factorForce; 
  else if(strcmp(key, "frictionGround")==0) return frictionGround; 
  else  return Configurable::getParam(key) ;
}


bool HurlingSnake::setParam(paramkey key, paramval val){
  if(!key) {
    fprintf(stderr, "%s: empty Key!\n", __FILE__);
    return false;
  }
  if(strcmp(key, "factorForce")==0) factorForce=val;
  else if(strcmp(key, "frictionGround")==0) frictionGround=val; 
  else if(strcmp(key, "place")==0) {
    Position p(0,0,3);
    place(p) ; 
  }
  else return Configurable::setParam(key, val);
  return true;
}



 
  
