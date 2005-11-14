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
 *   Revision 1.5.4.1  2005-11-14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.5  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/
#include <drawstuff/drawstuff.h>
#include "simulation.h"
#include "schlangeservo.h"

SchlangeServo::SchlangeServo ( const OdeHandle& odeHandle, 
			       const SchlangeServoConf& conf, const char* n) 
  : OdeRobot(odeHandle, n) 
{
  this->conf = conf;
  created=0;

  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile$", 
			      "$Revision$");
}
	
SchlangeServo::~SchlangeServo()
{
  if(created) destroy();
}
	

void SchlangeServo::place (Position pos, Color *c)
{
  
  pos.z += conf.segmDia/2;
  if(c)
    color = *c;

  if (!created){ 
    create(pos);
  }
  else{  
    Position basepos(dBodyGetPosition( objects[0].body ));
    Position d = pos - basepos;
    
    for ( unsigned int n = 0; n < objects.size(); n++ ){
      Position npos (dBodyGetPosition( objects[n].body ));
      npos = npos + d;
      dBodySetPosition ( objects[n].body , npos.x, npos.y, npos.z );
    }
  }  
}

/**
 *Draws all elements of the snake.
 **/
void SchlangeServo::draw()
{
  assert(created);
  dsSetTexture (texture);
  dsSetColor ( color.r , color.g , color.b );

  for ( int n = 0; n < conf.segmNumber; n++ ) {
    dsDrawCappedCylinder ( dGeomGetPosition ( objects[n].geom ) , 
			   dGeomGetRotation ( objects[n].geom ) , 
			   conf.segmLength , conf.segmDia );
  }
}


void SchlangeServo::doInternalStuff(const GlobalData& global){}

/**
 *This is the collision handling function for snake robots.
 *This overwrides the function collisionCallback of the class robot.
 *@param data
 *@param o1 first geometrical object, which has taken part in the collision
 *@param o2 second geometrical object, which has taken part in the collision
 *@return true if the collision was threated  by the robot, false if not
 **/
bool SchlangeServo::collisionCallback(void *data, dGeomID o1, dGeomID o2)
{
  //checks if one of the collision objects is part of the robot
  if( o1 == (dGeomID)snake_space || o2 == (dGeomID)snake_space ){
    int i,n;  
    const int N = 20;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    for (i=0; i<n; i++){
      //      contact[i].surface.mode = dContactMu2 | dContactSlip1 | dContactSlip2 |
      //	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.slip1 = 0.001;
      contact[i].surface.slip2 = 0.001;
      contact[i].surface.mu = conf.frictionGround; //*10;
      //      contact[i].surface.mu2 = conf.frictionGround;
      contact[i].surface.soft_erp = 0.9;
      contact[i].surface.soft_cfm = 0.001;

      dJointID c = dJointCreateContact( world, contactgroup, &contact[i]);
      dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)); 
    }
    return true;
  }
  return false;
}


/**
 *Reads the actual motor commands from an array, an sets all motors (forces) of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 **/
void SchlangeServo::setMotors ( const motor* motors, int motornumber )
{
  assert(created);
  int len = min(motornumber, (int)servos.size());
  // controller output as torques 
  for (int i = 0; i < len; i++){
    servos[i]->set(motors[i]);
  }
}	

/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the arrays

 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 **/
int SchlangeServo::getSensors ( sensor* sensors, int sensornumber )
{
  assert(created);
  int len = min(sensornumber, getSensorNumber());
  
  for (int n = 0; n < len; n++) {
    sensors[n] = servos[n]->get();
  }
	
  return len;
}


/** The list of all parameters with there value as allocated lists.
    @param keylist,vallist will be allocated with malloc (free it after use!)
    @return length of the lists
*/
Configurable::paramlist SchlangeServo::getParamList() const{
  paramlist list;
  list += pair<paramkey, paramval> (string("frictionGround"), conf.frictionGround);
  return list;
}


Configurable::paramval SchlangeServo::getParam(const paramkey& key) const{
  
  if(key == "frictionGround") return conf.frictionGround; 
  else  return Configurable::getParam(key) ;
}

bool SchlangeServo::setParam(const paramkey& key, paramval val){
  
  if(key == "frictionGround") conf.frictionGround = val; 
  else return Configurable::setParam(key, val);
  return true;
}


Position SchlangeServo::getPosition(){
  assert(created);
  int n = conf.segmNumber/2;
  return Position(dBodyGetPosition( objects[n].body));
}

int SchlangeServo::getSegmentsPosition(vector<Position> &poslist){
  assert(created);
  for(int n = 0; n < conf.segmNumber; n++){
    poslist.push_back(Position(dBodyGetPosition( objects[n].body)));
  }
  return conf.segmNumber;

}


void SchlangeServo::create(const Position& pos){
  if (created) {
    destroy();
  }
  // create car space and add it to the top level space
  snake_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (snake_space,0);

  dMass masse;
  dMatrix3 R;	
  //*************body definition**************
	
  dMassSetCappedCylinderTotal ( &masse , conf.segmMass , 2 , conf.segmLength , conf.segmDia );
	
  dRFromAxisAndAngle ( R , 0 , 1 , 0 , M_PI/2 );//rotation of the matrix R by 90 degree

  int half = conf.segmNumber/2;
  for(int n = 0; n < conf.segmNumber; n++){
    Object obj;
    obj.body = dBodyCreate(world);
    Position p = pos + Position((n-half)*conf.segmLength, 0 , conf.segmDia/2);
            
    dBodySetPosition(obj.body, p.x, p.y, p.z);

    dBodySetMass(obj.body, &masse);
	
    obj.geom = dCreateCCylinder(snake_space, conf.segmDia, conf.segmLength);
    dGeomSetBody(obj.geom, obj.body);
    
    dGeomSetRotation(obj.geom, R); //includes rotation of the body
    objects.push_back(obj);
  }

  //*****************joint definition***********
  for ( int n = 0; n < conf.segmNumber-1; n++ ) {		
    dJointID joint = dJointCreateHinge(world, 0);
    dJointAttach ( joint , objects[n].body , objects[n+1].body );

    Position p1(dBodyGetPosition(objects[n].body));
    Position p2(dBodyGetPosition(objects[n+1].body));
    Position anchor = (p1 + p2)*0.5;
    dJointSetHingeAnchor(joint, anchor.x, anchor.y, anchor.z);
    
    dJointSetHingeAxis(joint, 0, 0, 1);
    
    // setting stops at universal joints		
    dJointSetHingeParam(joint, dParamLoStop, -conf.jointLimit);
    dJointSetHingeParam(joint, dParamHiStop,  conf.jointLimit);
    
    // making stops bouncy
    //    dJointSetUniversalParam ( jointliste.back () , dParamBounce, 0.9 );
    //    dJointSetUniversalParam ( jointliste.back () , dParamBounce2, 0.9 );
    joints.push_back(joint); 
    HingeServo* servo =  new HingeServo(joint, -conf.jointLimit, conf.jointLimit, conf.servoPower);
    servos.push_back(servo);
  }	  
  
  created = true;
}


/** destroys vehicle and space
 */
void SchlangeServo::destroy(){
  if (created){
    for (int i=0; i<conf.segmNumber; i++){
      dBodyDestroy(objects[i].body);
      dGeomDestroy(objects[i].geom);     
    }
    // Todo: delete bumpers
    dSpaceDestroy(snake_space);
  }
  created=false;
}

/** sets the texture */
void SchlangeServo::setTexture(int texture){
  this->texture=texture;
}
