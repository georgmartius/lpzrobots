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
 *   Revision 1.9.4.6  2006-06-25 17:00:32  martius
 *   Id
 *
 *   Revision 1.9.4.5  2006/06/25 16:57:13  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.9.4.4  2005/12/30 22:54:38  martius
 *   removed parentspace!
 *
 *   Revision 1.9.4.3  2005/12/21 17:34:59  martius
 *   moved to osg
 *
 *   Revision 1.9.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.9.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.9  2005/11/09 13:26:31  martius
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

using namespace std;

namespace lpzrobots {

  /**
   * Constructor
   * @param w world in which robot should be created
   * @param s space in which robot should be created
   * @param c contactgroup for collision treatment
   */
  HurlingSnake::HurlingSnake(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
			     const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), oldp(0,0,0){
    factorForce=3.0;
    factorSensor=20.0;
    frictionGround=0.3;

    created=false;

    this->osgHandle.color=Color(1,1,0);

    NUM= 10;		/* number of spheres */
    //    SIDE= 0.2;		/* side length of a box */
    MASS= 1.0;		/* mass of a sphere*/
    RADIUS= 0.1732f;	/* sphere radius */

    sensorno = 2;
    motorno  = 2;

  };

 
  void HurlingSnake::update(){
    assert(created); // robot must exist
  
    for (int i=0; i<NUM; i++) { 
      object[i]->update();
    }
    for (int i=0; i < NUM-1; i++) { 
      joint[i]->update();
    }
  }

  void HurlingSnake::place(const osg::Matrix& pose){
    // lift the snake about its radius
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, RADIUS)); 
    create(p2);    
  };

  void HurlingSnake::doInternalStuff(const GlobalData& global){
    // mycallback is called for internal collisions! Only once per step
    dSpaceCollide(odeHandle.space, this, mycallback);
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
      dJointID c = dJointCreateContact( me->odeHandle.world, me->odeHandle.jointGroup, &contact[i]);
      dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	      
    }
  }
  bool HurlingSnake::collisionCallback(void *data, dGeomID o1, dGeomID o2){
    //checks if one of the collision objects is part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space){

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
	dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
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
  
    Pos p(object[NUM-1]->getPosition());      //read actual position
    Pos s = (p - oldp)*factorSensor;
    
    sensors[0]=s.x();
    sensors[1]=s.y();
    oldp=p;
    return len;
  }


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  void HurlingSnake::setMotors(const motor* motors, int motornumber){    
    //  dBodyAddForce (object[NUM-1].body,motors[0]*factorForce,motors[1]*factorForce,motors[2]*factorForce);
    dBodyAddForce (object[NUM-1]->getBody(),motors[0]*factorForce,motors[1]*factorForce,0);
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


  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
  int HurlingSnake::getSegmentsPosition(vector<Position> &poslist){
    Position pos;
    for (int i=0; i<NUM; i++){
      Pos p = object[i]->getPosition();
      poslist.push_back(p.toPosition());
    }   
    return NUM;
  }



  void HurlingSnake::create(const osg::Matrix& pose){   
    if (created){
      destroy();
    }
    // create vehicle space and add it to parentspace
    odeHandle.space = dSimpleSpaceCreate (parentspace);

    for (int i=0; i<NUM; i++) {
      object[i] = new Sphere(RADIUS);      
      if (i==NUM-1){
	OsgHandle osgHandle_head = osgHandle;
	osgHandle_head.color = Color(1, 0, 0);
	object[i]->init(odeHandle, MASS, osgHandle_head);
      } else {
	object[i]->init(odeHandle, MASS, osgHandle);
      }
      object[i]->setPose(osg::Matrix::translate(i*RADIUS*2*1.1, 0, 0) * pose);
      object[i]->getOSGPrimitive()->setTexture("Images/wood.rgb");	      
    }
    oldp = object[NUM-1]->getPosition();
    for (int i=0; i<(NUM-1); i++) {
      Pos p1(object[i]->getPosition());
      Pos p2(object[i+1]->getPosition());
      joint[i] = new BallJoint(object[i],object[i+1], (p1+p2)/2 );
      joint[i]->init(odeHandle, osgHandle, true, RADIUS/10);
    }

    created=true;
  }


  /** destroys robot
   */
  void HurlingSnake::destroy(){
    if (created){
      for (int i=0; i<NUM; i++){
	if(object[i]) delete object[i];
      }
      for (int i=0; i<NUM-1; i++){
	if(joint[i]) delete joint[i];
      }
      dSpaceDestroy(odeHandle.space);
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
      OdeRobot::place(Pos(0,0,3)) ; 
    }
    else return Configurable::setParam(key, val);
    return true;
  }

} 
  
