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
 *   Revision 1.4  2007-01-26 12:05:05  martius
 *   servos combinied into OneAxisServo
 *
 *   Revision 1.3  2006/07/20 17:19:45  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:42  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/06/25 16:57:50  martius
 *   Id
 *
 *   Revision 1.1.2.2  2006/06/25 16:57:17  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.1  2006/06/10 20:13:49  martius
 *   unknown walking object
 *
 *
 ***************************************************************************/
#include <assert.h>
#include <ode/ode.h>

// include primitives (box, spheres, cylinders ...)
#include "primitive.h"

// include joints
#include "joint.h"
#include "universalservo.h"

// include header file
#include "uwo.h"

using namespace osg;
using namespace std;

namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  Uwo::Uwo(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
	   const UwoConf& c, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(c)
  { 
    // robot is not created till now
    created=false;

    // choose color here a pastel white is used
    this->osgHandle.color = Color(1.0, 156/255.0, 156/255.0, 1.0f);
    
    conf.motorPower *= conf.mass;
    conf.legLength *= conf.size;
    legmass=conf.mass * conf.relLegmass / conf.legNumber;    // mass of each legs
  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  void Uwo::setMotors(const motor* motors, int motornumber){
    assert(created); // robot must exist

    int len = min(motornumber, getMotorNumber())/2;
    // controller output as torques 
    for (int i = 0; i < len; i++){
      servos[i]->set(motors[2*i], motors[2*i+1]);
    }

  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Uwo::getSensors(sensor* sensors, int sensornumber){
    assert(created);
    int len = min(sensornumber, getSensorNumber())/2;
    
    for (int n = 0; n < len; n++) {
      sensors[2*n]   = servos[n]->get1();
      sensors[2*n+1] = servos[n]->get2();
    }
    
    return 2*len;
  };


  void Uwo::place(const Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8)); 
    create(p2);    
  };


  /**
   * updates the osg notes
   */
  void Uwo::update(){
    assert(created); // robot must exist
  
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }

  };


  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
  */
  void Uwo::doInternalStuff(const GlobalData& global){}

  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects 
   *  and the default routine)  else false (collision is passed to other objects and 
   *  (if not treated) to the default routine).
   */
  bool Uwo::collisionCallback(void *data, dGeomID o1, dGeomID o2){
    assert(created); // robot must exist
    
    //checks if one of the collision objects is part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space ){
      int i,n;  
      const int N = 100;
      dContact contact[N];
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++){
	//      contact[i].surface.mode = dContactMu2 | dContactSlip1 | dContactSlip2 |
	//	dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |	
	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.slip1 = 0.005;
	contact[i].surface.slip2 = 0.005;
	contact[i].surface.mu = conf.frictionGround;
	contact[i].surface.soft_erp = 0.9;
	contact[i].surface.soft_cfm = 0.5;
	
	dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)); 
      }
      return true;
    }
    return false;
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void Uwo::create( const Matrix& pose ){
    if (created) {
      destroy();
    }
    
    odeHandle.space = dSimpleSpaceCreate (parentspace);

    // create body
    Primitive* trunk;
    double radius = conf.size / 2;
    trunk = new Cylinder(radius, conf.legLength / 5);
    trunk->init(odeHandle, conf.mass, osgHandle.changeAlpha(0.2));
    trunk->setPose(osg::Matrix::translate(0,0,conf.legLength / 5)*pose);
    objects.push_back(trunk);
    

    for ( int n = 0; n < conf.legNumber; n++ ) {
      double alpha = 2*M_PI*n/(double)conf.legNumber;
      Primitive* p;
      p = new Capsule(conf.legLength/8, conf.legLength); 
      p->init(odeHandle, legmass, osgHandle);
      Pos pos = Pos(sin(alpha) * radius * 0.8, 
		    cos(alpha) * radius * 0.8, 
		    -conf.legLength/2);
      p->setPose( osg::Matrix::translate(pos) * pose);
      objects.push_back(p);

      UniversalJoint* j;
      pos.z() = 0;
      
      if(conf.radialLegs){
	j = new UniversalJoint(trunk, p, pos * pose, 
			       Axis(cos(alpha),-sin(alpha),0)* pose, 
			       Axis(sin(alpha), cos(alpha),0)* pose);
      } else {
	j = new UniversalJoint(trunk, p, pos * pose, Axis(0,1,0)* pose, Axis(1,0,0)* pose);
      }
      j->init(odeHandle, osgHandle, true, conf.legLength/5 * 1.1);
      
      // setting stops at universal joints
      j->setParam(dParamLoStop, -conf.jointLimit*1.5);
      j->setParam(dParamHiStop,  conf.jointLimit*1.5);
      j->setParam(dParamLoStop2, -conf.jointLimit*1.5);
      j->setParam(dParamHiStop2,  conf.jointLimit*1.5);
      joints.push_back(j);
      UniversalServo* servo =  new UniversalServo(j, -conf.jointLimit, conf.jointLimit, 
						  conf.motorPower,
					          -conf.jointLimit, conf.jointLimit, 
						  conf.motorPower);
      servos.push_back(servo);

    }      
    
    created=true;
  }; 


  /** destroys vehicle and space
   */
  void Uwo::destroy(){
    if (created){
      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
	if(*i) delete *i;
      }
      objects.clear();
      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
	if(*i) delete *i;
      }
      joints.clear();
      for (vector<UniversalServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
	if(*i) delete *i;
      }
      servos.clear();
      dSpaceDestroy(odeHandle.space);
    }
    created=false;
  }



  /** The list of all parameters with there value as allocated lists.
      @param keylist,vallist will be allocated with malloc (free it after use!)
      @return length of the lists
  */
  Configurable::paramlist Uwo::getParamList() const{
    paramlist list;
    list += pair<paramkey, paramval> (string("frictionground"), conf.frictionGround);
    list += pair<paramkey, paramval> (string("motorpower"),   conf.motorPower);
    return list;
  }
  
  
  Configurable::paramval Uwo::getParam(const paramkey& key) const{    
    if(key == "frictionground") return conf.frictionGround; 
    else if(key == "motorpower") return conf.motorPower; 
    else  return Configurable::getParam(key) ;
  }
  
  bool Uwo::setParam(const paramkey& key, paramval val){    
    if(key == "frictionground") conf.frictionGround = val; 
    else if(key == "motorpower") {
      conf.motorPower = val; 
      for (vector<UniversalServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
	if(*i) (*i)->setPower(conf.motorPower, conf.motorPower);
      }
    } else 
      return Configurable::setParam(key, val);    
    return true;
  }


}
