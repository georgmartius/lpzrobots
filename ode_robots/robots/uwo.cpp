/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
#include <assert.h>
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include "primitive.h"

// include joints
#include "joint.h"
#include "twoaxisservo.h"

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
    addParameter("motorpower",&conf.motorPower,0,100);
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


  void Uwo::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8)); 
    create(p2);    
  };


  void Uwo::update(){
    assert(created); // robot must exist
  
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }

  };


  void Uwo::doInternalStuff(GlobalData& global){}

  void Uwo::create( const osg::Matrix& pose ){
    if (created) {
      destroy();
    }
    
    odeHandle.createNewSimpleSpace(parentspace,true);

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
      for (vector<UniversalServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
	if(*i) delete *i;
      }
      servos.clear();
      cleanup();
      odeHandle.deleteSpace();
    }
    created=false;
  }



  void  Uwo::notifyOnChange(const paramkey& key){    
    for (vector<UniversalServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
      if(*i) (*i)->setPower(conf.motorPower, conf.motorPower);
    }
  }


}
