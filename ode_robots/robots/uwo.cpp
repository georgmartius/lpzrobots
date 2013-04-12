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
    addParameter("sliderpower",&conf.sliderPower,0,100);
  };


  int Uwo::getMotorNumber(){
    return servos.size()*2 + sliderservos.size();
  };

  int Uwo::getSensorNumber(){
    return servos.size()*2 + sliderservos.size();
  };

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Uwo::setMotors(const motor* motors, int motornumber){
    assert(created); // robot must exist
    // controller output as torques
    int n=0;
    FOREACH(vector <TwoAxisServo*>, servos, s){
      (*s)->set(motors[n],motors[n+1]);
      n+=2;
    }
    FOREACH(vector <OneAxisServo*>, sliderservos, s){
      (*s)->set(motors[n++]);
    }

  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Uwo::getSensors(sensor* sensors, int sensornumber){
    assert(created);
    int n=0;
    FOREACHC(vector <TwoAxisServo*>, servos, s){
      sensors[n++]   = (*s)->get1();
      sensors[n++]   = (*s)->get2();
    }
    FOREACHC(vector <OneAxisServo*>, sliderservos, s){
      sensors[n++]   = (*s)->get();
    }
    return n;
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
      joints.push_back(j);
      // setting stops at universal joints
      TwoAxisServo* servo =  new TwoAxisServoVel(odeHandle,
                                                 j, -conf.jointLimit, conf.jointLimit,
                                                 conf.motorPower,
                                                 -conf.jointLimit, conf.jointLimit,
                                                 conf.motorPower);
      servos.push_back(servo);

      if(conf.useSliders){
        Primitive* f;
        double sliderlen = conf.legLength/4.0;
        f = new Sphere(conf.legLength/8.0);
        f->init(odeHandle, legmass/8.0, osgHandle);
        Pos pos = Pos(0,0,-(conf.legLength/2+sliderlen));
        f->setPose( osg::Matrix::translate(pos) * p->getPose());
        objects.push_back(f);
        SliderJoint* sj = new SliderJoint(p, f, pos * p->getPose(), Axis(0,0,1)* p->getPose());
        sj->init(odeHandle, osgHandle, true, sliderlen);
        joints.push_back(sj);
        //        OneAxisServo* sliderservo =  new OneAxisServo( sj, -sliderlen/2.0, sliderlen/2.0,
        //                                                       conf.sliderPower );
        OneAxisServo* sliderservo =  new OneAxisServoVel(odeHandle,
                                                         sj, -sliderlen/2.0, sliderlen/2.0,
                                                         conf.sliderPower );
        sliderservos.push_back(sliderservo);
      }

    }

    created=true;
  };


  /** destroys vehicle and space
   */
  void Uwo::destroy(){
    if (created){
      FOREACH(vector<UniversalServo*>, servos, i){
        if(*i) delete *i;
      }
      FOREACH(vector<OneAxisServo*>, sliderservos, i){
        if(*i) delete *i;
      }
      servos.clear();
      sliderservos.clear();
      cleanup();
      odeHandle.deleteSpace();
    }
    created=false;
  }



  void  Uwo::notifyOnChange(const paramkey& key){
    FOREACH(vector<UniversalServo*>, servos, i){
      if(*i) (*i)->setPower(conf.motorPower, conf.motorPower);
    }
    FOREACH(vector<OneAxisServo*>, sliderservos, i){
      if(*i) (*i)->setPower(conf.sliderPower);
    }
  }


}
