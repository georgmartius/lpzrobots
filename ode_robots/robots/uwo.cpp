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
    this->osgHandle.color = osgHandle.getColor("robot1");
    conf.motorPower *= conf.mass;
    conf.legLength *= conf.size;
    legmass=conf.mass * conf.relLegmass / conf.legNumber;    // mass of each legs
    addParameter("motorpower",&conf.motorPower,0,100, "power of servo motors");
    addParameter("sliderpowerfactor",&conf.sliderPowerFactor,0,100, "power factor for slider servos");
    addParameter("jointlimit", &conf.jointLimit, 0,1,"joint limit for leg joints");
    addParameter("sliderlength", &conf.sliderLength, 0,1,"length of slider");

  };

  void Uwo::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8 +  + (conf.useSliders ?  conf.sliderLength/2 : 0.0 )));
    create(p2);
  };


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

    OsgHandle legOsgHandle = osgHandle.changeColor("robot2");

    for ( int n = 0; n < conf.legNumber; n++ ) {
      double alpha = 2*M_PI*n/(double)conf.legNumber;
      Primitive* p;
      p = new Capsule(conf.legLength/8, conf.legLength);
      p->init(odeHandle, legmass, legOsgHandle);
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

      auto servo = std::make_shared<TwoAxisServoVel>(odeHandle, j,-conf.jointLimit, conf.jointLimit, 1,
                                                     -conf.jointLimit, conf.jointLimit, 1);

      servo->setBaseName("leg " + itos(n));
      if(conf.radialLegs)
        servo->setNamingFunc([](int i){ return i==0? " in+/out-" : " clockwise+/counter-cw-";});
      else
        servo->setNamingFunc([](int i){ return i==0? "x" : "y";});
      servos.push_back(servo);
      addSensor(servo);
      addMotor(servo);

      if(conf.useSliders){
        Primitive* f;
        f = new Sphere(conf.legLength/8);
        f->init(odeHandle, legmass/8, osgHandle);
        Pos pos = Pos(0,0,-(conf.legLength/2+conf.sliderLength/1.6));
        f->setPose( osg::Matrix::translate(pos) * p->getPose());
        objects.push_back(f);
        SliderJoint* sj = new SliderJoint(p, f, pos * p->getPose(), Axis(0,0,1)* p->getPose());
        sj->init(odeHandle, osgHandle, true, conf.sliderLength+conf.legLength/16);
        joints.push_back(sj);
        // limits etc will be set in notify()
        auto sliderservo = std::make_shared<SliderServoVel>(odeHandle, sj,-1,1, 1);

        sliderservo->setBaseName("leg " + itos(n) + " slider");
        sliderservos.push_back(sliderservo);
        addSensor(sliderservo);
        addMotor(sliderservo);
      }
    }
    notifyOnChange(""); // set parameters
    created=true;
  };


  /** destroys vehicle and space
   */
  void Uwo::destroy(){
    if (created){
      cleanup();
      odeHandle.deleteSpace();
      servos.clear();
      sliderservos.clear();
    }
    created=false;
  }

  void  Uwo::notifyOnChange(const paramkey& key){
    for(auto& i : servos){
      i->setPower(conf.motorPower, conf.motorPower);
      i->setMinMax1(-conf.jointLimit,+conf.jointLimit);
      i->setMinMax2(-conf.jointLimit,+conf.jointLimit);
    }
    for(auto& i : sliderservos){
      i->setPower(conf.sliderPowerFactor*conf.motorPower);
      i->setMinMax(-conf.sliderLength/2,conf.sliderLength/2);
    }
  }


}
