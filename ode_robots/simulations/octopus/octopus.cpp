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
 *   Revision 1.2  2010-03-09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.1  2009/05/05 08:27:30  martius
 *   taken from uwo and cleaned up
 *
 *   Revision 1.8  2008/05/07 16:45:52  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.7  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.6  2007/09/06 18:48:00  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.5  2007/02/13 19:32:20  martius
 *   twoaxisservo
 *
 *   Revision 1.4  2007/01/26 12:05:05  martius
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
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/twoaxisservo.h>

// include header file
#include "octopus.h"

using namespace osg;
using namespace std;

namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  Octopus::Octopus(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
           const OctopusConf& c, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(c)
  {
    // robot is not created till now
    created=false;

    // this makes the variable motorpower configurable (the reference is automatically overwritten)
    addParameter("motorpower", &conf.motorPower);

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
  void Octopus::setMotors(const motor* motors, int motornumber){
    assert(created); // robot must exist

    int len = min(motornumber, getMotorNumber())/2;
    // controller output as torques
    for (int i = 0; i < len; i++){
      servos[i]->setPower(conf.motorPower, conf.motorPower); // set power (because could be changed)
      servos[i]->set(motors[2*i], motors[2*i+1]);
    }

  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Octopus::getSensors(sensor* sensors, int sensornumber){
    assert(created);
    int len = min(sensornumber, getSensorNumber())/2;

    for (int n = 0; n < len; n++) {
      sensors[2*n]   = servos[n]->get1();
      sensors[2*n+1] = servos[n]->get2();
    }

    return 2*len;
  };


  void Octopus::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8));
    create(p2);
  };


  /**
   * updates the osg notes
   */
  void Octopus::update(){
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
      @param global structure that contains global data from the simulation environment
  */
  void Octopus::doInternalStuff(GlobalData& global){}

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void Octopus::create( const osg::Matrix& pose ){
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


    // create legs
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

      // legs either with radial joint orientation
      if(conf.radialLegs){
        j = new UniversalJoint(trunk, p, pos * pose,
                               Axis(cos(alpha),-sin(alpha),0)* pose,
                               Axis(sin(alpha), cos(alpha),0)* pose);
      } else { // or with the same joint orientation for all
        j = new UniversalJoint(trunk, p, pos * pose, Axis(0,1,0)* pose, Axis(1,0,0)* pose);
      }
      j->init(odeHandle, osgHandle.changeColor(Color(1,0,0)), conf.showJoints, conf.legLength/4 * 1.1);

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
  void Octopus::destroy(){
    if (created){
      for (vector<UniversalServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
        if(*i) delete *i;
      }
      servos.clear();
      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
        if(*i) delete *i;
      }
      joints.clear();
      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
        if(*i) delete *i;
      }
      objects.clear();
      odeHandle.deleteSpace();
    }
    created=false;
  }


}
