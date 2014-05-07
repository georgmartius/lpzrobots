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

// include primitives (box, spheres, cylinders ...)
#include "primitive.h"
#include "osgprimitive.h"

// include joints
#include "joint.h"

// include header file
#include "formel1.h"


using namespace osg;

namespace lpzrobots {

  // constructor:
  // - size of robot, maximal used force and speed factor are adjustable
  // - sphereWheels switches between spheres or wheels as wheels
  //   (wheels are only drawn, collision handling is always with spheres)
  Formel1::Formel1(const OdeHandle& odeHandle, const OsgHandle& osgHandle, double size/*=1.0*/,
               double force /*=3*/, double speed/*=15*/, bool sphereWheels /*=true*/)
    : // calling OdeRobots construtor with name of the actual robot
      OdeRobot(odeHandle, osgHandle, "Formel1", "$Id$")
  {

    // robot is not created till now
    created=false;

    // choose color (here the color of the "Nimm Zwei" candy is used,
    // where the name of the Nimm2 and Formel1 robots comes from ;-)
    this->osgHandle.color = Color(2, 156/255.0, 0, 1.0f);

    // maximal used force is calculated from the forece factor and size given to the constructor
    max_force   = force*size*size;

    // speed and type of wheels are set
    this->speed = speed;
    this->sphereWheels = sphereWheels;

    height=size;

    length=size/2.5; // length of body
    width=size/2;  // radius of body
    radius=size/6; // wheel radius
    wheelthickness=size/20; // thickness of the wheels (if wheels used, no spheres)
    cmass=8*size;  // mass of the body
    wmass=size;    // mass of the wheels
    sensorno=4;    // number of sensors
    motorno=4;     // number of motors
    segmentsno=5;  // number of segments of the robot
  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Formel1::setMotorsIntern(const double* motors, int motornumber){
    assert(created); // robot must exist
    // the number of controlled motors is minimum of
    // "number of motorcommands" (motornumber) and
    // "number of motors inside the robot" (motorno)
    int len = (motornumber < motorno)? motornumber : motorno;

    // for each motor the motorcommand (between -1 and 1) multiplied with speed
    // is set and the maximal force to realize this command are set
    for (int i=0; i<len; i++){
      joint[i]->setParam(dParamVel2, motors[i]*speed);
      joint[i]->setParam(dParamFMax2, max_force);
    }

    // another possibility is to set half of the difference between last set speed
    // and the actual desired speed as new speed; max_force is also set
    /*
      double tmp;
      int len = (motornumber < motorno)? motornumber : motorno;
      for (int i=0; i<len; i++){
      tmp=dJointGetHinge2Param(joint[i],dParamVel2);
      dJointSetHinge2Param(joint[i],dParamVel2,tmp + 0.5*(motors[i]*speed-tmp) );
      dJointSetHinge2Param (joint[i],dParamFMax2,max_force);
      }
    */
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Formel1::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created); // robot must exist

    // the number of sensors to read is the minimum of
    // "number of sensors requested" (sensornumber) and
    // "number of sensors inside the robot" (sensorno)
    int len = (sensornumber < sensorno)? sensornumber : sensorno;

    // for each sensor the anglerate of the joint is red and scaled with 1/speed
    for (int i=0; i<len; i++){
      sensors[i]=joint[i]->getPosition2Rate();
      sensors[i]/=speed;  //scaling
    }
    // the number of red sensors is returned
    return len;
  };


  void Formel1::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, width*0.6));
    create(p2);
  };


  /**
   * updates the osg notes
   */
  void Formel1::update() {
    OdeRobot::update();
    assert(created); // robot must exist

    for (int i=0; i<segmentsno; i++) {
      object[i]->update();
    }
    for (int i=0; i < 4; i++) {
      joint[i]->update();
    }

  };

  /** things for collision handling inside the space of the robot can be done here
   */
  void Formel1::mycallback(void *data, dGeomID o1, dGeomID o2){
    // do collisions handling for collisions between parts inside the space of the robot here
    // this has no meaning for this robot, because collsions between wheels and body are ignored
    // but if parts of the robot can move against each other this is important

    // the follwing (not active) code part can be used to check if objects which had collisions
    // are inside the list of objects of the robot
    /*  Formel1* me = (Formel1*)data;
        if(isGeomInObjectList(me->object, me->segmentsno, o1)
        && isGeomInObjectList(me->object, me->segmentsno, o2)){
        return;
        }
    */
  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void Formel1::create( const osg::Matrix& pose ){
    if (created) {  // if robot exists destroy it
      destroy();
    }
    // create car space and add it to the top level space
    odeHandle.createNewSimpleSpace(parentspace,true);

    Capsule* cap = new Capsule(width/2, length);
    cap->setTexture("Images/wood.rgb");
    cap->init(odeHandle, cmass, osgHandle);
    // rotate and place body (here by 90° around the y-axis)
    cap->setPose(Matrix::rotate(M_PI/2, 0, 1, 0) * pose);
    object[0]=cap;

    // create wheel bodies
    osgHandle.color= Color(0.8,0.8,0.8);
    for (int i=1; i<5; i++) {

      Sphere* sph = new Sphere(radius);
      sph->setTexture("Images/wood.rgb");
      sph->init(odeHandle, wmass, osgHandle);
      // rotate and place body (here by 90° around the x-axis)
      Vec3 wpos = Vec3( ((i-1)/2==0?-1:1)*length/2.0,
                        ((i-1)%2==0?-1:1)*(width*0.5+wheelthickness),
                        -width*0.6+radius );
      sph->setPose(Matrix::rotate(M_PI/2, 0, 0, 1) * Matrix::translate(wpos) * pose);
      object[i]=sph;
    }

    // generate 4 joints to connect the wheels to the body
    for (int i=0; i<4; i++) {
      Pos anchor(dBodyGetPosition (object[i+1]->getBody()));
      joint[i] = new Hinge2Joint(object[0], object[i+1], anchor, Vec3(0,0,1), Vec3(0,1,0));
      joint[i]->init(odeHandle, osgHandle, true, 2.01 * radius);
    }
    for (int i=0; i<4; i++) {
      // set stops to make sure wheels always stay in alignment
      joint[i]->setParam(dParamLoStop, 0);
      joint[i]->setParam(dParamHiStop, 0);
    }

    created=true; // robot is created
  };


  /** destroys vehicle and space
   */
  void Formel1::destroy(){
    if (created){
      for (int i=0; i<4; i++){
        if(joint[i]) delete joint[i]; // destroy bodies and geoms
      }
      for (int i=0; i<segmentsno; i++){
        if(object[i]) delete object[i]; // destroy bodies and geoms
      }
      odeHandle.deleteSpace();
    }
    created=false; // robot does not exist (anymore)
  }

}
