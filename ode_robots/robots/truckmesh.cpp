/***************************************************************************
 *                                                                         *
 *  TruckMesh robot, an example for using Meshes for robot bodies          *
 *  basic code taken from the nimm4 robot                                  *
 *                                                                         *
 **************************************************************************/
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

// include primitives (box, spheres, cylinders, meshes ...)
#include "primitive.h"

// include joints
#include "joint.h"

// include header file
#include "truckmesh.h"
#include "osgprimitive.h" // get access to graphical (OSG) primitives

using namespace osg;


namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  // - size of robot, maximal used force and speed factor are adjustable
  TruckMesh::TruckMesh(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       const std::string& name,
                       double size/*=1.0*/, double force /*=3*/, double speed/*=15*/, double mass/*=1*/)
    : // calling OdeRobots construtor with name of the actual robot
    OdeRobot(odeHandle, osgHandle, name, "$Id$")
  {

    // robot is not created till now
    created=false;

    // choose color (here the color of the "Nimm Zwei" candy is used,
    // where the name of the Nimm2 and Nimm4 robots comes from ;-)
    this->osgHandle.color = Color(2, 156/255.0, 0, 1.0f);

    // maximal used force is calculated from the force factor and size given to the constructor
    max_force   = force*size*size;

    // speed and type of wheels are set
    this->speed = speed;

    height=size;

    length=size*1.5; // length of the truck
    middlewidth=size/10; // for y axis, it's the middle of the truck
    middlelength=-size*0.326;
    width=size*0.4;  // width of the truck
    radius=size*0.0995; // wheel radius
    wheelthickness=size/20; // thickness of the wheels (if wheels used, no spheres)
    cmass=mass*size*8;  // mass of the body
    wmass=mass*size;    // mass of the wheels
    sensorno=6;    // number of sensors
    motorno=6;     // number of motors
    segmentsno=7;  // number of segments of the robot
  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void TruckMesh::setMotorsIntern(const double* motors, int motornumber){
    assert(created); // robot must exist
    // the number of controlled motors is minimum of
    // "number of motorcommands" (motornumber) and
    // "number of motors inside the robot" (motorno)
    int len = (motornumber < motorno)? motornumber : motorno;

    // for each motor the motorcommand (between -1 and 1) multiplied with speed
    // is set and the maximal force to realize this command are set
    for (int i=0; i<len; i++){
      joints[i]->setParam(dParamVel2, motors[i]*speed);
      joints[i]->setParam(dParamFMax2, max_force);
    }

    // another possibility is to set half of the difference between last set speed
    // and the actual desired speed as new speed; max_force is also set
    /*
      double tmp;
      int len = (motornumber < motorno)? motornumber : motorno;
      for (int i=0; i<len; i++){
      tmp=dJointGetHinge2Param(joints[i],dParamVel2);
      dJointSetHinge2Param(joints[i],dParamVel2,tmp + 0.5*(motors[i]*speed-tmp) );
      dJointSetHinge2Param (joints[i],dParamFMax2,max_force);
      }
    */
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int TruckMesh::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created); // robot must exist

    // the number of sensors to read is the minimum of
    // "number of sensors requested" (sensornumber) and
    // "number of sensors inside the robot" (sensorno)
    int len = (sensornumber < sensorno)? sensornumber : sensorno;

    // for each sensor the anglerate of the joint is red and scaled with 1/speed
    for (int i=0; i<len; i++){
      sensors[i]=dynamic_cast<Hinge2Joint*>(joints[i])->getPosition2Rate();
      sensors[i]/=speed;  //scaling
    }
    // the number of red sensors is returned
    return len;
  };


  void TruckMesh::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, height*0.26));
    create(p2);
  };


  /**
   * updates the osg notes
   */
  void TruckMesh::update() {
    OdeRobot::update();
    assert(created); // robot must exist

    for (int i=0; i<segmentsno; i++) { // update objects
      objects[i]->update();
    }
    for (int i=0; i < 6; i++) { // update joints
      joints[i]->update();
    }

  };

  /** things for collision handling inside the space of the robot can be done here
   */
  void TruckMesh::mycallback(void *data, dGeomID o1, dGeomID o2){
    // do collisions handling for collisions between parts inside the space of the robot here
    // this has no meaning for this robot, because collsions between wheels and body are ignored
    // but if parts of the robot can move against each other this is important

    // the follwing (not active) code part can be used to check if objects which had collisions
    // are inside the list of objects of the robot
    /*  Nimm4* me = (Nimm4*)data;
        if(isGeomInObjectList(me->object, me->segmentsno, o1)
        && isGeomInObjectList(me->object, me->segmentsno, o2)){
        return;
        }
    */
  }

  /** this function is called in each timestep. It should perform robot-internal checks,
      like space-internal collision detection, sensor resets/update etc.
      @param global structure that contains global data from the simulation environment
  */
  void TruckMesh::doInternalStuff(GlobalData& global){}


  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void TruckMesh::create( const osg::Matrix& pose ){
    if (created) {  // if robot exists destroy it
      destroy();
    }
    // create car space and add it to the top level space
    odeHandle.createNewSimpleSpace(parentspace,true);

    objects.resize(7);  // 1 mesh, 6 wheels
    joints.resize(6); // joints between mesh and each wheel

    // create mesh for main body
    // initialize it with ode-, osghandle and mass
    // rotate and place body (here by 90° around the y-axis)
    // use texture 'wood' for mesh
    // put it into objects[0]
    Mesh* mesh = new Mesh("Meshes/dumptruck.osg",height/20.0f);
    mesh->getOSGPrimitive()->setTexture("Images/really_white.rgb");
    mesh->init(odeHandle, cmass, osgHandle);
    mesh->setPose(/*Matrix::rotate(M_PI/2, 0, 1, 0) */ pose);
    objects[0]=mesh;

    // create wheel bodies
    osgHandle.color= Color(1.0,1.0,1.0);
    for (int i=1; i<7; i++) {
      // create cylinder with radius and wheelthickness
      // and initializ it with odehandle, osghandle and mass
      // calculate position of wheels(must be at desired positions relative to the body)
      // rotate and place body (here by 90° around the x-axis)
      // set texture for wheels
      Cylinder* cyl=0;
      Vec3 wpos;
      if (i<3) { // back wheels
        cyl = new Cylinder(radius,wheelthickness*1.80);
        wpos = Vec3(middlelength-length*0.343,
                    middlewidth+((i-1)%2==0?-1.02:1)*width*0.35,
                    -height*0.302+radius );
      }
      else if (i<5){ // middle wheels
        cyl = new Cylinder(radius,wheelthickness*1.80);
        wpos = Vec3(middlelength-length*0.201,
                    middlewidth+((i-1)%2==0?-1.02:1)*width*0.35,
                    -height*0.302+radius );
      }
      else if (i<7){ // front wheels
        cyl = new Cylinder(radius,wheelthickness*1.02);
        wpos = Vec3(middlelength+length*0.407,
                    middlewidth+((i-1)%2==0?-1.05:1)*width*0.387,
                    -height*0.302+radius);
      }
      assert(cyl);
      cyl->getOSGPrimitive()->setTexture("Images/tire_full.rgb");
      cyl->init(odeHandle, wmass, osgHandle);
      cyl->setPose(Matrix::rotate(M_PI/2, 1, 0, 0) * Matrix::translate(wpos) * pose);
      objects[i]=cyl;
    }

    // generate 6 joints to connect the wheels to the body
    for (int i=0; i<6; i++) {
      Pos anchor(dBodyGetPosition (objects[i+1]->getBody()));
      joints[i] = new Hinge2Joint(objects[0], objects[i+1], anchor, Axis(0,0,1)*pose, Axis(0,1,0)*pose);
      joints[i]->init(odeHandle, osgHandle, true, 2.01 * wheelthickness);
    }
    for (int i=0; i<6; i++) {
      // set stops to make sure wheels always stay in alignment
      joints[i]->setParam(dParamLoStop, 0);
      joints[i]->setParam(dParamHiStop, 0);
    }

    created=true; // robot is created
  };


  /** destroys vehicle and space
   */
  void TruckMesh::destroy(){
    if (created){
      cleanup();
      odeHandle.deleteSpace();
    }
    created=false; // robot does not exist (anymore)
  }

}
