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
#include "osgprimitive.h"

// include joints
#include "joint.h"

// include header file
#include "mirmorph.h"

using namespace osg;


namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  // - size of robot, maximal used force and speed factor are adjustable
  // - sphereWheels switches between spheres or wheels as wheels
  //   (wheels are only drawn, collision handling is always with spheres)
  Mirmorph::Mirmorph(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
               const std::string& name,
               MirmorphConf conf, double force /*=3*/, double speed/*=15*/)
    : // calling OdeRobots construtor with name of the actual robot
      OdeRobot(odeHandle, osgHandle, name, "$Id$")
  {
    // robot is not created till now
    created=false;

    // choose color (here the color of the "Nimm Zwei" candy is used,
    // where the name of the Nimm2 and Nimm4 robots comes from ;-)
    this->osgHandle.color = Color(255, 255, 255, 1.0f);



    // speed and type of wheels are set
    this->speed = speed;
    this->sphereWheels = conf.sphereWheels;

    height=conf.height;
    length=conf.length; // length of body
    width=conf.width;  // diameter of body
    radius=conf.radius; // wheel radius
    wheelthickness=conf.wheelthickness; // thickness of the wheels (if wheels used, no spheres)
    wheelheights = conf.wheelheights;

    double radius_average = 0;
    double wt_average = 0;
    for(int i=0; i<4; i++){
      radius_average += radius[i];
      wt_average = wheelthickness[i];
    }
    radius_average /= 4.0;
    wt_average /= 4.0;
    //cmass=8*(height/3 + length*2.5/3 + width*2/3);  // mass of the body
    //wmass=10*wt_average + 3*radius_average;    // mass of the wheels
    cmass = 27 * (length*width*height);
    wmass = 11 * (M_PI*radius_average*wt_average);
    sensorno=4;    // number of sensors
    motorno=4;     // number of motors
    segmentsno=5;  // number of segments of the robot

    // maximal used force is calculated from the force factor and size given to the constructor
    double size = height/5 + length*2.5/5 + width*4/5 + radius_average*6/5 + wt_average*20/5;
    max_force   = force*size*size;

    wheelsubstance.toRubber(50);
  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Mirmorph::setMotorsIntern(const double* motors, int motornumber){
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
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Mirmorph::getSensorsIntern(sensor* sensors, int sensornumber){
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


  void Mirmorph::placeIntern(const osg::Matrix& pose){
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
  void Mirmorph::update() {
    OdeRobot::update();
    assert(created); // robot must exist

    for (int i=0; i<segmentsno; i++) { // update objects
      objects[i]->update();
    }
    for (int i=0; i < 4; i++) { // update joints
      joints[i]->update();
    }

  };


  /** creates vehicle at desired pose
      @param pose matrix with desired position and orientation
  */
  void Mirmorph::create( const osg::Matrix& pose ){
    if (created) {  // if robot exists destroy it
      destroy();
    }
    // create car space
    odeHandle.createNewSimpleSpace(parentspace, true);
    objects.resize(5);  // 1 capsule, 4 wheels
    joints.resize(4); // joints between cylinder and each wheel

    OdeHandle wheelHandle(odeHandle);
    // make the material of the wheels a hard rubber
    wheelHandle.substance = wheelsubstance;
    // create box for main body
    // initialize it with ode-, osghandle and mass
    // rotate and place body (here by -90� around the y-axis)
    // use texture 'wood' for body
    // put it into objects[0]
    Box* body = new Box(length, width, height);
    //body->setTexture("Images/wood.rgb");
    body->init(odeHandle, cmass, osgHandle);
    //body->setPose(Matrix::rotate(M_PI, 0, 0, 1) * pose);
    body->setPose(Matrix::rotate(M_PI, 0, 1, 0) * pose);
    //body->setPose(pose);
    objects[0]=body;

    // create wheels
    /*   front
         -----
      1 |     | 2
        |     |
        |     |
      3 |     | 4
         -----
     */
    if(sphereWheels){
      for (int i=1; i<5; i++) {
        // create sphere with radius
        // and initialize it with odehandle, osghandle and mass
        // calculate position of wheels(must be at desired positions relative to the body)
        // rotate and place body (here by 90Deg around the x-axis)
        // set texture for wheels
        Sphere* sph = new Sphere(radius[i-1]);
        //sph->setTexture("Images/wood.rgb");
        sph->init(wheelHandle, wmass, osgHandle.changeColor(Color(0.8,0.8,0.8)));
        Vec3 wpos = Vec3( ((i-1)/2==0?-1:1)*(length/2.0-length/8.0),
                          ((i-1)%2==0?-1:1)*(width*0.5),
                          -wheelheights[i-1]);
        sph->setPose(Matrix::rotate(M_PI/2, 0, 0, 1) * Matrix::translate(wpos) * pose);
        objects[i]=sph;
      }
    }
    else{
      for (int i=1; i<5; i++) {
        // create cilinders with wheelthickness and radius
        // and initialize it with odehandle, osghandle and mass
        // calculate position of wheels(must be at desired positions relative to the body)
        // rotate and place body (here by 90Deg around the x-axis)
        // set texture for wheels
        Cylinder* cyl = new Cylinder(radius[i-1], wheelthickness[i-1]);
        //cyl->setTexture("Images/wood.rgb");
        cyl->init(wheelHandle, wmass, osgHandle.changeColor(Color(0.3, 0.3, 0.3)));
        Vec3 wpos = Vec3(((i-1)/2==0?-1:1)*(length/2.0-length/8.0),
                          ((i-1)%2==0?-1:1)*((width*0.5+wheelthickness[i-1]*0.5)+0.05),
                          -wheelheights[i-1]);
        cyl->setPose(Matrix::rotate(M_PI/2, 1, 0, 0) * Matrix::translate(wpos) * pose);
        objects[i]=cyl;
      }
    }

    // generate 4 joints to connect the wheels to the body
    for (int i=0; i<4; i++) {
      Pos anchor(dBodyGetPosition (objects[i+1]->getBody()));
      joints[i] = new Hinge2Joint(objects[0], objects[i+1], anchor, Axis(0,0,1)*pose, Axis(0,1,0)*pose);
      joints[i]->init(odeHandle, osgHandle, true, 1.01 * wheelthickness[i]);
    }
    for (int i=0; i<4; i++) {
      // set stops to make sure wheels always stay in alignment
      joints[i]->setParam(dParamLoStop, 0);
      joints[i]->setParam(dParamHiStop, 0);
    }

    created=true; // robot is created
  };


  /** destroys vehicle and space
   */
  void Mirmorph::destroy(){
    if (created){
      cleanup();
      odeHandle.deleteSpace(); // destroy space
    }
    created=false; // robot does not exist (anymore)
  }

}
