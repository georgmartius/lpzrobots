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

#include "arm2segm.h"

using namespace std;

namespace lpzrobots{

  Arm2Segm::Arm2Segm(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const Arm2SegmConf armConf):
    OdeRobot(odeHandle, osgHandle,"Arm2Segm", "$Id$"), conf(armConf){

    parentspace=odeHandle.space;

    speed=1.0;
    factorSensors=2.0;
    sensorno=conf.segmentsno;
    motorno=conf.segmentsno;
    this->osgHandle.color = Color(1, 0, 0, 1.0f);

    addParameterDef("speed", &speed, 1.0, 0,10);
    addParameterDef("factorSensors", &factorSensors, 2, 0,10);

    created=false;
  };

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Arm2Segm::setMotorsIntern(const double* motors, int motornumber){
    assert(created); // robot must exist
    // the number of controlled motors is minimum of
    // "number of motorcommands" (motornumber) and
    // "number of motors inside the robot" (motorno)
    int len = (motornumber < motorno)? motornumber : motorno;

    // for each motor the motorcommand (between -1 and 1) multiplied with speed
    // is set (maximal force defined by amotors, see create() below)
    for (int i=0; i<len; i++){
      amotors[i]->set(1, motors[i]*speed);
    }

    // another possibility is to set half of the difference between last set speed
    // and the actual desired speed as new speed;
    /*
    double tmp;
    for (int i=0; i<len; i++){
      tmp=amotors[i]->get(1);
      amotors[i]->set(1,tmp + 0.5*(motors[i]*speed-tmp) );
    }
    */
  };


  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Arm2Segm::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created); // robot must exist

    // the number of sensors to read is the minimum of
    // "number of sensors requested" (sensornumber) and
    // "number of sensors inside the robot" (sensorno)
    int len = (sensornumber < sensorno)? sensornumber : sensorno;

    // for each sensor the anglerate of the joint is red and scaled with 1/speed
    for (int i=0; i<len; i++){
      sensors[i]=amotors[i]->get(1);  // is equal to: ((HingeJoint*)joints[i])->getPosition1Rate()
      // or read angle of each joint:
      // sensors[i]=((HingeJoint*)joints[i])->getPosition1();
      sensors[i]/=speed;  //scaling
    }
    // the number of red sensors is returned
    return len;
  };

  /** sets the vehicle to position pos, sets color to c, and creates robot if necessary
      @param pose desired position of the robot in struct Position
  */
  void Arm2Segm::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the base
    // to set it on the ground when the z component of the position is 0
    // base_length*0.5 is added (without this half of the base will be in the ground)
    osg::Matrix p2;
    p2 = pose
      // TODO: create is not robust enough to endure this pose !!
      //      * osg::Matrix::rotate(M_PI/2, 0, 0, 1)
      * osg::Matrix::translate(osg::Vec3(0, 0, conf.base_length* 0.5));
    create(p2);


      // p->setPose(osg::Matrix::rotate(M_PI/2, 0, 0, 1) *
//                  osg::Matrix::translate((n-half)*conf.segmLength, 0 , conf.segmDia/2) *
//                  pose);

  };


  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments)
      @return length of the list
  */
  int Arm2Segm::getSegmentsPosition(std::vector<Position> &poslist){
    for (int i=0; i<conf.segmentsno; i++){
      Pos p = objects[i]->getPosition();
      poslist.push_back(p.toPosition());
    }
    return conf.segmentsno;
  };


  void Arm2Segm::update() {
    OdeRobot::update();
    assert(created); // robot must exist


  };


  Primitive* Arm2Segm::getMainPrimitive() const{
    // at the moment returning position of last arm,
    // better would be a tip at the end of this arm, as in muscledArm
    return objects[conf.segmentsno-1];
  }


  /** creates vehicle at desired position
  */
  void Arm2Segm::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to parentspace
    odeHandle.createNewSimpleSpace(parentspace,false);

    // create base
    Primitive* o = new Box(conf.base_length, conf.base_width, conf.base_length);
    //o->getOSGPrimitive()->setTexture("Images/wood.rgb");
    o -> init(odeHandle, conf.base_mass, osgHandle);
    o->setPose( pose); // set base to given pose

    objects.push_back(o);

    // create arms
    for (int i=0; i<(conf.segmentsno-1); i++){
      o = new Box(conf.arm_length, conf.arm_width, conf.arm_width);
      o -> init(odeHandle, conf.arm_mass, osgHandle);
      // move arms to desired places
      o -> setPose(osg::Matrix::translate(osg::Vec3(// shift (armlength -JOINT_OFFSET) to have overlapp
                                                    (i+0.5)*conf.arm_length - (i+1)*conf.joint_offset,
                                                    // shift to have place between arms
                                                    (i+1)*conf.arm_offset
                                                    + (i+0.5)*conf.arm_width + 0.5*conf.base_width,
                                                    // height is ok, no shift
                                                    0) ) * pose);
      objects.push_back(o);
    }

    // hinge joint and angular motor to connect base with world
    Pos p1(objects[0]->getPosition());
    HingeJoint* j = new HingeJoint(0, objects[0], p1, osg::Vec3(0,0,1) /** pose*/);
    j -> init(odeHandle, osgHandle,/*withVisual*/true);
    joints.push_back(j);
    AngularMotor1Axis* a=new AngularMotor1Axis(odeHandle, (OneAxisJoint *)joints[0], conf.max_force);
    amotors.push_back(a);

    // hinge joint and angular motor to connect base with first arm
    Pos p2(objects[1]->getPosition());
    p1[1]=(p1[1]+p2[1])/2;
    j = new HingeJoint(objects[0], objects[1], p1, osg::Vec3(0,1,0) /** pose*/);
    j -> init(odeHandle, osgHandle,/*withVisual*/true);
    joints.push_back(j);
    a=new AngularMotor1Axis(odeHandle, (OneAxisJoint *)joints[1], conf.max_force);
    amotors.push_back(a);

    // hinge joint and angular motor to connect arms
    for (int i=2; i<conf.segmentsno; i++) {
      Pos po1(objects[i-1]->getPosition());
      Pos po2(objects[i]->getPosition());
      Pos po3( (po1+po2)/2);
      j = new HingeJoint(objects[i-1], objects[i], po3, osg::Vec3(0,1,0) /** pose*/);
      j -> init(odeHandle, osgHandle,/*withVisual*/true);
      joints.push_back(j);
      a=new AngularMotor1Axis(odeHandle, (OneAxisJoint *)joints[i], conf.max_force);
      amotors.push_back(a);
    }

// --------------
// TODO: add tip at the end of arm for easier getPosition; temporarily positioning of transform object does not work
    osg::Matrix ps;
    ps.makeIdentity();
    Primitive* o1 = new Sphere(conf.arm_width*0.8);
    Primitive* o2 = new Transform(objects[objects.size()-1], o1, osg::Matrix::translate(0, conf.arm_length*0.5, 0) * ps);
    o2->init(odeHandle, /*mass*/0, osgHandle, /*withBody*/ false);
// --------------

    created=true;
  };


  /** destroys vehicle and space
   */
  void Arm2Segm::destroy(){
    if (created){
      for (vector<AngularMotor1Axis*>::iterator i=amotors.begin(); i!=amotors.end(); i++){
        if (*i) delete *i;
      }
      amotors.clear();
      for (vector<Joint*>::iterator i=joints.begin(); i!=joints.end(); i++){
        if (*i) delete *i;
      }
      joints.clear();
      for (vector<Primitive*>::iterator i=objects.begin(); i!=objects.end(); i++){
        if (*i) delete *i;
      }
      objects.clear();
      odeHandle.deleteSpace();
    }
    created=false;
  };

}
