/***************************************************************************
 *   Copyright (C) 2012 by1                                                *
 *    Timo Nachstedt <nachstedt@physik3.gwdg.de>                           *
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

#include "nejihebi.h"

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>
// include joints
#include <ode_robots/joint.h>
// include servo motors
#include <ode_robots/twoaxisservo.h>
// include standard algorithm methods (e.g. min, max)
#include <algorithm>

namespace lpzrobots {

  Nejihebi::Nejihebi(const OdeHandle& odeHandle,
      const OsgHandle& osgHandle, const Conf& c, const std::string& name)
      : OdeRobot(odeHandle, osgHandle, name, "0.1"), conf(c) {
    head = 0;
    setNameOfInspectable("Nejihebi");
    addInspectableDescription("test", "ein test");
  }

  Nejihebi::~Nejihebi() {
    for(unsigned int i=0; i<servos.size(); i++)
      if (servos[i]) delete servos[i];
  }

  void Nejihebi::create(const osg::Matrix& pose) {
    double screwdifference = conf.innerBody.frontLength
        + conf.innerBody.backLength;

    // create screws
    for (int i=0; i<conf.numberOfScrews; i++)
      screws.push_back( createScrew(
          osg::Matrix::translate(1.0*i*screwdifference,0,0)*pose, i%2));

    // create joints between screws
    for (int i=0; i<conf.numberOfScrews-1; i++)
    {
      UniversalJoint * joint = new UniversalJoint(screws[i].innerPart,
          screws[i+1].innerPart,
          screws[i].screwBase->getPosition()
          * osg::Matrix::translate(conf.innerBody.backLength,0,0),
          Axis(0,0,1)*pose,
          Axis(0,1,0)*pose);
      joint->init(odeHandle, osgHandle);
      joints.push_back(joint);
      // create corresponding motor
      TwoAxisServoVel * servo = new TwoAxisServoVel(odeHandle, joint,
          conf.jointUnit.yaw.minAngle,
          conf.jointUnit.yaw.maxAngle,
          conf.jointUnit.yaw.power,
          conf.jointUnit.pitch.minAngle,
          conf.jointUnit.pitch.maxAngle,
          conf.jointUnit.pitch.power,
          conf.jointUnit.damping,
          conf.jointUnit.maxVel);
      servos.push_back(servo);
    }

    // create head
    head = new Box(conf.head.length, conf.head.width, conf.head.height);
    head->init(odeHandle, conf.head.mass,
        osgHandle.changeColor(conf.head.color));
    head->setPose(osg::Matrix::translate(
        -conf.innerBody.frontLength-0.5*conf.head.length,0,0)*pose);
    objects.push_back(head);
    // fix head to inner body of first screw
    if (conf.numberOfScrews>0) {
      FixedJoint * joint = new FixedJoint(screws[0].innerPart, head);
      joint->init(odeHandle, osgHandle, false);
    }
    // create  ball bearings
    for (int i=0; i<2; i++) {
      const double pm = 1-2*(i%2);
      // stick for ball bearing
      Box * stick = new Box(conf.ballBearing.stickWidth,
          conf.ballBearing.stickWidth, conf.ballBearing.stickLength);
      Transform * stickTransform = new Transform(head, stick,
          osg::Matrix::translate(0,
              -0.5*pm*(conf.head.width-conf.ballBearing.stickWidth),
              -0.5*(conf.head.height+conf.ballBearing.stickLength)));
      stickTransform->init(odeHandle, conf.ballBearing.stickMass,
          osgHandle.changeColor(conf.ballBearing.stickColor));
      objects.push_back(stickTransform);
      // ball
      Sphere * ball = new Sphere(conf.ballBearing.radius);
      ball->init(odeHandle, conf.ballBearing.mass,
          osgHandle.changeColor(conf.ballBearing.color));
      const Pose ballPosition = osg::Matrix::translate(
              -conf.innerBody.frontLength-0.5*conf.head.length,
              0,
              0)
          * osg::Matrix::translate(0,
              -0.5*pm*(conf.head.width-conf.ballBearing.stickWidth),
              -0.5*conf.head.height-conf.ballBearing.stickLength)
          * pose;
      ball->setPose(ballPosition);
      objects.push_back(ball);
      // ball joint
      BallJoint * joint = new BallJoint(stickTransform, ball,
          ball->getPosition());
      joint->init(odeHandle, osgHandle, false);
      joints.push_back(joint);
    }

    // name motors and senors
    for (int i=0; i<conf.numberOfScrews; i++) {
      nameMotor(i, "goal speed screw " + std::itos(i));
      nameSensor(i, "present angle of screw " + std::itos(i));
    }
    for (int i=0; i<conf.numberOfScrews-1; i++) {
      nameMotor(conf.numberOfScrews+2*i,
          "goal position of yaw joint " + std::itos(i));
      nameMotor(conf.numberOfScrews+2*i+1,
          "goal position of pitch joint " + std::itos(i));
      nameMotor(3*conf.numberOfScrews-2+2*i,
          "torque limit of yaw joint " + std::itos(i));
      nameMotor(3*conf.numberOfScrews-2+2*i+1,
          "torque limit of pitch joint " + std::itos(i));
      nameSensor(conf.numberOfScrews+4*i,
          "present position of yaw joint " + std::itos(i));
      nameSensor(conf.numberOfScrews+4*i+1,
          "present speed of yaw joint " + std::itos(i));
      nameSensor(conf.numberOfScrews+4*i+2,
          "present position of pitch joint " + std::itos(i));
      nameSensor(conf.numberOfScrews+4*i+3,
          "present speed of pitch joint " + std::itos(i));
    }
    // name internal inspectables
    for (int i=0; i<conf.numberOfScrews; i++) {
      const std::string n =std::itos(i);
      addInspectableDescription("screw_"+n+"_angle", "angle of screw "+n);
      addInspectableDescription("screw_"+n+"_speed", "speed of screw "+n);
    }
    for (int i=0; i<conf.numberOfScrews-1; i++) {
      const std::string n =std::itos(i);
      addInspectableDescription("yaw_"+n+"_angle", "angle of yaw joint "+n);
      addInspectableDescription("yaw_"+n+"_speed", "speed of yaw joint "+n);
      addInspectableDescription("yaw_"+n+"_power", "power of yaw joint "+n);
      addInspectableDescription("pitch_"+n+"_angle", "angle of pitch joint "+n);
      addInspectableDescription("pitch_"+n+"_speed", "speed of pitch joint "+n);
      addInspectableDescription("pitch_"+n+"_power", "power of pitch joint "+n);
    }
  }

  Nejihebi::Screw Nejihebi::createScrew(const osg::Matrix& pose, const bool inverted) {
    // inner body part

    Box* innerPart = new Box(
        conf.innerBody.frontLength+conf.innerBody.backLength,
        conf.innerBody.width, conf.innerBody.height);
    innerPart->init(odeHandle, conf.innerBody.mass, osgHandle.changeColor(conf.innerBody.color));
    innerPart->setPose(
        osg::Matrix::translate(0.5*(conf.innerBody.backLength-conf.innerBody.frontLength),0,0)
        * pose);
    objects.push_back(innerPart);

    // screw base component
    Cylinder* unitBase = new Cylinder(conf.screwbase.radius,
        conf.screwbase.width);
    unitBase->init(odeHandle, conf.screwbase.mass, osgHandle.changeColor(conf.screwbase.color));
    unitBase->setPose(
        osg::Matrix::rotate(0.5*M_PI,0,1,0)
        * osg::Matrix::translate(0, 0, 0)
        * pose);
    objects.push_back(unitBase);

    // rotation joint
    HingeJoint * rotationJoint = new HingeJoint (innerPart, unitBase,
        unitBase->getPosition(), Axis(1,0,0) * pose);
    rotationJoint->init(odeHandle, osgHandle);
    joints.push_back(rotationJoint);

    // blades
    for (int i = 0; i<conf.screwbase.blades; i++){
      const double angle = 2.0*M_PI*(double(i)/double(conf.screwbase.blades));
      Box * blade = new Box(conf.blade.length, conf.blade.width, conf.blade.height);
      const double anglefactor = inverted ? -1 : 1;
      Transform * bladeTransform = new Transform(unitBase, blade,
          osg::Matrix::rotate(0.50*M_PI,0,1,0)
          * osg::Matrix::rotate(anglefactor * conf.blade.angle,1,0,0)
          * osg::Matrix::translate(conf.screwbase.radius,0,0)
          * osg::Matrix::translate(0.5*conf.blade.height,0,0)
          * osg::Matrix::rotate(angle,0,0,1)
      );
      bladeTransform->init(odeHandle, conf.blade.mass, osgHandle.changeColor(conf.blade.color));
      objects.push_back(bladeTransform);
      // wheels
      for (int j=0; j < conf.blade.wheels; j++) {
        Cylinder* wheel = new Cylinder(conf.wheel.radius, conf.wheel.width);
        wheel->init(odeHandle, conf.wheel.mass, osgHandle.changeColor(conf.wheel.color));
        wheel->setPose(
            osg::Matrix::translate(
                conf.wheel.posradius,
                0,
                0.5*(2*(j%2)-1)*(conf.blade.width+conf.wheel.width))
            * osg::Matrix::rotate(anglefactor*conf.blade.angle-0.5*M_PI,1,0,0)
            * osg::Matrix::rotate(
                conf.wheel.possegment
                  *(conf.blade.wheels==1
                      ? 0
                      : double(j)/double(conf.blade.wheels-1)-0.5)
                ,0,
                cos(anglefactor*conf.blade.angle),
                sin(anglefactor*conf.blade.angle))
            * osg::Matrix::translate(conf.wheel.posshift,0,0)
            * osg::Matrix::rotate(angle,0,0,1)
            * unitBase->getPose());
        objects.push_back(wheel);
        odeHandle.addIgnoredPair(wheel, bladeTransform);
        HingeJoint* joint = new HingeJoint(unitBase, wheel,
            wheel->getPosition(),
            Axis(0,1,0)
                * osg::Matrix::rotate(-anglefactor*conf.blade.angle,0,0,1)
                * osg::Matrix::rotate(angle,1,0,0)
        );
        joint->init(odeHandle, osgHandle, conf.wheel.axisVisible,
            conf.wheel.axisSize);
        joints.push_back(joint);
      }
    }
    Screw s;
    s.innerPart = innerPart;
    s.joint     = rotationJoint;
    s.screwBase = unitBase;
    return s;
  }

  const Nejihebi::Conf& Nejihebi::getConf() const {
    return conf;
  }

  Nejihebi::Conf Nejihebi::getDefaultConf() {
    Conf c;
    c.ballBearing.color        = Color(0.8,0.8,0.8);
    c.ballBearing.mass         = 0.010; //kg
    c.ballBearing.radius       = 0.100; //dm
    c.ballBearing.stickColor   = Color(0.2,0.2,0.2);
    c.ballBearing.stickLength  = 0.550; //dm
    c.ballBearing.stickMass    = 0.05; //kg
    c.ballBearing.stickWidth   = 0.050; //dm
    c.head.color               = Color(0.4,0,0.1);
    c.head.width               = 1.300; //dm
    c.head.height              = 0.200; //dm
    c.head.length              = 0.550; //dm
    c.head.mass                = 0.2; //kg
    c.innerBody.color          = Color(0.2,0.2,0.2);
    c.innerBody.frontLength    = 1.030; //dm
    c.innerBody.backLength     = 1.230; //dm
    c.innerBody.height         = 0.200; //dm
    c.innerBody.mass           = 0.5; //kg
    c.innerBody.width          = 0.200; //dm
    c.jointUnit.damping        = 0.05;
    c.jointUnit.maxVel         = 10.0;
    c.jointUnit.pitch.minAngle = -0.5*M_PI;
    c.jointUnit.pitch.maxAngle = 0.5*M_PI;
    c.jointUnit.pitch.power    = 500;
    c.jointUnit.yaw.minAngle   = -0.5*M_PI;
    c.jointUnit.yaw.maxAngle   = 0.5*M_PI;
    c.jointUnit.yaw.power      = 500;
    c.numberOfScrews           = 4;
    c.blade.angle              = 0.25*M_PI;
    c.blade.color              = Color(0.3,0,0.1);
    c.blade.length             = 1.100; //dm
    c.blade.wheels             = 4;
    c.blade.width              = 0.030; //dm
    c.blade.height             = 0.150; //dm
    c.screwbase.color          = Color(0.3,0,0.1);
    c.screwbase.radius         = 0.450; //dm
    c.screwbase.width          = 0.300; //dm
    c.screwbase.mass           = 1.0; //kg
    c.screwbase.maxSpeed       = 4.8;
    c.screwbase.maxForce       = 200;
    c.screwbase.blades         = 8;
    c.wheel.axisVisible        = true;
    c.wheel.axisSize           = 0.050; //dm
    c.wheel.color              = Color(0.8,0.8,0.8);
    c.wheel.mass               = 0.010; //kg
    c.wheel.posradius          = 1.120; //dm
    c.wheel.possegment         = 0.3*M_PI;
    c.wheel.posshift           = -0.5;
    c.wheel.radius             = 0.125; //dm
    c.wheel.width              = 0.050; //dm
    return c;
  }

  Inspectable::iparamkeylist Nejihebi::getInternalParamNames() const {
    iparamkeylist list = Inspectable::getInternalParamNames();
    for (int i=0; i<conf.numberOfScrews; i++) {
      list.push_back("screw_" + std::itos(i) + "_angle");
      list.push_back("screw_" + std::itos(i) + "_speed");
    }
    for (int i=0; i<conf.numberOfScrews-1; i++) {
      list.push_back("yaw_" + std::itos(i) + "_angle");
      list.push_back("yaw_" + std::itos(i) + "_speed");
      list.push_back("yaw_" + std::itos(i) + "_power");
      list.push_back("pitch_" + std::itos(i) + "_angle");
      list.push_back("pitch_" + std::itos(i) + "_speed");
      list.push_back("pitch_" + std::itos(i) + "_power");
    }
    return list;
  }

  Inspectable::iparamvallist Nejihebi::getInternalParams() const {
    iparamvallist list = Inspectable::getInternalParams();
    for (int i=0; i<conf.numberOfScrews; i++) {
      list.push_back(screws[i].joint->getPosition1());
      list.push_back(screws[i].joint->getPosition1Rate());
    }
    for (int i=0; i<conf.numberOfScrews-1; i++) {
      list.push_back(servos[i]->getJoint()->getPosition1());
      list.push_back(servos[i]->getJoint()->getPosition1Rate());
      list.push_back(servos[i]->getPower1());
      list.push_back(servos[i]->getJoint()->getPosition2());
      list.push_back(servos[i]->getJoint()->getPosition2Rate());
      list.push_back(servos[i]->getPower2());
    }
    return list;
  }

  Primitive* Nejihebi::getMainPrimitive() const {
    return head;
  }

  int Nejihebi::getMotorNumberIntern() {
    return 5*conf.numberOfScrews-4;
  }

  int Nejihebi::getSensorNumberIntern() {
    return 5*conf.numberOfScrews-4;
  }

  int Nejihebi::getSensorsIntern(sensor* sensors, int sensornumber) {
    assert(sensornumber == 5*conf.numberOfScrews-4);
    const unsigned int noScrews = screws.size();
    const unsigned int noServos = servos.size();
    for (unsigned int i=0; i<noScrews; i++) {
      sensors[i] = screws[i].joint->getPosition1()/M_PI;
    }
    for (unsigned int i=0; i<noServos; i++) {
      sensors[noScrews+4*i]   = servos[i]->getJoint()->getPosition1()/M_PI;
      sensors[noScrews+4*i+1] =
          servos[i]->getJoint()->getPosition1Rate()/conf.jointUnit.maxVel;
      sensors[noScrews+4*i+2] = servos[i]->getJoint()->getPosition2()/M_PI;
      sensors[noScrews+4*i+3] =
          servos[i]->getJoint()->getPosition2Rate()/conf.jointUnit.maxVel;
    }
    return noScrews+4*noServos;
  }

  void Nejihebi::nameSensor(const int sensorNo, const std::string& name) {
    addInspectableDescription("x["+std::itos(sensorNo)+"]", "sensor: "+name);
  }

  void Nejihebi::nameMotor(const int motorNo, const std::string& name) {
    addInspectableDescription("y["+std::itos(motorNo)+"]", "motor: "+name);
  }

  void Nejihebi::placeIntern(const osg::Matrix& pose) {
    create(pose);
  }

  void Nejihebi::setMotorsIntern(const double* motors, int motornumber) {
    assert(motornumber == 5*conf.numberOfScrews-4);
    // for each motor the motorcommand (between -1 and 1) multiplied with max
    // speed is set and the maximal force to realize this command are set
    const unsigned int noScrews = screws.size();
    const unsigned int noServos = servos.size();
    unsigned int len = std::min(unsigned(motornumber), noScrews);
    for (unsigned int i=0; i<len; i++){
      screws[i].joint->setParam(dParamVel, clip(motors[i],-1.0,1.0)*conf.screwbase.maxSpeed);
      screws[i].joint->setParam(dParamFMax, conf.screwbase.maxForce);
    }

    // temporary array for servo commands
    std::vector<double> commands(4*noServos, 0);
    len = std::min(unsigned(motornumber), noScrews+4*noServos);
    for(unsigned int i=noScrews; i<len; i++)
      commands[i-noScrews] = motors[i];

    // use array to set motors
    for (unsigned int i=0; i<noServos; i++) {
      servos[i]->setPower(
          conf.jointUnit.yaw.power   * clip(motors[noScrews+2*noServos+2*i],-1.0,1.0),
          conf.jointUnit.pitch.power * clip(motors[noScrews+2*noServos+2*i+1],-1.0,1.0));
      servos[i]->set( clip(motors[noScrews+2*i],-1.0,1.0)*M_PI,
          clip(motors[noScrews+2*i+1],-1.0,1.0)*M_PI );
    }
  }

}



