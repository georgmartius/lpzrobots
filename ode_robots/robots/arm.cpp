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

#include <iostream>
#include <assert.h>

#include "mathutils.h"
#include "arm.h"
#include "oneaxisservo.h"
#include "sensor.h"


#include <selforg/controller_misc.h>
#include <selforg/matrix.h>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>

using namespace std;
using namespace matrix;

namespace lpzrobots{



  //    OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(conf)
  //    OdeRobot(odeHandle, osgHandle), conf(conf)
  Arm::Arm(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
           const ArmConf& conf, const std::string& name):
    OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(conf)
  {
    // robot not created up to now
    created=false;

    parentspace=odeHandle.space;

    factorSensors=1/2.7; // normalization of position w.r.t. arm length (only for endeffector position)
    addParameter("motorPower", &this->conf.motorPower, 0,50);
    addParameterDef("factorSensors", &factorSensors, 1.0/2.7,0,10);
    addParameter("damping", &this->conf.damping, 0,1 );
    addParameterDef("print", &print, 3, 0,10 );


    int s=0;
    FOREACHC(list<Sensor*>, conf.sensors, i){
      s += (*i)->getSensorNumber();
    }
    if(conf.withContext){
      sensorno=7 + s; // (endeff pos + dummy | joint angles) + context sensors
    }else{
      sensorno=4 + s; // (endeff pos + dummy | joint angles hingeServo values (shoulder: 3, elbow: 1))
    }

    motorno=4; // dito


    // standard objects color: white
    // color definition: rgb
    // (1,0,0) = red
    // (1,1,0) = yellow
    // (0,1,0) = green
    // (0,0,1) = blue
    // (1,1,1) = white
    this->osgHandle.color = Color(1.1,1,1.4);

    endeff.set(3,1);

  };

  /**
   * sets the pose of the vehicle
   * @param pose desired 4x4 pose matrix
   */
  void Arm::placeIntern(const osg::Matrix& pose)
  {
    // the position of the robot is the center of the body
    // to set the arm on the ground when the z component of the position is 0
    // body_height/2 is added
    osg::Matrix p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.body_height/2));
    create(p2);
  };

  /**
   * draws the arm
   */
  void Arm::update()
  {
    assert(created); // robot must exist
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++)
      {
        if(*i) (*i)->update();
      }
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++)
      {
        if(*i) (*i)->update();
      }
    osg::Vec3 pos = objects[hand]->getPosition();
    endeff.val(0,0)=pos[0];
    endeff.val(1,0)=pos[1];
    endeff.val(2,0)=pos[2];
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Arm::getSensorsIntern(sensor* sensors, int sensornumber)
  {
    assert(created); // robot must exist
    unsigned int len=0;

    if(conf.useJointSensors){
      // get the hingeServos
      for(len=0; (len<hingeServos.size()); len++) {
          sensors[len] = hingeServos[len]->get();
      }
    }else{
      // get endeffector position ("watching" arm)
      // factorSensors (total arm length) used to normalize position to [-1,1]
      osg::Vec3 pos = objects[hand]->getPosition();
      sensors[len++]=pos[0];
      sensors[len++]=pos[1];
      sensors[len++]=pos[2];
      sensors[len++]=pos[2];
      // TODO scaleShoulderCentered(sensors);
    }
    if(conf.withContext){
      osg::Vec3 pos = objects[hand]->getPosition();
      sensors[len++]=pos[0];
      sensors[len++]=pos[1];
      sensors[len++]=pos[2];
    }

    FOREACH(list<Sensor*>, conf.sensors, i){
        len += (*i)->get(sensors+len, sensornumber-len);
    }

    //                printf("sensors: ");
    //    for(unsigned int n=0; n<3; n++)
    //                {
    //            printf("%f ", sensors[n]);
    //                }
    //                printf("\n");

    return len;
  };

  /** sets actual motorcommands
   *  @param motors motors scaled to [-1,1]
   *  @param motornumber length of the motor array
   */
  void Arm::setMotorsIntern(const double* motors, int motornumber)
  {
    assert(created); // robot must exist
    unsigned int len = min(motornumber, getMotorNumberIntern());// /2;  // <- 2FELHAFT
    // controller output as torques
    for(unsigned int i=0; (i<len) && (i<hingeServos.size()); i++)
      {
        hingeServos[i]->set(/*0.5**/motors[i]);
      }
    //                printf("motors: ");
    //                for(unsigned int i=0; (i<len) && (i<hingeServos.size()); i++)
    //                {
    //                        printf("%f ", motors[i]);
    //                }
    //                printf("\n");
  };

  /** returns a vector with the positions of all segments of the robot
   *  @param poslist vector of positions (of all robot segments)
   *  @return length of the list
   */
  int Arm::getSegmentsPosition(std::vector<Position> &poslist)
  {
    assert(created);
    for (int i=0; i<(int)objects.size(); i++)
      {
        poslist.push_back(Position(dBodyGetPosition(objects[i]->getBody())));
        // Pos p(objects[i]->getPosition());
        // poslist.push_back(p.toPosition());
      }
    return (int)objects.size();
  };

  void Arm::getEndeffectorPosition(double* position)
  {
    osg::Vec3 pos = objects[hand]->getPosition();
    position[0]=pos[0];
    position[1]=pos[1];
    position[2]=pos[2];
  }

  void Arm::doInternalStuff(GlobalData& globalData)
  {
  }

  /**
   * creates arm at desired position
   */
  void Arm::create(const osg::Matrix& pose)
  {
    if (created)
      {
        destroy();
      }

    // create vehicle space & add to parentspace
    // robot will be inserted in the vehicle space
    odeHandle.createNewSimpleSpace(parentspace,false);

    // ======= OBJECTS AND JOINTS AND MOTORS ========

    // === BODY CREATION ==============
    // create body, initialize, set position and add to list of objects
    Primitive* _base = new Box(conf.body_width, conf.body_depth, conf.body_height);
    //_base->getOSGPrimitive()->setTexture("Images/wood.rgb");
    _base->init(odeHandle, conf.body_mass, osgHandle);
    _base->setPose(pose);
    objects.push_back(_base);

    // position of shoulder joint part one
    osg::Matrix _pose = osg::Matrix::translate((conf.body_width/2)+(conf.shoulder_radius)+(2*conf.joint_offset),
                                               0,(conf.body_height/2)-(conf.shoulder_radius)) * pose;

    // === SHOULDER ==========
    // shoulder joint is devided in two objects
    // to be linked by one axis joints to have influence on constraints for each DOF

    // creating procedure (see above) shoulder part one
    Primitive* _shoulder1 = new Sphere(conf.shoulder_radius);
    _shoulder1->init(odeHandle, conf.shoulder_mass, osgHandle);
    _shoulder1->setPose(_pose);
    objects.push_back(_shoulder1);

    // === one axis joint for shoulder elevation =====
    // === Biess, Flash (2006): THETA ================
    // creation procedure (see above) 1st DOF @ shoulder
    HingeJoint* HJ_elev = new HingeJoint(objects[base], objects[shoulder1],
                                         Pos(0,0,0)*_pose,
                                         Axis(1, 0, 0)*_pose);
    HJ_elev->init(odeHandle, osgHandle, true);
    HJ_elev->setParam(dParamLoStop, conf.elevation_min);
    HJ_elev->setParam(dParamHiStop, conf.elevation_max);
    joints.push_back(HJ_elev);
    // ANMERKUNG: conf.elevation_min beeinflusst OBEREN Armanschlag?
    // ANMERKUNG 2: min und max skalieren die GESCHWINDIGKEIT der Bewegung! "travel bounds"? :-(
    //    oneaxisservo.h: /** min and max values are understood as travel bounds. Min should be less than 0.*/
    // servo motor for joint
    HingeServo* elev_servo = new HingeServo(HJ_elev, conf.servoFactor*conf.elevation_min, conf.servoFactor*conf.elevation_max, conf.motorPower, conf.damping, 0);
    hingeServos.push_back(elev_servo);

    // position of shoulder joint part 2
    _pose = osg::Matrix::translate( (2*conf.shoulder_radius)+(2*conf.joint_offset), 0, 0) * _pose;
    // creating procedure (see above) shoulder part two
    Primitive* _shoulder2 = new Sphere(conf.shoulder_radius);
    _shoulder2->init(odeHandle, conf.shoulder_mass, osgHandle);
    _shoulder2->setPose(_pose);
    objects.push_back(_shoulder2);

    // === one axis joint for shoulder azimuthal angle =====
    // === Biess, Flash (2006): ETA ================
    // creation procedure (see above) 2nd DOF @ shoulder
    HingeJoint* HJ_azimut = new HingeJoint(objects[shoulder1], objects[shoulder2],
                                           Pos(-(conf.shoulder_radius + conf.joint_offset),0,0)*_pose,
                                           Axis(0, 0, 1)*_pose); // rotated roation axis (y-axis of shoulder centered coordinate system)
    // because of lifting the arm M_PI/2 in the beginning
    HJ_azimut->init(odeHandle, osgHandle, true);
    HJ_azimut->setParam(dParamLoStop, conf.azimuthal_min);
    HJ_azimut->setParam(dParamHiStop, conf.azimuthal_max);
    joints.push_back(HJ_azimut);
    // servo motor for joint
    HingeServo* azimut_servo = new HingeServo(HJ_azimut, conf.servoFactor*conf.azimuthal_min, conf.servoFactor*conf.azimuthal_max, conf.motorPower/2, 2*conf.damping, 0);
    hingeServos.push_back(azimut_servo);


    // === UPPER ARM =========
    // position of upper arm
    //  initial turn of the upper arm is M_PI/4;
    osg::Matrix shoulder2_pose = osg::Matrix::rotate(M_PI/4, 0, 1, 0) * _pose;
    _pose = osg::Matrix::translate( 0,0, -(conf.upperarm_length/2))*
      osg::Matrix::rotate(M_PI/2, 1, 0, 0) * // initial elevation of upper arm of M_PI/2
      osg::Matrix::translate( (conf.upperarm_radius)+(conf.shoulder_radius)+(2*conf.joint_offset), 0,0)
      * shoulder2_pose;
    // creating procedure (see above) upper arm
    Primitive* _upperArm = new Capsule(conf.upperarm_radius, conf.upperarm_length);
    _upperArm->init(odeHandle, conf.upperarm_mass, osgHandle);
    _upperArm->setPose(_pose);
    objects.push_back(_upperArm);

    // avoid collision of upper arm with body
    odeHandle.addIgnoredPair(objects[base],objects[upperArm]);

    //                // marking of x-axis (shoulder centered coordinate system)
    //                Primitive* _xaxis_prim = new Sphere(0.05);
    //                // glue sphere to upper arm via transform object (at given position - relative position w.r.t. upper arm)
    //                Primitive* _xaxis_trans = new Transform(objects[upperArm], _xaxis_prim,
    //                        osg::Matrix::translate(conf.upperarm_radius, 0, conf.upperarm_length/2));
    //                // initialize transform object with zero mass
    //                _xaxis_trans->init(odeHandle, 0, osgHandle);
    //                // marking of y-axis (shoulder centered coordinate system)
    //                Primitive* _yaxis_prim = new Sphere(0.03);
    //                Primitive* _yaxis_trans = new Transform(objects[upperArm], _yaxis_prim,
    //                        osg::Matrix::translate(0, conf.upperarm_radius, conf.upperarm_length/2));
    //                _yaxis_trans->init(odeHandle, 0, osgHandle);

    // === one axis joint for shoulder humeral angle =====
    // === Biess, Flash (2006): XSI ================
    // creation procedure (see above) 3rd DOF @ shoulder
    HingeJoint* HJ_humer = new HingeJoint(objects[shoulder2], objects[upperArm],
                                          Pos(conf.shoulder_radius + conf.joint_offset,0,0)*shoulder2_pose,
                                          Axis(0, -1, 0)*shoulder2_pose); // rotated rotation axis (z-axis of shoulder centered coordinate system)
    // because of lifting the arm M_PI/2 in the beginning
    HJ_humer->init(odeHandle, osgHandle, true);
    HJ_humer->setParam(dParamLoStop, conf.humeral_min);
    HJ_humer->setParam(dParamHiStop, conf.humeral_max);
    joints.push_back(HJ_humer);
    // servo motor for joint
    HingeServo* humer_servo = new HingeServo(HJ_humer, conf.servoFactor*conf.humeral_min, conf.servoFactor*conf.humeral_max, conf.motorPower/4, conf.damping, 0, 30/*maxVel*/);
    hingeServos.push_back(humer_servo);


    // === FOREARM =========
    /* move reference point to elbow */
    osg::Matrix elbow_pose= osg::Matrix::translate(0, 0, -conf.upperarm_length/2) *
      _pose;
    /* coordinate transform for forarm */
    _pose = osg::Matrix::translate(0,0,-conf.forearm_length/2)* /* move forarm so that 0,0,0 is at lower end */
      osg::Matrix::rotate(M_PI/3, 0, 1, 0)*  // initial flexion of elbow joint of M_PI/3
      elbow_pose;
    // creating procedure (see above) forearm
    Primitive* _foreArm = new Capsule(conf.forearm_radius, conf.forearm_length);
    _foreArm->init(odeHandle, conf.forearm_mass, osgHandle);
    _foreArm->setPose(_pose);
    objects.push_back(_foreArm);

   // === one axis joint for Elbow =======
    // === Biess, Flash (2006): PHI =======
    // create joint between upper arm and forearm, initialize, set constraints, add to list of joints
    HingeJoint* HJ_elbow = new HingeJoint(objects[upperArm], objects[foreArm],
                                          Pos(0,0,0)*elbow_pose,
                                          //osg::Vec3(pos[0], pos[1]+conf.upperarm_length/2+conf.joint_offset, pos[2]) /* anchor of joint */,
                                          Axis(0, 1, 0)* elbow_pose); // rotation axis: y-axis of shoulder centered coordinate system
    // = z-axis of world coordinate system, because of initial rotation of shoulder joint (M_PI/2 round y-axis)
    HJ_elbow->init(odeHandle, osgHandle, true);
    HJ_elbow->setParam(dParamLoStop, conf.elbow_min);
    HJ_elbow->setParam(dParamHiStop, conf.elbow_max);
    joints.push_back(HJ_elbow);
    // create servo motor for elbow joint, add to list of motors
    // min und max beeinlussen Geschwindigkeit?!
    HingeServo* elbow_servo = new HingeServo(HJ_elbow, conf.servoFactor*conf.elbow_min, conf.servoFactor*conf.elbow_max,
                                             conf.scaleMotorElbow * conf.motorPower, conf.damping, 0, 30); // P D I(0) MaxVel
    hingeServos.push_back(elbow_servo);


    // === HAND =========
    // position of hand
    _pose = osg::Matrix::translate(0, 0, -conf.forearm_length/2 ) * _pose;
    // creating procedure (see above) hand
    Primitive* _hand = new Sphere(1.3*conf.forearm_radius);
    _hand->init(odeHandle, 0.005 /*almost weightless*/, osgHandle);
    _hand->setPose(_pose);
    objects.push_back(_hand);

    // === fixed wrist ===========
    FixedJoint* FJ_hand = new FixedJoint(objects[foreArm], objects[hand]);
    FJ_hand->init(odeHandle, osgHandle);
    joints.push_back(FJ_hand);


    printf("size: %ld objects, %ld joints, %ld hingeservos\n", (long int)objects.size(),
           (long int)joints.size(), (long int)hingeServos.size());

    FOREACH(list<Sensor*>, conf.sensors, i){
        (*i)->init(objects[hand]);
    }


    created=true;
  };

  /**
   * destroys vehicle and space
   */
  void Arm::destroy()
  {
    if (created){
      for(list<Sensor*>::iterator i = conf.sensors.begin(); i != conf.sensors.end(); i++){
          if(*i) delete *i;
      }
      conf.sensors.clear();

      for (vector<HingeServo*>::iterator i = hingeServos.begin(); i!= hingeServos.end(); i++)
        {
          if(*i) delete *i;
        }
      hingeServos.clear();
      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++)
        {
          if(*i) delete *i;
        }
      joints.clear();
      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++)
        {
          if(*i) delete *i;
        }
      objects.clear();

      odeHandle.deleteSpace();
    }
    created=false;
  };

  void Arm::notifyOnChange(const paramkey& key)
  {
    if(key == "motorPower") {
      double val = conf.motorPower;
      hingeServos[0]->setPower(val);   // elevation
      hingeServos[1]->setPower(val/2); // azimutal
      hingeServos[2]->setPower(val/4); // humeral
      hingeServos[3]->setPower(val*conf.scaleMotorElbow);
      //       FOREACH (vector<HingeServo*>, hingeServos, i) {
      //         (*i)->setPower(val);
      //       }
    } else if(key == "damping") {
      FOREACH (vector<HingeServo*>, hingeServos, i) {
        (*i)->setDamping(conf.damping);
      }
    }
  };

//  list<Inspectable::iparamkey> Arm::getInternalParamNames() const
//  {
//    list<Inspectable::iparamkey> keylist;
////    keylist+=storeMatrixFieldNames(endeff,"endeff");
//    //        printf("gotInternalParamNames\n");
//    return keylist;
//  }
//
//  list<Inspectable::iparamval> Arm::getInternalParams() const
//  {
//    list<Inspectable::iparamval> l;
////    l+=endeff.convertToList();
//    //        printf("gotInternalParams\n");
//    return l;
//  }

  //list<Inspectable::ILayer> Arm::getStructuralLayers() const
  //{
  //        list<Inspectable::ILayer> l;
  //        return l;
  //}
  //
  //list<Inspectable::IConnection> Arm::getStructuralConnections() const
  //{
  //        list<Inspectable::IConnection> l;
  //        return l;
  //}

}
