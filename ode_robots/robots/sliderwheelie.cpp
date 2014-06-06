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

#include "sliderwheelie.h"
using namespace std;

namespace lpzrobots {

  SliderWheelie::SliderWheelie(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                               const SliderWheelieConf& conf,
                               const std::string& name, const std::string& revision)
    : OdeRobot(odeHandle, osgHandle, name, (revision.empty()? "$Id$" : revision))
    , conf(conf)
  {
    created=false;
    center=0;
    dummycenter=0;
    addParameter("frictionground", &this->conf.frictionGround,0,2);
    addParameter("powerratio"    , &this->conf.powerRatio,0,20);
    addParameter("motorpower"    , &this->conf.motorPower,0,100);
    addParameter("motordamp"     , &this->conf.motorDamp,0,1);
    addParameter("sensorfactor"  , &this->conf.sensorFactor,0,10);
  }

  SliderWheelie::~SliderWheelie() {
    if(created) destroy();
  }


  void SliderWheelie::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
    create(pose * osg::Matrix::translate(osg::Vec3(0, 0, 0.5*conf.segmLength*conf.segmNumber/M_PI
                                                   + conf.segmDia/2)));
  }

  void SliderWheelie::update() {
    OdeRobot::update();
    assert(created); // robot must exist
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }
    if(center) center->update();
  }

  void SliderWheelie::doInternalStuff(GlobalData& global){
  }

  void SliderWheelie::setMotorsIntern(const double* motors, int motornumber) {
   assert(created);
   unsigned int len = min(motornumber, getMotorNumberIntern());
   unsigned int n=0;
   // controller output as torques
   if(conf.motorType != SliderWheelieConf::AngularMotor){
     for(unsigned int i=0; (n<len) && (i<hingeServos.size()); i++, n++) {
       hingeServos[i]->set(motors[n]);
     }
   }else{
     for(unsigned int i=0; (n<len) && (i<angularMotors.size()); i++, n++) {
       angularMotors[i]->set(1, motors[n]);
     }
   }

   for(unsigned int i=0; (n<len) && (i<sliderServos.size()); i++, n++) {
     sliderServos[i]->set(motors[n]);
   }

   /// update center position
   if(center){
     Pos p;
     Pos v;
     for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
       p += (*i)->getPosition();
       v += (*i)->getVel();
     }
     p /= objects.size();
     v /= objects.size();
     center->setPosition(p);
     dummycenter->setPosition(p);
     dummycenter->setVel(v);
   }
  }

  int SliderWheelie::getSensorsIntern(sensor* sensors, int sensornumber) {
   assert(created);
   unsigned int len=min(sensornumber, getSensorNumberIntern());
   unsigned int n=0;
   // get the hingeServos
   if(conf.motorType != SliderWheelieConf::AngularMotor){
     for(unsigned int i=0; (n<len) && (i<hingeServos.size()); i++, n++) {
       sensors[n] = hingeServos[i]->get();
     }
   }else{
     for(unsigned int i=0; (n<len) && (i<angularMotors.size()); i++, n++) {
       sensors[n] = angularMotors[i]->get(1);
     }
   }

   for(unsigned int i=0; (n<len) && (i<sliderServos.size()); i++,n++) {
     sensors[n] = sliderServos[i]->get();
   }
   return n;
  }


  void SliderWheelie::create(const osg::Matrix& pose) {
    if (created) {
      destroy();
    }

    odeHandle.createNewSimpleSpace(parentspace,false);
    //    odeHandle.substance.toRubber(10);

    if(conf.jointLimitOut<0){
      conf.jointLimitOut = 2*M_PI/conf.segmNumber;
    }
    vector<Pos> ancors;
    // angular positioning
    for(int n = 0; n < conf.segmNumber; n++) {
      osg::Matrix m = osg::Matrix::rotate(M_PI*2*n/conf.segmNumber, 0, -1, 0) * pose;
      if(n%2==0 && conf.sliderLength > 0){ // slider segment

        Primitive* p1 = new Box(conf.segmDia/2, conf.segmDia*4*2, conf.segmLength/2);
        if(!conf.texture.empty()){
          p1->setTexture(conf.texture);
        }
        // p1->setTexture("Images/wood.rgb");
        p1->init(odeHandle, conf.segmMass/2 , osgHandle);
        p1->setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, 0) *
                    osg::Matrix::translate(-conf.segmLength/4,0,
                                           -0.5*conf.segmLength*conf.segmNumber/M_PI) * m );
        objects.push_back(p1);

        Primitive* p2 = new Box(conf.segmDia/2, conf.segmDia*4*2, conf.segmLength/2);
        if(!conf.texture.empty()){
          p2->setTexture(conf.texture);
        }
        // p2->setTexture("Images/dusty.rgb");
        p2->init(odeHandle, conf.segmMass/2 , osgHandle);
        p2->setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, 0) *
                    osg::Matrix::translate(conf.segmLength/4,0,
                                           -0.5*conf.segmLength*conf.segmNumber/M_PI) * m );
        objects.push_back(p2);

        const Pos& pos1(p1->getPosition());
        const Pos& pos2(p2->getPosition());

        SliderJoint* j = new SliderJoint(p1, p2, (pos1 + pos2)/2,
                                         Axis(1,0,0)*m);
        j->init(odeHandle, osgHandle, true, conf.segmDia);

        joints.push_back(j);

        SliderServo* servo = new SliderServo(j,
                                             -conf.segmLength*conf.sliderLength/2,
                                             conf.segmLength*conf.sliderLength/2,
                                             conf.motorPower*conf.powerRatio, conf.motorDamp);
        sliderServos.push_back(servo);
      }else{ // normal segment
        Primitive* p1 = new Box(conf.segmDia/2,
                                /* conf.segmDia*4*( (n+1)%4 ==0 ? 3 : (n%2 ==0 ? 2 : 1)), */
                                conf.segmDia*8,
                                conf.segmLength-conf.segmDia/2);
        if(!conf.texture.empty()){
          p1->setTexture(conf.texture);
        }
        p1->init(odeHandle, conf.segmMass * ( (n+1)%4 ==0 ? 1.0 : 1), osgHandle);
        p1->setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, 0) *
                    osg::Matrix::translate(0,0,-0.5*conf.segmLength*conf.segmNumber/M_PI) * m );
        objects.push_back(p1);
      }
      ancors.push_back(Pos(conf.segmLength/2,0,-0.5*conf.segmLength*conf.segmNumber/M_PI) * m);
     }


   //***************** hinge joint definition***********
    int i = 0;
    for(int n=0; n < conf.segmNumber ; n++, i++) {
      if(n%2==0 && conf.sliderLength > 0){
        i++;
      }
      int o1 = i;
      int o2 = (i+1) % objects.size();
      HingeJoint* j = new HingeJoint(objects[o1], objects[o2],
                                     ancors[n],
                                     Axis(0,1,0)*pose);
      Color c;
      if(n==0) c = Color(0.85,0.88,0.88);
      //      else if(n == conf.segmNumber/2) c = Color(0.6,0.56,0);
      else c = osgHandle.color;
      j->init(odeHandle, osgHandle.changeColor(c), true, conf.segmDia*4);
      joints.push_back(j);

      HingeServo* servo;
      switch (conf.motorType){
      case SliderWheelieConf::Servo:
        servo = new HingeServo(j, -conf.jointLimitOut,
                                           conf.jointLimitIn,
                                           conf.motorPower, conf.motorDamp,0);
        hingeServos.push_back(servo);
        break;
      case SliderWheelieConf::CenteredServo:
        servo = new OneAxisServoCentered(j, -conf.jointLimitOut, conf.jointLimitIn,
                                         conf.motorPower, conf.motorDamp,0);
        hingeServos.push_back(servo);
        break;
      case SliderWheelieConf::AngularMotor:
        AngularMotor* amotor = new AngularMotor1Axis(odeHandle, j, conf.motorPower);
        j->setParam(dParamLoStop, -conf.jointLimitOut);
        j->setParam(dParamHiStop, conf.jointLimitIn);
        angularMotors.push_back(amotor);
        break;
      }
    }

    // create virtual center
    center = new Sphere(0.1);
    OdeHandle centerHandle = odeHandle;
    centerHandle.substance.toNoContact();
    center->init(centerHandle, 0, osgHandle.changeAlpha(0.4),
                 conf.showCenter ? Primitive::Geom | Primitive::Draw : Primitive::Geom);
    center->setPose(osg::Matrix::translate(0,0,0) * pose);
    dummycenter = new DummyPrimitive();
    dummycenter->setPosition(center->getPosition());

    notifyOnChange("frictionground");

    created=true;

  }

  /** destroys vehicle and space
   */
  void SliderWheelie::destroy() {
    if(created) {
      FOREACH(vector<AngularMotor*>, angularMotors, i){
        if(*i) delete *i;
      }
      angularMotors.clear();

      FOREACH(vector<HingeServo*>, hingeServos, i){
        if(*i) delete *i;
      }
      hingeServos.clear();

      FOREACH(vector<SliderServo*>, sliderServos, i){
        if(*i) delete *i;
      }
      sliderServos.clear();

      cleanup();

      if(center) delete center;
      if(dummycenter) delete dummycenter;
      odeHandle.deleteSpace();
   }
  }

  void SliderWheelie::notifyOnChange(const paramkey& key){
    if(key == "frictionground") {
      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++) {
        if(*i) (*i)->substance.roughness=conf.frictionGround;
      }
    } else if(key == "motorpower") {
      FOREACH(vector<HingeServo*>, hingeServos, i) {
        if(*i) (*i)->setPower(conf.motorPower);
      }
      FOREACH(vector<AngularMotor*>, angularMotors, i) {
        if(*i) (*i)->setPower(conf.motorPower);
      }
    } else if(key == "motordamp") {
      FOREACH(vector<HingeServo*>, hingeServos, i) {
        if(*i) (*i)->setDamping(conf.motorDamp);
      }
      FOREACH(vector<SliderServo*>, sliderServos, i) {
        if(*i) (*i)->setDamping(conf.motorDamp);
      }
    } else if(key == "powerratio") {
      for(vector<SliderServo*>::iterator i=sliderServos.begin(); i!=sliderServos.end(); i++) {
        if(*i) (*i)->setPower(conf.motorPower * conf.powerRatio);
      }
    }
  }


}
