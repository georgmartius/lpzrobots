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
#include <ode_robots/primitive.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>

// include header file
#include "vierbeiner.h"

using namespace osg;
using namespace std;

namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  VierBeiner::VierBeiner(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
           const VierBeinerConf& c, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "0.7"), conf(c)
  {
    // robot is not created till now
    created=false;

    // choose color here a pastel white is used
    this->osgHandle.color = Color(1.0, 1,1,1);
    //    this->osgHandle.color = Color(1.0, 156/255.0, 156/255.0, 1.0f);
    //    addParameter("elast", &conf.elasticity);

    legmass=conf.mass * conf.relLegmass / conf.legNumber;    // mass of each legs

    addParameter("powerfactor",    &conf.powerFactor,0,10);
    addParameter("dampingfactor",  &conf.dampingFactor,0,10);
    addParameter("hippower",       &conf.hipPower,0,100);
    addParameter("hipdamping",     &conf.hipDamping,0,1);
    addParameter("kneepower",      &conf.kneePower,0,100);
    addParameter("kneedamping",    &conf.kneeDamping,0,1);
    addParameter("anklepower",     &conf.anklePower,0,100);
    addParameter("ankledamping",   &conf.ankleDamping,0,1);
    addParameter("hipjointlimit",  &conf.hipJointLimit,-M_PI,M_PI);
    addParameter("kneejointlimit", &conf.kneeJointLimit,-M_PI,M_PI);
  };


  int VierBeiner::getMotorNumberIntern(){
    return headtailservos.size() + hipservos.size() + kneeservos.size() + ankleservos.size();
  };

  /* sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void VierBeiner::setMotorsIntern(const double* motors, int motornumber){
    assert(created); // robot must exist

    // controller output as torques
    int n=0;
    FOREACH(vector <HingeServo*>, headtailservos, s){
      (*s)->set(motors[n]);
      n++;
    }
    FOREACH(vector <HingeServo*>, hipservos, s){
      (*s)->set(motors[n]);
      n++;
    }
    FOREACH(vector <HingeServo*>, kneeservos, s){
      (*s)->set(motors[n]);
      n++;
    }
    FOREACH(vector <HingeServo*>, ankleservos, s){
      (*s)->set(motors[n]);
      n++;
    }
    assert(min(motornumber, getMotorNumberIntern())==n);
    /// set knee servos to set point 0 (spring emulation)
//     FOREACH(vector <HingeServo*>, kneeservos, s){
//       (*s)->set(0);
//     }
    /// set ankle servos to set point 0 (spring emulation)
    //    FOREACH(vector <HingeServo*>, ankleservos, s){
    //      (*s)->set(0);
    //    }
  };

  int VierBeiner::getSensorNumberIntern(){
    return headtailservos.size() + hipservos.size() + kneeservos.size() + ankleservos.size();
  };

  /* returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int VierBeiner::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created);
    int n=0;
    FOREACHC(vector <HingeServo*>, headtailservos, s){
      sensors[n]   = (*s)->get();
      n++;
    }
    FOREACHC(vector <HingeServo*>, hipservos, s){
      sensors[n]   = (*s)->get();
      n++;
    }
    FOREACHC(vector <HingeServo*>, kneeservos, s){
      sensors[n]   = (*s)->get();
      n++;
    }
    FOREACHC(vector <HingeServo*>, ankleservos, s){
      sensors[n]   = (*s)->get();
      n++;
    }
    assert(min(sensornumber, getSensorNumberIntern())==n);
    return n;
  };


  void VierBeiner::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    //    Matrix p2;
    //    p2 = pose * TRANSM(Vec3(0, 0, conf.legLength + conf.legLength/8));
    create(pose);
  };


  /**
   * updates the osg notes
   */
  void VierBeiner::update() {
    OdeRobot::update();
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
  void VierBeiner::doInternalStuff(GlobalData& global){
  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void VierBeiner::create( const osg::Matrix& pose ){
    if (created) {
      destroy();
    }

    odeHandle.createNewSimpleSpace(parentspace,false);
    OsgHandle osgHandleJ = osgHandle.changeColor(Color(1.0,0.0,0.0));
    HingeJoint* j;
    HingeServo* servo;

    // create body
    double twidth = conf.size / 1.5;
    double theight = conf.size / 4;
    if(conf.hippo){
      trunk = new Sphere(conf.size/2);
      //      trunk = new Capsule(conf.size/2.5,conf.size/5.0);
    }else{
      trunk = new Box(conf.size, twidth, theight);
    }
    trunk->setTexture("Images/toy_fur3.jpg");
    trunk->init(odeHandle, conf.mass*0.8, osgHandle);
    if(conf.hippo){
      trunk->setPose(TRANSM(0,0,conf.legLength*1.1)*pose);
    }else{
      trunk->setPose(TRANSM(0,0,conf.legLength)*pose);
    }
    objects.push_back(trunk);
    if(conf.useBigBox){
      // the pole is a non-visible box which hinders the dog from falling over.
      Primitive* pole;
      double poleheight=conf.size*2;
      pole = new Box(conf.size*1.6,twidth*2.5,poleheight);
      bigboxtransform= new Transform(trunk,pole, TRANSM(0,0,theight/2+poleheight/2));
      //bigboxtransform->init(odeHandle, 0, osgHandle.changeAlpha(0.1), Primitive::Geom | Primitive::Draw);
      bigboxtransform->init(odeHandle, 0, osgHandle, Primitive::Geom);
      objects.push_back(bigboxtransform);
    }

    // create head and neck
    double necklength = conf.size/6;
    double neckwidth = theight/2;
    double headmass = conf.mass*0.1;
    double headlength = conf.size/8;
    neck = new Capsule(neckwidth,necklength);
    neck->setTexture("Images/toy_fur3.jpg");
    neck->init(odeHandle, headmass/2, osgHandle);
    Pos neckpos(conf.size/2.05,0,conf.legLength);
    neck->setPose(TRANSM(0,0,necklength/2) *
                  ROTM(M_PI/4,0,1,0) *
                  TRANSM(neckpos)*pose);
    objects.push_back(neck);
    Primitive* head;
    head = new Capsule(neckwidth,theight);
    head->setTexture("Images/fur4.jpg");
    headtrans = new Transform(neck, head, TRANSM(0, 0, -headlength/2)
                          * ROTM(-M_PI/2,0,1,0)
                          * TRANSM(0, 0, necklength));
    headtrans->init(odeHandle, headmass/2, osgHandle);
    objects.push_back(headtrans);
    ///ignore collision between box on top of dog and head and also between head and body
    if(conf.useBigBox){
      odeHandle.addIgnoredPair(bigboxtransform,headtrans);
      odeHandle.addIgnoredPair(bigboxtransform,neck);
    }
    odeHandle.addIgnoredPair(trunk,headtrans);

    //now create a kind of face
    if (conf.drawstupidface) {
    Primitive* mouth;
    mouth = new Cylinder(0.95*neckwidth,0.1*neckwidth);
    mouth_trans = new Transform(neck,mouth, TRANSM(0, 0 ,0)
                                * ROTM(-M_PI,0,1,0)
                                * TRANSM(1.5*headlength,
                                                    0.0*headlength, 1.0*headlength));
    mouth_trans->init(odeHandle, headmass/20, osgHandle);
    mouth->setColor("Lippenstiftrot");
    objects.push_back(mouth_trans);
    if(conf.useBigBox)
      odeHandle.addIgnoredPair(bigboxtransform,mouth_trans);
    Primitive* ear_r;
    ear_r = new Box(0.05*neckwidth,0.9*neckwidth,0.9*neckwidth);
    ear_r->setTexture("Images/fur4.jpg");
    ear_r_trans = new Transform(neck,ear_r, TRANSM(0, headlength/1.3,0)
                          * ROTM(-M_PI/4,M_PI/5,1,0)
                          * TRANSM(-1.0*headlength, 0, 2.5*headlength));
    ear_r_trans->init(odeHandle, headmass/20, osgHandle);
    objects.push_back(ear_r_trans);
    if(conf.useBigBox)
        odeHandle.addIgnoredPair(bigboxtransform,ear_r_trans);
    Primitive* ear_l;
    ear_l = new Box(0.05*neckwidth,0.9*neckwidth,0.9*neckwidth);
    ear_l->setTexture("Images/fur4.jpg");
    ear_l_trans = new Transform(neck,ear_l, TRANSM(0, -headlength/1.3,0)
                          * ROTM(-M_PI/4,-M_PI/5,1,0)
                          * TRANSM(-1.0*headlength, 0, 2.5*headlength));
    ear_l_trans->init(odeHandle, headmass/20, osgHandle);
    objects.push_back(ear_l_trans);
    if(conf.useBigBox)
      odeHandle.addIgnoredPair(bigboxtransform,ear_l_trans);
    // this is actually the left eye
    Primitive* eye_r;
    eye_r = new Capsule(0.2*neckwidth,0.0*neckwidth);
    eye_r->setTexture("Images/eye.rgb");
    eye_r_trans = new Transform(neck,eye_r, ROTM(M_PI,0,0,1) * ROTM(M_PI/4,0,1,0)
                                * TRANSM(0,headlength/1.5,0)
                                * ROTM(-M_PI/2,0,1,0)
                                * TRANSM(0, 0, 2.1*headlength));
    eye_r_trans->init(odeHandle, headmass/20, osgHandle);
    eye_r->setColor("Lilienweiss");
    objects.push_back(eye_r_trans);
    if(conf.useBigBox)
      odeHandle.addIgnoredPair(bigboxtransform,eye_r_trans);
    Primitive* eye_l;
    eye_l = new Capsule(0.2*neckwidth,0.0*neckwidth);
    eye_l->setTexture("Images/eye.rgb");
    eye_l_trans = new Transform(neck,eye_l, ROTM(-M_PI,0,0,1) * ROTM(M_PI/4,0,1,0)
                                * TRANSM(0,-headlength/1.5,0)
                                * ROTM(-M_PI/2,0,1,0)
                                * TRANSM(0, 0, 2.1*headlength));
    eye_l_trans->init(odeHandle, headmass/20, osgHandle);
    eye_l->setColor("Lilienweiss");
    objects.push_back(eye_l_trans);
    if(conf.useBigBox)
      odeHandle.addIgnoredPair(bigboxtransform,eye_l_trans);
    } //this much for the face

    j = new HingeJoint(trunk, neck, neckpos * pose, Axis(0,0,1) * pose);
    j->init(odeHandle, osgHandleJ, true, theight * 1.2);
    joints.push_back(j);
    servo =  new HingeServo(j, -M_PI/4, M_PI/4, headmass/2);
    headtailservos.push_back(servo);

    // create tail
    double taillength = conf.size/3;
    double tailwidth = taillength/10;
    double tailmass  = headmass/3;
    tail = new Capsule(tailwidth,taillength);
    tail->setTexture("Images/fur3.jpg");
    tail->init(odeHandle, headmass/2, osgHandle);
    Pos tailpos(-conf.size/1.96,0,conf.legLength+theight/3);
    tail->setPose(TRANSM(0,0,taillength/2) *
                  ROTM(M_PI/2.2,0,-1,0) *
                  TRANSM(tailpos)*pose);
    objects.push_back(tail);
    j = new HingeJoint(trunk, tail, tailpos * pose, Axis(0,1,0) * pose);
    j->init(odeHandle, osgHandleJ, true, tailwidth * 2.05);
    j->setParam(dParamLoStop, -M_PI/2);
    j->setParam(dParamHiStop,  M_PI/2);
    joints.push_back(j);
    servo =  new HingeServo(j, -M_PI/3, M_PI/3, tailmass*3);
    headtailservos.push_back(servo);
    ///ignore collision between box on top of dog and tail
    if(conf.useBigBox)
      odeHandle.addIgnoredPair(bigboxtransform,tail);

    // legs  (counted from back to front)
    double legdist = conf.size*0.9 / (conf.legNumber/2-1);
    for ( int n = 0; n < conf.legNumber; n++ ) {
      double l1 =       n<2 ? conf.legLength*0.45 : conf.legLength*0.5;
      double t1       = conf.legLength/10;
      double hipangle = n<2 ? 0 : M_PI/12; // -M_PI/18 : -M_PI/18
      double hiplowstop  = -conf.hipJointLimit;
      double hiphighstop = conf.hipJointLimit;
      double l2 =       n<2 ? conf.legLength*0.45 : conf.legLength*0.5;
      double t2       = conf.legLength/10;
      double kneeangle = n<2 ? M_PI/5 : -M_PI/5;
      double kneelowstop = -conf.kneeJointLimit;
      double kneehighstop = conf.kneeJointLimit;
      double l3 = n<2 ? conf.legLength*0.25 : 0;  // front legs have no feet
      double t3       = conf.legLength/12;
      double ankleangle = n<2 ? -M_PI/3 : 0;
      double anklelowstop = -M_PI/5;
      double anklehighstop = M_PI/3;

      // upper limp
      Primitive* p1;
      Pos pos = Pos(-conf.size/(2+0.2) + ((int)n/2) * legdist,
                    n%2==0 ? - twidth/2 : twidth/2,
                    n<2 ? conf.legLength  : conf.legLength - theight/3);
      osg::Matrix m = TRANSM(pos) * pose;

      p1 = new Capsule(t1, l1);
      p1->setTexture("Images/toy_fur3.jpg");
      p1->init(odeHandle, legmass*0.6, osgHandle);
      osg::Matrix m1 = TRANSM(0,0,-l1/2) * ROTM(hipangle,0,1,0) * m;
      p1->setPose(m1);
      objects.push_back(p1);
      // powered hip joint
      Pos nullpos(0,0,0);
      j = new HingeJoint(trunk, p1, nullpos * m, Axis(0,1,0) * m);
      j->init(odeHandle, osgHandleJ, true, t1 * 2.1);
      joints.push_back(j);
      servo =  new HingeServo(j,hiplowstop, hiphighstop,
                              conf.hipPower, conf.hipDamping,0 );
      hipservos.push_back(servo);

      // lower limp
      Primitive* p2;
      p2 = new Capsule(t2, l2);
      p2->setTexture("Images/toy_fur3.jpg");
      p2->init(odeHandle, legmass*0.3, osgHandle);
      osg::Matrix m2 = TRANSM(0,0,-l2/2) * ROTM(kneeangle,0, 1,0) *
        TRANSM(0,0,-l1/2) * m1;
      p2->setPose(m2);
      objects.push_back(p2);
      // powered knee joint
      j = new HingeJoint(p1, p2, Pos(0,0,-l1/2) * m1, Axis(0,n<2 ? -1 : 1,0) * m1);
      j->init(odeHandle, osgHandleJ, true, t1 * 2.1);
      joints.push_back(j);
      // lower limp should not collide with body!
      if(!conf.legBodyCollisions){
        legparts.push_back(p2);
        odeHandle.addIgnoredPair(trunk,p2);
      }

      // servo used as a spring
      servo =  new HingeServo(j, kneelowstop, kneehighstop, conf.kneePower, conf.kneeDamping,0);
      kneeservos.push_back(servo);


      if(n<2){
        // feet
        Primitive* p3;
        p3 = new Capsule(t3, l3);
        p3->setTexture("Images/toy_fur3.jpg");
        p3->init(odeHandle, legmass*0.2, osgHandle);
        osg::Matrix m3 = TRANSM(0,0,-l3/2) * ROTM(ankleangle,0, 1,0) *
          TRANSM(0,0,-l2/2) * m2;
        p3->setPose(m3);
        objects.push_back(p3);
        // powered ankle joint
        j = new HingeJoint(p2, p3, Pos(0,0,-l2/2) * m2, Axis(0,1,0) * m2);
        j->init(odeHandle, osgHandleJ, true, t2 * 2.1);
        joints.push_back(j);
        // feet should not collide with body!
        if(!conf.legBodyCollisions){
          legparts.push_back(p3);
          odeHandle.addIgnoredPair(trunk,p3);
        }

        // servo used as a spring
        servo =  new HingeServo(j, anklelowstop, anklehighstop, conf.anklePower, conf.ankleDamping, 0);
        ankleservos.push_back(servo);
      }

    }
    notifyOnChange("dummy"); // set all parameters
    created=true;
  };


  /** destroys vehicle and space
   */
  void VierBeiner::destroy(){
    if (created){
      if(conf.useBigBox){
        odeHandle.removeIgnoredPair(bigboxtransform,headtrans);
        odeHandle.removeIgnoredPair(bigboxtransform,neck);
        odeHandle.removeIgnoredPair(bigboxtransform,tail);
      }
      odeHandle.removeIgnoredPair(trunk,headtrans);

    if (conf.drawstupidface) {
      if(conf.useBigBox){
        odeHandle.removeIgnoredPair(bigboxtransform,mouth_trans);
        odeHandle.removeIgnoredPair(bigboxtransform,eye_l_trans);
        odeHandle.removeIgnoredPair(bigboxtransform,eye_r_trans);
        odeHandle.removeIgnoredPair(bigboxtransform,ear_l_trans);
        odeHandle.removeIgnoredPair(bigboxtransform,ear_r_trans);
      }
    }

    FOREACH(list<Primitive*>, legparts, p){
      odeHandle.removeIgnoredPair(trunk,*p);
    }

    FOREACH(vector<HingeServo*>, hipservos, i){
      if(*i) delete *i;
    }
      hipservos.clear();
      FOREACH(vector<HingeServo*>, kneeservos, i){
        if(*i) delete *i;
      }
      kneeservos.clear();
      FOREACH(vector<HingeServo*>, ankleservos, i){
        if(*i) delete *i;
      }
      ankleservos.clear();
      FOREACH(vector<HingeServo*>, headtailservos, i){
        if(*i) delete *i;
      }
      headtailservos.clear();

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

  void VierBeiner::notifyOnChange(const paramkey& key){
    // simply set all parameters
    FOREACH(vector<HingeServo*>, hipservos, i){
      if(*i) (*i)->setPower(conf.hipPower * conf.powerFactor);
    }
    FOREACH(vector<HingeServo*>, hipservos, i){
      if(*i) { (*i)->setDamping(conf.hipDamping * conf.dampingFactor); }
    }
    FOREACH(vector<HingeServo*>, kneeservos, i){
      if(*i) (*i)->setPower(conf.kneePower * conf.powerFactor);
    }
    FOREACH(vector<HingeServo*>, kneeservos, i){
      if(*i) {(*i)->setDamping(conf.kneeDamping * conf.dampingFactor);}
    }
    FOREACH(vector<HingeServo*>, ankleservos, i){
      if(*i) (*i)->setPower(conf.anklePower * conf.powerFactor);
    }
    FOREACH(vector<HingeServo*>, ankleservos, i){
      if(*i) {(*i)->setDamping(conf.ankleDamping * conf.dampingFactor); }
    }
    FOREACH(vector<HingeServo*>, hipservos, i){
      if(*i) (*i)->setMinMax(-conf.hipJointLimit,+conf.hipJointLimit);
    }
    FOREACH(vector<HingeServo*>, kneeservos, i){
      if(*i) (*i)->setMinMax(-conf.kneeJointLimit,+conf.kneeJointLimit);
    }
  }

}
