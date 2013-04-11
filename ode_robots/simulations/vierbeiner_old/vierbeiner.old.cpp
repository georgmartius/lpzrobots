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
 *   Revision 1.4  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.3  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.2  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.1  2007/02/23 09:37:50  der
 *   *** empty log message ***
 *
 *   Revision 1.1  2007/02/02 08:58:03  martius
 *   dog
 * *
 *
 ***************************************************************************/
#include <assert.h>
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>

// include header file
#include "vierbeiner.old.h"

using namespace osg;
using namespace std;

namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  VierBeinerOld::VierBeinerOld(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
           const VierBeinerOldConf& c, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(c)
  {
    // robot is not created till now
    created=false;

    // choose color here a pastel white is used
    this->osgHandle.color = Color(1.0, 156/255.0, 156/255.0, 1.0f);

    conf.motorPower *= conf.mass;
    conf.legLength *= conf.size;
    legmass=conf.mass * conf.relLegmass / conf.legNumber;    // mass of each legs
  };


  int VierBeinerOld::getMotorNumber(){
    return headtailservos.size() + hipservos.size() + kneeservos.size();
  };

  /* sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void VierBeinerOld::setMotors(const motor* motors, int motornumber){
    assert(created); // robot must exist

    int len = min(motornumber, getMotorNumber());
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
    assert(len==n);
    /// set knee servos to set point 0 (spring emulation)
//     FOREACH(vector <HingeServo*>, kneeservos, s){
//       (*s)->set(0);
//     }
  };

  int VierBeinerOld::getSensorNumber(){
    return headtailservos.size() + hipservos.size() + kneeservos.size();
  };

  /* returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int VierBeinerOld::getSensors(sensor* sensors, int sensornumber){
    assert(created);
    int len = min(sensornumber, getSensorNumber());
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
    assert(len==n);
    return n;
  };


  void VierBeinerOld::place(const Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    //    Matrix p2;
    //    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8));
    create(pose);
  };


  /**
   * updates the osg notes
   */
  void VierBeinerOld::update(){
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
      @param GlobalData structure that contains global data from the simulation environment
  */
  void VierBeinerOld::doInternalStuff(const GlobalData& global){ }

  /** checks for internal collisions and treats them.
   *  In case of a treatment return true (collision will be ignored by other objects
   *  and the default routine)  else false (collision is passed to other objects and
   *  (if not treated) to the default routine).
   */
  bool VierBeinerOld::collisionCallback(void *data, dGeomID o1, dGeomID o2){
    assert(created); // robot must exist

    //checks if one of the collision objects is part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space ){
      int i,n;
      const int N = 100;
      dContact contact[N];
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++){
        //      contact[i].surface.mode = dContactMu2 | dContactSlip1 | dContactSlip2 |
        //        dContactSoftERP | dContactSoftCFM | dContactApprox1;
        contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
          dContactSoftERP | dContactSoftCFM | dContactApprox1;
        contact[i].surface.slip1 = 0.005;
        contact[i].surface.slip2 = 0.005;
        contact[i].surface.mu = conf.frictionGround;
        contact[i].surface.soft_erp = 0.9;
        contact[i].surface.soft_cfm = 0.1;

        dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
        dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
      }
      return true;
    }
    return false;
  }


  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void VierBeinerOld::create( const Matrix& pose ){
    if (created) {
      destroy();
    }

    odeHandle.space = dSimpleSpaceCreate (parentspace);
    OsgHandle osgHandleJ = osgHandle.changeColor(Color(1.0,1.0,0.0));
    HingeJoint* j;
    HingeServo* servo;

    // create body
    Primitive* trunk;
    double twidth = conf.size / 1.5;
    double theight = conf.size / 6;
    trunk = new Box(conf.size, twidth, theight);
    trunk->init(odeHandle, conf.mass*0.8, osgHandle);
    trunk->setPose(osg::Matrix::translate(0,0,conf.legLength)*pose);
    objects.push_back(trunk);

    // create head and neck
    Primitive* neck;
    double necklength = conf.size/8;
    double neckwidth = theight/2;
    double headmass = conf.mass*0.1;
    neck = new Capsule(neckwidth,necklength);
    neck->init(odeHandle, headmass/2, osgHandle);
    Pos neckpos(conf.size/2.05,0,conf.legLength);
    neck->setPose(osg::Matrix::translate(0,0,necklength/2) *
                  osg::Matrix::rotate(M_PI/4,0,1,0) *
                  osg::Matrix::translate(neckpos)*pose);
    objects.push_back(neck);
    Primitive* head;
    head = new Sphere(theight);
    Primitive* trans = new Transform(neck, head, Matrix::translate(neckwidth/2, 0, necklength));
    trans->init(odeHandle, 0, osgHandle);
    objects.push_back(trans);
    j = new HingeJoint(trunk, neck, neckpos * pose, Axis(0,0,1) * pose);
    j->init(odeHandle, osgHandleJ, true, theight * 1.2);
    joints.push_back(j);
    servo =  new HingeServo(j, -M_PI/4, M_PI/4,
                            headmass,0.1,0.1);
    headtailservos.push_back(servo);

    // create tail
    Primitive* tail;
    double taillength = conf.size/4;
    double tailwidth = taillength/10;
    double tailmass  = headmass/2;
    tail = new Capsule(tailwidth,taillength);
    tail->init(odeHandle, headmass/2, osgHandle);
    Pos tailpos(-conf.size/1.96,0,conf.legLength+theight/3);
    tail->setPose(osg::Matrix::translate(0,0,taillength/2) *
                  osg::Matrix::rotate(M_PI/2.2,0,-1,0) *
                  osg::Matrix::translate(tailpos)*pose);
    objects.push_back(tail);
    j = new HingeJoint(trunk, tail, tailpos * pose, Axis(0,1,0) * pose);
    j->init(odeHandle, osgHandleJ, true, tailwidth * 2.05);
    j->setParam(dParamLoStop, -M_PI/2);
    j->setParam(dParamHiStop,  M_PI/2);
    joints.push_back(j);
    servo =  new HingeServo(j, -M_PI/3, M_PI/3, tailmass*2);
    headtailservos.push_back(servo);


    // legs  (counted from back to front)
    double legdist = conf.size*0.9 / (conf.legNumber/2-1);
    for ( int n = 0; n < conf.legNumber; n++ ) {
      double motorPower = conf.motorPower - 0.5 * conf.motorPower * ((int)n/2) / (conf.legNumber/2);

      // upper limp
      Primitive* p1;
      Pos pos = Pos(-conf.size/(2+0.2) + ((int)n/2) * legdist,
                    n%2==0 ? - twidth/2 : twidth/2,
                    conf.legLength);
      osg::Matrix m = osg::Matrix::translate(pos) * pose;

      double l1=conf.legLength*0.65;
      p1 = new Capsule(l1/8, l1);
      p1->init(odeHandle, legmass*0.6, osgHandle);
      double hipangle = n<2 ? 0 : M_PI/18;
      osg::Matrix m1 = osg::Matrix::translate(0,0,-l1/2) * osg::Matrix::rotate(hipangle,0,1,0) * m;
      p1->setPose(m1);
      objects.push_back(p1);

      // lower limp
      Primitive* p2;
      double l2=conf.legLength*0.5;
      p2 = new Capsule(l2/8, l2);
      p2->init(odeHandle, legmass*0.3, osgHandle);
      osg::Matrix m2 = osg::Matrix::translate(0,0,-l2/2) * osg::Matrix::rotate(-M_PI/5,0, n<2 ? -1 : 1,0) *
        osg::Matrix::translate(0,0,-l1/2) * m1;
      p2->setPose(m2);
      objects.push_back(p2);

      // powered hip joint
      Pos nullpos(0,0,0);
      j = new HingeJoint(trunk, p1, nullpos * m, Axis(0,1,0) * m);
      j->init(odeHandle, osgHandleJ, true, l1/8 * 2.1);
      j->setParam(dParamLoStop, -conf.jointLimit*1.5);
      j->setParam(dParamHiStop,  conf.jointLimit*1.5);
      joints.push_back(j);
      servo =  new HingeServo(j, -conf.jointLimit, conf.jointLimit,
                              motorPower,0.1,0);
      hipservos.push_back(servo);

      // passive knee joint
      j = new HingeJoint(p1, p2, Pos(0,0,-l1/2) * m1, Axis(0,n<2 ? -1 : 1,0) * m1);
      j->init(odeHandle, osgHandleJ, true, l1/8 * 2.1);

      // setting stops
      double lowstop = -M_PI/5;
      double highstop = M_PI/3;
      j->setParam(dParamLoStop, lowstop*1.5);
      j->setParam(dParamHiStop, highstop*1.5);
      joints.push_back(j);
      // servo used as a spring
      servo =  new HingeServo(j, lowstop, highstop, conf.kneePower, conf.kneeDamping,0);
      kneeservos.push_back(servo);
    }

    created=true;
  };


  /** destroys vehicle and space
   */
  void VierBeinerOld::destroy(){
    if (created){
      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
        if(*i) delete *i;
      }
      objects.clear();
      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
        if(*i) delete *i;
      }
      joints.clear();
      FOREACH(vector<HingeServo*>, hipservos, i){
        if(*i) delete *i;
      }
      hipservos.clear();
      FOREACH(vector<HingeServo*>, kneeservos, i){
        if(*i) delete *i;
      }
      kneeservos.clear();
      FOREACH(vector<HingeServo*>, headtailservos, i){
        if(*i) delete *i;
      }
      headtailservos.clear();
      dSpaceDestroy(odeHandle.space);
    }
    created=false;
  }



  /** The list of all parameters with there value as allocated lists.
      @param keylist,vallist will be allocated with malloc (free it after use!)
      @return length of the lists
  */
  Configurable::paramlist VierBeinerOld::getParamList() const{
    paramlist list;
    list += pair<paramkey, paramval> (string("frictionground"), conf.frictionGround);
    list += pair<paramkey, paramval> (string("motorpower"),   conf.motorPower);
    list += pair<paramkey, paramval> (string("kneepower"),   conf.kneePower);
    list += pair<paramkey, paramval> (string("kneedamping"),   conf.kneeDamping);
    return list;
  }


  Configurable::paramval VierBeinerOld::getParam(const paramkey& key, bool traverseChildren) const{
    if(key == "frictionground") return conf.frictionGround;
    else if(key == "motorpower") return conf.motorPower;
    else if(key == "kneepower") return conf.kneePower;
    else if(key == "kneedamping") return conf.kneeDamping;
    else  return Configurable::getParam(key) ;
  }

  bool VierBeinerOld::setParam(const paramkey& key, paramval val, bool traverseChildren){
    if(key == "frictionground") conf.frictionGround = val;
    else if(key == "motorpower") {
      conf.motorPower = val;
      FOREACH(vector<HingeServo*>, hipservos, i){
        if(*i) (*i)->setPower(conf.motorPower);
      }
    } else if(key == "kneepower") {
      conf.kneePower = val;
      FOREACH(vector<HingeServo*>, kneeservos, i){
        if(*i) (*i)->setPower(conf.kneePower);
      }
    } else if(key == "kneedamping") {
      conf.kneeDamping = val;
      FOREACH(vector<HingeServo*>, kneeservos, i){
        if(*i) {(*i)->damping() = conf.kneeDamping; cout << (*i)->damping() << endl; }
      }
    } else return Configurable::setParam(key, val);
    return true;
  }


}
