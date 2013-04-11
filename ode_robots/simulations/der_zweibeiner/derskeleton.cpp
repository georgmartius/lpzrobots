/***************************************************************************
 *   Copyright (C) 2007 by Robot Group Leipzig                             *
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
 *   Revision 1.5  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.4  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.3  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.2  2007/11/21 13:18:10  der
 *   ralfs aenderungen
 *
 *   Revision 1.3  2007/09/06 18:50:33  martius
 *   *** empty log message ***
 *
 *   Revision 1.2  2007/07/31 08:35:28  martius
 *   addSpace
 *
 *   Revision 1.1  2007/07/17 07:25:26  martius
 *   first attempt to build a two legged robot (humanoid)
 *
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
#include <ode_robots/twoaxisservo.h>

// include header file
#include "derskeleton.h"

using namespace osg;
using namespace std;

namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  Skeleton::Skeleton(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
           const SkeletonConf& c, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(c)
  {
    // robot is not created till now
    created=false;

    //    this->osgHandle.color = Color(1.0, 1,1,1);
    // choose color here a pastel white is used
    this->osgHandle.color = Color(217/255.0, 209/255.0, 109/255.0, 1.0f);
  };


  int Skeleton::getMotorNumber(){
    if(conf.onlyPrimaryFunctions)
      return hipservos.size() + kneeservos.size() + ankleservos.size() + armservos.size()+ 1/*pelvis*/ ;
    else
      return hipservos.size()*2 + kneeservos.size() + ankleservos.size() + armservos.size()*2 +
        2/*pelvis*/+ headservos.size();
  };

  /* sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Skeleton::setMotors(const motor* motors, int motornumber){
    assert(created); // robot must exist

    int len = min(motornumber, getMotorNumber());
    // controller output as torques
    int n=0;
    FOREACH(vector <TwoAxisServo*>, hipservos, s){
      if(conf.onlyPrimaryFunctions){
        (*s)->set(motors[n],0);
      } else {
        (*s)->set(motors[n],motors[n+1]);
        n++;
      }
      n++;
    }
    FOREACH(vector <OneAxisServo*>, kneeservos, s){
      (*s)->set(motors[n]);
      n++;
    }
    FOREACH(vector <OneAxisServo*>, ankleservos, s){
      (*s)->set(motors[n]);
      n++;
    }
    FOREACH(vector <TwoAxisServo*>, armservos, s){
      if(conf.onlyPrimaryFunctions){
        (*s)->set(motors[n],0);
      } else {
        (*s)->set(motors[n],motors[n+1]);
        n++;
      }
      n++;
    }

    if(!conf.onlyPrimaryFunctions){
      pelvisservo->set(motors[n],motors[n+1]);
      n+=2;
    }else{
      pelvisservo->set(motors[n],0);
      n++;
    }

    FOREACH(vector <OneAxisServo*>, headservos, s){
      if(!conf.onlyPrimaryFunctions){
        (*s)->set(motors[n]);
        n++;
      }else
        (*s)->set(0);
    }

    assert(len==n);
  };

  int Skeleton::getSensorNumber(){
    if(conf.onlyPrimaryFunctions)
      return hipservos.size() + kneeservos.size() + ankleservos.size() + armservos.size() + 1 /*pelvis*/;
    else
      return hipservos.size()*2 + kneeservos.size() + ankleservos.size() + armservos.size()*2
        + 2 /*pelvis*/ + headservos.size()  ;
  };

  /* returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Skeleton::getSensors(sensor* sensors, int sensornumber){
    assert(created);
    int len = min(sensornumber, getSensorNumber());
    int n=0;
    FOREACHC(vector <TwoAxisServo*>, hipservos, s){
      sensors[n]   = (*s)->get1();
      if(!conf.onlyPrimaryFunctions){
        n++;
        sensors[n]   = (*s)->get2();
      }
      n++;
    }
    FOREACHC(vector <OneAxisServo*>, kneeservos, s){
      sensors[n]   = (*s)->get();
      n++;
    }
    FOREACHC(vector <OneAxisServo*>, ankleservos, s){
      sensors[n]   = (*s)->get();
      n++;
    }
    FOREACHC(vector <TwoAxisServo*>, armservos, s){
      sensors[n]   = (*s)->get1();
      if(!conf.onlyPrimaryFunctions){
        n++;
        sensors[n]   = (*s)->get2();
      }
      n++;
    }

    sensors[n] = pelvisservo->get1();
    n++;
    if(!conf.onlyPrimaryFunctions){
      sensors[n] = pelvisservo->get2();
      n++;
      FOREACHC(vector <OneAxisServo*>, headservos, s){
        sensors[n]   = (*s)->get();
        n++;
      }
    }
    assert(len==n);
    return n;
  };


  void Skeleton::place(const Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    //    Matrix p2;
    //    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8));
    create(pose);
  };


  /**
   * updates the osg notes
   */
  void Skeleton::update(){
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
  void Skeleton::doInternalStuff(const GlobalData& global){
  }


  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void Skeleton::create( const Matrix& pose ){
    if (created) {
      destroy();
    }

    odeHandle.space = dSimpleSpaceCreate (parentspace);
    odeHandle.addSpace(odeHandle.space);
    OsgHandle osgHandleJ = osgHandle.changeColor(Color(1.0,0.0,0.0));
    HingeJoint* j;
    UniversalJoint* uj;
    FixedJoint* fj;
    OneAxisServo* servo1;
    TwoAxisServo* servo2;
    Primitive* b ;

    objects.clear();
    objects.resize(LastPart);

    // this is taken from DANCE, therefore the body creation is rather static
    // body creation
    // Hip
    b = new Box(0.2,0.1,0.1);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::translate(0, 1.131, 0.0052) * pose );
    b->setMass(16.61, 0, 0, 0, 0.0996, 0.1284, 0.1882, 0, 0, 0);
    objects[Hip]=b;

    // Trunk_comp
    //    b = new Mesh("Meshes/skeleton/Trunk_comp_center.wrl",1);
    b = new Box(0.3,0.45,0.2);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::translate(0, 1.39785, 0.0201) * pose );
    b->setMass(29.27, 0, 0, 0, 0.498, 0.285, 0.568, 0, 0, 0);
    objects[Trunk_comp]=b;

    // Neck
    b = new Capsule(0.05,0.08);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0, 1.6884, 0.0253) * pose );
    b->setMass(1, 0, 0, 0, 0.0003125, 0.0003125, 0.0003125, 0, 0, 0);
    objects[Neck]=b;


    // Head_comp
    b = new Sphere(0.1);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::translate(0, 1.8106, 0.063) * pose );
    b->setMass(5.89, 0, 0, 0, 0.0413, 0.0306, 0.0329, 0, 0, 0);
    objects[Head_comp]=b;

    // Left_Shoulder
    b = new Capsule(0.04,0.28);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(0.3094, 1.587, 0.0227) * pose );
    b->setMass(2.79, 0, 0, 0, 0.00056, 0.021, 0.021, 0, 0, 0);
    objects[Left_Shoulder]=b;

    // Left_Forearm
    b = new Capsule(0.035,0.28);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(0.5798, 1.5909, 0.024) * pose );
    b->setMass(1.21, 0, 0, 0, 0.00055, 0.0076, 0.0076, 0, 0, 0);
    objects[Left_Forearm]=b;

    // Left_Hand
    b = new Cylinder(0.06,0.05);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0.7826, 1.5948, 0.024) * pose );
    b->setMass(0.55, 0, 0, 0, 0.00053, 0.047, 0.0016, 0, 0, 0);
    objects[Left_Hand]=b;

    // Right_Shoulder
    b = new Capsule(0.04,0.28);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(-0.3094, 1.587, 0.0227) * pose );
    b->setMass(2.79, 0, 0, 0, 0.00056, 0.021, 0.021, 0, 0, 0);
    objects[Right_Shoulder]=b;

    // Right_Forearm
    b = new Capsule(0.035,0.28);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(-0.5798, 1.5909, 0.024) * pose );
    b->setMass(1.21, 0, 0, 0, 0.00055, 0.0076, 0.0076, 0, 0, 0);
    objects[Right_Forearm]=b;

    // Right_Hand
    b = new Cylinder(0.06,0.05);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(-0.7826, 1.5948, 0.024) * pose );
    b->setMass(0.55, 0, 0, 0, 0.00053, 0.047, 0.0016, 0, 0, 0);
    objects[Right_Hand]=b;

    // Left_Thigh
    b = new Capsule(0.07,0.43);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0)* osg::Matrix::rotate(-M_PI/60,0,0,1) *
               osg::Matrix::translate(0.0949, 0.8525, 0.0253) * pose );
    b->setMass(8.35, 0, 0, 0, 0.145, 0.0085, 0.145, 0, 0, 0);
    objects[Left_Thigh]=b;

    // Left_Shin
    b = new Capsule(0.06,0.35);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0.0702, 0.3988, 0.0357) * pose );
    b->setMass(4.16, 0, 0, 0, 0.069, 0.0033, 0.069, 0, 0, 0);
    objects[Left_Shin]=b;

    // Left_Foot
    b = new Box(0.1,0.05,0.3);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::translate(0.0624, 0.1388, 0.0708) * pose );
    b->setMass(1.34, 0, 0, 0, 0.0056, 0.0056, 0.00036, 0, 0, 0);
    objects[Left_Foot]=b;

    // Right_Thigh
    b = new Capsule(0.07,0.43);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0)* osg::Matrix::rotate(M_PI/60,0,0,1) *
               osg::Matrix::translate(-0.0949, 0.8525, 0.0253) * pose );
    b->setMass(8.35, 0, 0, 0, 0.145, 0.0085, 0.145, 0, 0, 0);
    objects[Right_Thigh]=b;

    // Right_Shin
    b = new Capsule(0.06,0.35);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(-0.0702, 0.3988, 0.0357) * pose );
    b->setMass(4.16, 0, 0, 0, 0.069, 0.0033, 0.069, 0, 0, 0);
    objects[Right_Shin]=b;

    // Right_Foot
    b = new Box(0.1,0.05,0.3);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::translate(-0.0624, 0.1388, 0.0708) * pose );
    b->setMass(1.34, 0, 0, 0, 0.0056, 0.0056, 0.00036, 0, 0, 0);
    objects[Right_Foot]=b;


    // joint creation
    // Hip and Trunk
    //    j = new BallJoint(objects[Hip], objects[Trunk_comp], Pos(0, 1.2516, 0.0552) * pose, Axis(0,0,1) * pose);
    uj = new UniversalJoint(objects[Hip], objects[Trunk_comp], Pos(0, 1.2516, 0.0552) * pose,
                           Axis(0,0,1) * pose, Axis(0,1,0) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.2);
    joints.push_back(uj);

    pelvisservo =  new TwoAxisServo(uj, -conf.pelvisJointLimit, conf.pelvisJointLimit, conf.pelvisPower,
                                    -conf.pelvisJointLimit, conf.pelvisJointLimit, conf.pelvisPower);



    // Neck and Trunk

    // The head and Neck joint we will make very simple, just lateral movements posible
    /// actually we should have a ball joint in the neck and another 2D joint to the head
    j = new HingeJoint(objects[Trunk_comp], objects[Neck], Pos(0, 1.6442, 0.0188) * pose,
                       Axis(0,0,1) * pose);
    j->init(odeHandle, osgHandleJ, true, 0.12);
    joints.push_back(j);

    servo1 = new OneAxisServo(j, -M_PI/10, M_PI/10, 20,0.1);
    headservos.push_back(servo1);

    // Head and Neck
    fj = new FixedJoint(objects[Neck], objects[Head_comp]); // ,Pos(0, 1.7326, 0.0318) * pose);
    fj->init(odeHandle, osgHandleJ, false);
    joints.push_back(fj);

    // Trunk and Shoulders (Arms)
    uj = new UniversalJoint(objects[Trunk_comp], objects[Left_Shoulder], Pos(0.1768, 1.587, 0.0214) * pose,
                           Axis(0,0,1) * pose, Axis(0,1,0) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.12);
    joints.push_back(uj);

    servo2 =  new TwoAxisServo(uj, -conf.armJointLimit, conf.armJointLimit, conf.armPower,
                                    -conf.armJointLimit, conf.armJointLimit, conf.armPower);
    //                                    -M_PI/30, M_PI/4, conf.armPower);
    armservos.push_back(servo2);

    uj = new UniversalJoint(objects[Trunk_comp], objects[Right_Shoulder], Pos(-0.1768, 1.587, 0.0214) * pose,
                           Axis(0,0,-1) * pose, Axis(0,-1,0) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.12);
    joints.push_back(uj);

    servo2 =  new TwoAxisServo(uj, -conf.armJointLimit, conf.armJointLimit, conf.armPower,
                                    -conf.armJointLimit, conf.armJointLimit, conf.armPower);
    //                                    -M_PI/30, M_PI/4, conf.armPower);
    armservos.push_back(servo2);

    // Arms and Hands are fixed
    fj = new FixedJoint(objects[Left_Shoulder], objects[Left_Forearm]); // ,Pos(0.442, 1.587, 0.024) * pose);
    fj->init(odeHandle, osgHandleJ, false);
    joints.push_back(fj);
    fj = new FixedJoint(objects[Right_Shoulder], objects[Right_Forearm]); // ,Pos(-0.442, 1.587, 0.024) * pose);
    fj->init(odeHandle, osgHandleJ, false);
    joints.push_back(fj);
    fj = new FixedJoint(objects[Left_Forearm], objects[Left_Hand]); // ,Pos(0.7176, 1.5948, 0.024) * pose);
    fj->init(odeHandle, osgHandleJ, false);
    joints.push_back(fj);
    fj = new FixedJoint(objects[Right_Forearm], objects[Right_Hand]); // ,Pos(-0.7176, 1.5948, 0.024) * pose);
    fj->init(odeHandle, osgHandleJ, false);
    joints.push_back(fj);

    // TODO: substitute the servos with Muscles
    // Hip and Thighs
    uj = new UniversalJoint(objects[Hip], objects[Left_Thigh], Pos(0.1118, 1.0904, 0.011) * pose,
                           Axis(1,0,0) * pose, Axis(0,0,-1) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.15);
    joints.push_back(uj);

    servo2 =  new TwoAxisServo(uj, -conf.hipJointLimit, conf.hipJointLimit, conf.hipPower,
                              -conf.hip2JointLimit, conf.hip2JointLimit*2, conf.hip2Power, conf.hipDamping);
    servo2->damping2() = conf.hip2Damping;
    hipservos.push_back(servo2);

    uj = new UniversalJoint(objects[Hip], objects[Right_Thigh], Pos(-0.1118, 1.0904, 0.011) * pose,
                           Axis(1,0,0) * pose, Axis(0,0,1) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.15);
    joints.push_back(uj);

    servo2 =  new TwoAxisServo(uj, -conf.hipJointLimit, conf.hipJointLimit, conf.hipPower,
                              -conf.hip2JointLimit, conf.hip2JointLimit*2, conf.hip2Power, conf.hipDamping);
    servo2->damping2() = conf.hip2Damping;
    hipservos.push_back(servo2);


    // Thighs and Shins (Knees)
    j = new HingeJoint(objects[Left_Thigh], objects[Left_Shin], Pos(0.078, 0.6146, 0.0396) * pose,
                       Axis(1,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true, 0.12);
    joints.push_back(j);

    servo1 = new OneAxisServo(j, -conf.kneeJointLimit, conf.kneeJointLimit, conf.kneePower, conf.kneeDamping,0);
    kneeservos.push_back(servo1);

    j = new HingeJoint(objects[Right_Thigh], objects[Right_Shin], Pos(-0.078, 0.6146, 0.0396) * pose,
                       Axis(1,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true, 0.12);
    joints.push_back(j);

    servo1 = new OneAxisServo(j, -conf.kneeJointLimit, conf.kneeJointLimit, conf.kneePower, conf.kneeDamping,0);
    kneeservos.push_back(servo1);

    // Shins and Feet (Ankles)
    j = new HingeJoint(objects[Left_Shin], objects[Left_Foot], Pos(0.0624, 0.183, 0.0318) * pose,
                       Axis(1,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true, 0.1);
    joints.push_back(j);

    servo1 = new OneAxisServo(j, -conf.ankleJointLimit, conf.ankleJointLimit, conf.anklePower, conf.ankleDamping,0);
    ankleservos.push_back(servo1);

    j = new HingeJoint(objects[Right_Shin], objects[Right_Foot], Pos(-0.0624, 0.183, 0.0318) * pose,
                       Axis(1,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true, 0.1);
    joints.push_back(j);

    servo1 = new OneAxisServo(j, -conf.ankleJointLimit, conf.ankleJointLimit, conf.anklePower, conf.ankleDamping,0);
    ankleservos.push_back(servo1);


    created=true;
  };


  /** destroys vehicle and space
   */
  void Skeleton::destroy(){
    if (created){
//       odeHandle.removeIgnoredPair(bigboxtransform,headtrans);
//       odeHandle.removeIgnoredPair(bigboxtransform,neck);
//       odeHandle.removeIgnoredPair(trunk,headtrans);
//       odeHandle.removeIgnoredPair(bigboxtransform,tail);


      FOREACH(vector<TwoAxisServo*>, hipservos, i){
        if(*i) delete *i;
      }
      hipservos.clear();
      FOREACH(vector<OneAxisServo*>, kneeservos, i){
        if(*i) delete *i;
      }
      kneeservos.clear();
      FOREACH(vector<OneAxisServo*>, ankleservos, i){
        if(*i) delete *i;
      }
      ankleservos.clear();
      FOREACH(vector<OneAxisServo*>, headservos, i){
        if(*i) delete *i;
      }
      FOREACH(vector<TwoAxisServo*>, armservos, i){
        if(*i) delete *i;
      }
      headservos.clear();

      if(pelvisservo) delete pelvisservo;

      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
        if(*i) delete *i;
      }
      joints.clear();

      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
        if(*i) delete *i;
      }
      objects.clear();
      odeHandle.removeSpace(odeHandle.space);
      dSpaceDestroy(odeHandle.space);
    }

    created=false;
  }



  /** The list of all parameters with there value as allocated lists.
      @param keylist,vallist will be allocated with malloc (free it after use!)
      @return length of the lists
  */
  Configurable::paramlist Skeleton::getParamList() const{
    paramlist list;
    list += pair<paramkey, paramval> (string("hippower"),   conf.hipPower);
    list += pair<paramkey, paramval> (string("hipdamping"),   conf.hipDamping);
    list += pair<paramkey, paramval> (string("hipjointlimit"),   conf.hipJointLimit);
    list += pair<paramkey, paramval> (string("kneepower"),   conf.kneePower);
    list += pair<paramkey, paramval> (string("kneedamping"),   conf.kneeDamping);
    list += pair<paramkey, paramval> (string("kneejointlimit"),   conf.kneeJointLimit);
    list += pair<paramkey, paramval> (string("anklepower"),   conf.anklePower);
    list += pair<paramkey, paramval> (string("ankledamping"),   conf.ankleDamping);
    list += pair<paramkey, paramval> (string("anklejointlimit"),   conf.ankleJointLimit);
    list += pair<paramkey, paramval> (string("armpower"),   conf.armPower);
    list += pair<paramkey, paramval> (string("armdamping"),   conf.armDamping);
    list += pair<paramkey, paramval> (string("armjointlimit"),   conf.armJointLimit);
    list += pair<paramkey, paramval> (string("hip2power"),   conf.hip2Power);
    list += pair<paramkey, paramval> (string("hip2damping"),   conf.hip2Damping);
    list += pair<paramkey, paramval> (string("hip2jointlimit"),   conf.hip2JointLimit);
    list += pair<paramkey, paramval> (string("pelvispower"),   conf.pelvisPower);
    list += pair<paramkey, paramval> (string("pelvisdamping"),   conf.pelvisDamping);
    list += pair<paramkey, paramval> (string("pelvisjointlimit"),   conf.pelvisJointLimit);
    return list;
  }


  Configurable::paramval Skeleton::getParam(const paramkey& key, bool traverseChildren) const{
    if(key == "hippower") return conf.hipPower;
    else if(key == "hipdamping") return conf.hipDamping;
    else if(key == "hipjointlimit") return conf.hipJointLimit;
    else if(key == "kneepower") return conf.kneePower;
    else if(key == "kneedamping") return conf.kneeDamping;
    else if(key == "kneejointlimit") return conf.kneeJointLimit;
    else if(key == "anklepower") return conf.anklePower;
    else if(key == "ankledamping") return conf.ankleDamping;
    else if(key == "anklejointlimit") return conf.ankleJointLimit;
    else if(key == "armpower") return conf.armPower;
    else if(key == "armdamping") return conf.armDamping;
    else if(key == "armjointlimit") return conf.armJointLimit;
    else if(key == "hip2power") return conf.hip2Power;
    else if(key == "hip2damping") return conf.hip2Damping;
    else if(key == "hip2jointlimit") return conf.hip2JointLimit;
    else if(key == "pelvispower") return conf.pelvisPower;
    else if(key == "pelvisdamping") return conf.pelvisDamping;
    else if(key == "pelvisjointlimit") return conf.pelvisJointLimit;

    else  return Configurable::getParam(key) ;
  }

  bool Skeleton::setParam(const paramkey& key, paramval val, bool traverseChildren){
    if(key == "hippower") {
      conf.hipPower = val;
      FOREACH(vector<TwoAxisServo*>, hipservos, i){
        if(*i) (*i)->power1() = conf.hipPower;
      }
    } else if(key == "hipdamping") {
      conf.hipDamping = val;
      FOREACH(vector<TwoAxisServo*>, hipservos, i){
        if(*i) { (*i)->damping1() = conf.hipDamping; }
      }
    } else if(key == "hipjointlimit") {
      conf.hipJointLimit = val;
      FOREACH(vector<TwoAxisServo*>, hipservos, i){
        if(*i) (*i)->setMinMax1(-val,+val);
      }
    } else if(key == "hip2power") {
      conf.hip2Power = val;
      FOREACH(vector<TwoAxisServo*>, hipservos, i){
        if(*i) (*i)->power2() = conf.hip2Power;
      }
    } else if(key == "hip2damping") {
      conf.hip2Damping = val;
      FOREACH(vector<TwoAxisServo*>, hipservos, i){
        if(*i) { (*i)->damping2() = conf.hip2Damping; }
      }
    } else if(key == "hip2jointlimit") {
      conf.hip2JointLimit = val;
      FOREACH(vector<TwoAxisServo*>, hipservos, i){
        if(*i) (*i)->setMinMax2(-val,+val);
      }
    } else if(key == "kneepower") {
      conf.kneePower = val;
      FOREACH(vector<OneAxisServo*>, kneeservos, i){
        if(*i) (*i)->power() = conf.kneePower;
      }
    } else if(key == "kneedamping") {
      conf.kneeDamping = val;
      FOREACH(vector<OneAxisServo*>, kneeservos, i){
        if(*i) {(*i)->damping() = conf.kneeDamping;}
      }
    } else if(key == "kneejointlimit") {
      conf.kneeJointLimit = val;
      FOREACH(vector<OneAxisServo*>, kneeservos, i){
        if(*i) (*i)->setMinMax(-val,+val);
      }
    } else if(key == "anklepower") {
      conf.anklePower = val;
      FOREACH(vector<OneAxisServo*>, ankleservos, i){
        if(*i) (*i)->power() = conf.anklePower;
      }
    } else if(key == "ankledamping") {
      conf.ankleDamping = val;
      FOREACH(vector<OneAxisServo*>, ankleservos, i){
        if(*i) {(*i)->damping() = conf.ankleDamping; }
      }
    } else if(key == "anklejointlimit") {
      conf.ankleJointLimit = val;
      FOREACH(vector<OneAxisServo*>, ankleservos, i){
        if(*i) (*i)->setMinMax(-val,+val);
      }
    } else if(key == "armpower") {
      conf.armPower = val;
      FOREACH(vector<TwoAxisServo*>, armservos, i){
        if(*i) (*i)->power1() = conf.armPower;
        if(*i) (*i)->power2() = conf.armPower;
      }
    } else if(key == "armdamping") {
      conf.armDamping = val;
      FOREACH(vector<TwoAxisServo*>, armservos, i){
        if(*i) {(*i)->damping1() = conf.armDamping; }
        if(*i) {(*i)->damping2() = conf.armDamping; }
      }
    }  else if(key == "armjointlimit") {
      conf.armJointLimit = val;
      FOREACH(vector<TwoAxisServo*>, armservos, i){
        if(*i) (*i)->setMinMax1(-val,+val);
        if(*i) (*i)->setMinMax2(-val,+val);
      }
    } else if(key == "pelvispower") {
      conf.pelvisPower = val;
      pelvisservo->power1() = conf.kneePower;
      pelvisservo->power2() = conf.kneePower;
    } else if(key == "pelvisdamping") {
      conf.pelvisDamping = val;
      pelvisservo->damping1() = conf.pelvisDamping;
      pelvisservo->damping2() = conf.pelvisDamping;
    } else if(key == "pelvisjointlimit") {
      conf.pelvisJointLimit = val;
      pelvisservo->setMinMax1(-val,+val);
      pelvisservo->setMinMax2(-val,+val);
    } else return Configurable::setParam(key, val);
    return true;
  }

}
