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
 *   Revision 1.6  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.5  2010/11/05 13:54:05  martius
 *   store and restore for robots implemented
 *
 *   Revision 1.4  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.3  2009/11/26 14:21:54  der
 *   Larger changes
 *   :wq
 *
 *   wq
 *
 *   Revision 1.2  2009/08/12 10:30:25  der
 *   skeleton has belly joint
 *   works fine with centered servos
 *
 *   Revision 1.1  2009/08/10 15:00:46  der
 *   version that Ralf did at home
 *   Skeleton bugfixing, works now fine with ServoVel
 *
 *   Revision 1.16  2009/08/09 20:20:37  der
 *   From PC home
 *
 *   Revision 1.15  2009/05/11 17:01:20  martius
 *   new velocity servos implemented
 *   reorganized parameters, now also neck and elbows are configurable
 *
 *   Revision 1.14  2009/04/26 10:29:21  martius
 *   moved setcolor for boxed to osgHandle.changeColor
 *
 *   Revision 1.13  2009/03/30 13:56:07  martius
 *   fixed textures
 *
 *   Revision 1.12  2009/01/20 17:29:52  martius
 *   cvs commit
 *
 *   Revision 1.11  2008/11/14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.10  2008/11/11 19:42:20  der
 *   Some changes
 *
 *   Revision 1.9  2008/06/20 14:03:01  guettler
 *   reckturner
 *
 *   Revision 1.8  2008/05/27 13:25:12  guettler
 *   powerfactor moved to skeleton
 *
 *   Revision 1.7  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.6  2008/04/10 12:27:13  der
 *   some changes
 *
 *   Revision 1.5  2008/03/14 08:04:23  der
 *   Some changes in main and skeleton (with new outfit)
 *
 *   Revision 1.4  2008/02/28 07:43:03  der
 *   small changes
 *
 *   Revision 1.3  2008/02/08 13:35:10  der
 *   satelite teaching
 *
 *   Revision 1.2  2008/02/07 14:25:02  der
 *   added setTexture and setColor for skeleton
 *
 *   Revision 1.1  2008/01/29 09:52:16  der
 *   first version
 *
 *   Revision 1.4  2007/11/07 13:27:28  martius
 *   doInternalstuff changed
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
#include <ode_robots/angularmotor.h>
#include <ode_robots/raysensorbank.h>
#include <ode_robots/irsensor.h>

// include header file
#include "skeleton.h"

using namespace osg;
using namespace std;

namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  Skeleton::Skeleton(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
           SkeletonConf& c, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(c)
  {
    // robot is not created till now
    created=false;

    //    this->osgHandle.color = Color(1.0, 1,1,1);
    // choose color here a pastel white is used
    //    this->osgHandle.color = Color(217/255.0, 209/255.0, 109/255.0, 1.0f);
    conf.hipPower*= conf.powerfactor;
    conf.hip2Power*= conf.powerfactor;
    conf.pelvisPower*= conf.powerfactor;
    conf.backPower*= conf.powerfactor;
    conf.kneePower*= conf.powerfactor;
    conf.anklePower*= conf.powerfactor;
    conf.armPower*= conf.powerfactor;
    conf.elbowPower*= conf.powerfactor;
    conf.neckPower*= conf.powerfactor;

    conf.hipDamping*= conf.dampingfactor;
    conf.hip2Damping*= conf.dampingfactor;
    conf.pelvisDamping*= conf.dampingfactor;
    conf.backDamping*= conf.dampingfactor;
    conf.kneeDamping*= conf.dampingfactor;
    conf.ankleDamping*= conf.dampingfactor;
    conf.armDamping*= conf.dampingfactor;
    conf.elbowDamping*= conf.dampingfactor;
    conf.neckDamping*= conf.dampingfactor;

    addParameter("hippower",   &conf.hipPower);
    addParameter("hipdamping",   &conf.hipDamping);
    addParameter("hipvelocity", &conf.hipVelocity);
    addParameter("hipjointlimit",   &conf.hipJointLimit);
    addParameter("hip2power",   &conf.hip2Power);
    addParameter("hip2damping",   &conf.hip2Damping);
    addParameter("hip2jointlimit",   &conf.hip2JointLimit);
    addParameter("neckpower",   &conf.neckPower);
    addParameter("neckdamping",   &conf.neckDamping);
    addParameter("neckvelocity",   &conf.neckVelocity);
    addParameter("neckjointlimit",   &conf.neckJointLimit);
    addParameter("kneepower",   &conf.kneePower);
    addParameter("kneedamping",   &conf.kneeDamping);
    addParameter("kneevelocity",   &conf.kneeVelocity);
    addParameter("kneejointlimit",   &conf.kneeJointLimit);
    addParameter("anklepower",   &conf.anklePower);
    addParameter("ankledamping",   &conf.ankleDamping);
    addParameter("anklevelocity",   &conf.ankleVelocity);
    addParameter("anklejointlimit",   &conf.ankleJointLimit);
    addParameter("armpower",   &conf.armPower);
    addParameter("armdamping",   &conf.armDamping);
    addParameter("armvelocity",   &conf.armVelocity);
    addParameter("armjointlimit",   &conf.armJointLimit);
    addParameter("elbowpower",   &conf.elbowPower);
    addParameter("elbowdamping",   &conf.elbowDamping);
    addParameter("elbowvelocity",   &conf.elbowVelocity);
    addParameter("elbowjointlimit",   &conf.elbowJointLimit);
    addParameter("pelvispower",   &conf.pelvisPower);
    addParameter("pelvisdamping",   &conf.pelvisDamping);
    addParameter("pelvisvelocity",   &conf.pelvisVelocity);
    addParameter("pelvisjointlimit",   &conf.pelvisJointLimit);
    addParameter("backpower",   &conf.backPower);
    addParameter("backdamping",   &conf.backDamping);
    addParameter("backvelocity",   &conf.backVelocity);
    addParameter("backjointlimit",   &conf.backJointLimit);
  };


  int Skeleton::getMotorNumber(){
    if(conf.onlyPrimaryFunctions)
      return hipservos.size() + kneeservos.size() + ankleservos.size() + armservos.size()+ 1/*pelvis*/ ;
    else
      return hipservos.size()*2 + kneeservos.size() + ankleservos.size() + armservos.size()*2 + arm1servos.size() +
        1/*pelvis*/+ backservos.size() +2*headservos.size();
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
    FOREACH(vector <OneAxisServo*>, arm1servos, s){
      (*s)->set(motors[n]);
      n++;
    }
    pelvisservo->set(motors[n]);
    n++;
    if(conf.useBackJoint){
      if(!conf.onlyPrimaryFunctions){
        FOREACH(vector <OneAxisServo*>, backservos, s){
          (*s)->set(motors[n]);
          n++;
        }
      }else{
        FOREACH(vector <OneAxisServo*>, backservos, s){
          (*s)->set(0);
        }
      }
    }

    //    FOREACH(vector <OneAxisServo*>, headservos, s){
//         (*s)->set(motors[n]);
//         n++;
//}
    FOREACH(vector <TwoAxisServo*>, headservos, s){
      if(!conf.onlyPrimaryFunctions){

        (*s)->set(motors[n],motors[n+1]);
        n+=2;
      }else
//         (*s)->set(0);
        (*s)->set(0,0);
    }

    FOREACH(vector <AngularMotor*>, frictionmotors, s){
      // force both axis to have zero velocity
      (*s)->set(0, 0.0);
      (*s)->set(1, 0.0);
    }

    assert(len==n);
  };

  int Skeleton::getSensorNumber(){
    int numberSensors=0;

    if(conf.onlyPrimaryFunctions){
      numberSensors +=hipservos.size() + kneeservos.size() + ankleservos.size() +
        armservos.size() + arm1servos.size() + 1 /*pelvis*/;
    } else {
    //  return 1;
      numberSensors += hipservos.size()*2 + kneeservos.size() + ankleservos.size() +
        armservos.size()*2 + arm1servos.size() +
        1/*pelvis*/+ backservos.size() + 2* headservos.size() ;
    }

    numberSensors += irSensorBank.size();

//     // add new one!
//     // head and trunk position (z): +2
//     //    numberSensors+=2;

    return numberSensors;
  }

  /*****************************
GUIDE adding new sensors
1. in getSensorNumber() Anzahl der Sensoren korrigieren: numberSensors+=1;
2. in getSensors() dem Array sensors neue Sensorwerte zuweisen, z.B: sensors[n++]=getHeadPosition().z;


   ****************************/

  /* returns actual sensorvaluess
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Skeleton::getSensors(sensor* sensors, int sensornumber){
    assert(created);
    int len = min(sensornumber, getSensorNumber());
    int n=0; // index variable
    FOREACHC(vector <TwoAxisServo*>, hipservos, s){ //0-3
      sensors[n]   = (*s)->get1();
      if(!conf.onlyPrimaryFunctions){
        n++;
        sensors[n]   = (*s)->get2();
      }
      n++;
    }
//     PID pid1 = hipservos.front()->pid1;
//     cout << pid1.force << " \t" <<  pid1.P << " " << pid1.I << " " << pid1.D << "\n";
    FOREACHC(vector <OneAxisServo*>, kneeservos, s){//4-5
      sensors[n]   = (*s)->get();
      n++;
    }
    FOREACHC(vector <OneAxisServo*>, ankleservos, s){//6-7
      sensors[n]   = (*s)->get();
      n++;
    }
    FOREACHC(vector <TwoAxisServo*>, armservos, s){//8-11
      sensors[n]   = (*s)->get1();
      if(!conf.onlyPrimaryFunctions){
        n++;
        sensors[n]   = (*s)->get2();
      }
      n++;
    }
    FOREACHC(vector <OneAxisServo*>, arm1servos, s){//12-13
      sensors[n]   = (*s)->get();
      n++;
    }
    sensors[n] = pelvisservo->get(); // 14
    n++;
    if(conf.useBackJoint){            // 15 - 16
      FOREACHC(vector <OneAxisServo*>, backservos, s){
        sensors[n]   = (*s)->get();
        n++;
      }
    }

    //   FOREACHC(vector <OneAxisServo*>, headservos, s){

    //         sensors[n]   = (*s)->get();
    //         n++;
    //}
    FOREACHC(vector <TwoAxisServo*>, headservos, s){ // 17-18
      sensors[n]   = (*s)->get1();
      sensors[n+1]   = (*s)->get2();
      n+=2;
    }

   n += irSensorBank.get(sensors+n, sensornumber-n);

   //   // add z-headPosition as sensor and increment n!
      //   sensors[n++]=getHeadPosition().z;
     //    sensors[n++]=getTrunkPosition().z;

    assert(len==n);
    return n;
  };


  void Skeleton::placeIntern(const Matrix& pose){
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
    irSensorBank.update();
  };


  /** this function is called in each timestep. It should perform robot-internal checks,
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
  */
  void Skeleton::doInternalStuff(GlobalData& global){
    irSensorBank.reset();
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
    BallJoint*  bj;
    OneAxisServo* servo1;
    TwoAxisServo* servo2;
    Primitive* b ;
    //    AngularMotor* f;

    objects.clear();
    objects.resize(LastPart);

    // this is taken from DANCE, therefore the body creation is rather static
    // body creation
    // Hip
    b = new Box(0.2,0.1,0.1);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle.changeColor(conf.handColor));
    b->setPose(osg::Matrix::translate(0, 1.131, 0.0052) * pose );
//    b->setMass(/*16*/.61, 0, 0, 0, 0.0996, 0.1284, 0.1882, 0, 0, 0);
    b->setMass(.5*conf.massfactor);
    objects[Hip]=b;

    // Pole1
 //    b = new Box(.4,.3,.4);
//     b->init(odeHandle, 1,osgHandle);
//     b->setPose(osg::Matrix::translate(0, 1.4, -.3) * pose );
//       b->setMass(0.001*16.61, 0, 0, 0, 0.0996, 0.1284, 0.1882, 0, 0, 0);
//       //  b->setMass(0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0);
//     objects[Pole]=b;
//      // Pole2
//     b = new Box(1.45,.9,.15);
//     b->init(odeHandle, 1,osgHandle);
//     b->setPose(osg::Matrix::translate(0, 1.4, 1.3) * pose );
//       b->setMass(0.001*16.61, 0, 0, 0, 0.0996, 0.1284, 0.1882, 0, 0, 0);
//       //  b->setMass(0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0);
//     objects[Pole2]=b;

    // Trunk_comp
    b = new Box(0.3,0.168,.19);
    //    b = new Box(0.3,0.45,.2);
    b->setTexture(conf.trunkTexture);
    b->init(odeHandle, 1,osgHandle.changeColor(conf.trunkColor));
    b->setPose(osg::Matrix::translate(0, 1.177, 0.0201) * pose );
    //    b->setPose(osg::Matrix::translate(0, 1.39785, 0.0201) * pose );
//     b->setMass(/*29*/.27, 0, 0, 0, 0.498, 0.285, 0.568, 0, 0, 0);
    b->setMass(.12*conf.massfactor);//.3
   //  b = new Capsule(0.3,0.2);
//     b->init(odeHandle, 1,osgHandle);
//     b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0, 1.6884, 0.0253) * pose );
//     b->setMass(.1/*1*/, 0, 0, 0, 0.0003125, 0.0003125, 0.0003125, 0, 0, 0);
    objects[Trunk_comp]=b;

    // Belly
    //b = new Mesh("Meshes/skeleton/Trunk_comp_center.wrl",1);
    b = new Box(0.3,0.14,.19);
    //    b = new Box(0.3,0.45,.2);
    b->setTexture(conf.trunkTexture);
    b->init(odeHandle, 1,osgHandle.changeColor(conf.trunkColor));
    b->setPose(osg::Matrix::translate(0, 1.33, 0.0201) * pose );
    //    b->setPose(osg::Matrix::translate(0, 1.39785, 0.0201) * pose );
//     b->setMass(/*29*/.27, 0, 0, 0, 0.498, 0.285, 0.568, 0, 0, 0);
    b->setMass(.12*conf.massfactor);//.3
   //  b = new Capsule(0.3,0.2);
//     b->init(odeHandle, 1,osgHandle);
//     b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0, 1.6884, 0.0253) * pose );
//     b->setMass(.1/*1*/, 0, 0, 0, 0.0003125, 0.0003125, 0.0003125, 0, 0, 0);
    objects[Belly]=b;

    // Thorax
    b = new Box(0.33,0.33,0.21); //.235);
    b->setTexture(conf.trunkTexture);
    b->init(odeHandle, 1,osgHandle.changeColor(conf.trunkColor));
    b->setPose(osg::Matrix::translate(0, 1.50, 0.03/*0.035*/) * pose );
    b->setMass(1.0*conf.massfactor);//.3
    objects[Thorax]=b;

    double headsize=0.1;

    //  Neck
    b = new Capsule(0.05,0.03+headsize);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle.changeColor(conf.bodyColor));
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0, 1.6884+headsize/2, 0.0253) * pose );
//     b->setMass(.1/*1*/, 0, 0, 0, 0.0003125, 0.0003125, 0.0003125, 0, 0, 0);
    b->setMass(.05*conf.massfactor);//.01
    objects[Neck]=b;


    // Head_comp
    b = new Sphere(headsize);
    b->setTexture(conf.headTexture);
    // b->setPose(osg::Matrix::translate(0, 1.79, 0.063) * pose );
    //    b->init(odeHandle, 1,osgHandle);
    // b->setMass(5.89, 0, 0, 0, 0.0413, 0.0306, 0.0329, 0, 0, 0);
//     b->setMass(.1, 0, 0, 0, 0.0413, 0.0306, 0.0329, 0, 0, 0);
//    b->setMass(0.03*conf.massfactor);
//    b->setColor(conf.headColor);
    objects[Head_comp]=b;

    // Connect Head and Neck
    Transform* t = new Transform(objects[Neck], objects[Head_comp],
                                 osg::Matrix::translate(0, 0, -(.05)));
    t->init(odeHandle, 1,osgHandle);
    objects[Head_trans] = t;
    irSensorBank.init(odeHandle, osgHandle);
    if(conf.irSensors){
      // add Eyes ;-)
      RaySensor* sensor = new IRSensor(1,0.02);
      Matrix R = Matrix::translate(0,0,headsize) * Matrix::rotate(M_PI/10, 0, 1, 0) *
        Matrix::translate(0,headsize/10,0);
      irSensorBank.registerSensor(sensor, objects[Head_comp], R, 1.0, RaySensor::drawAll);
      sensor = new IRSensor(1,0.02);
      R = Matrix::translate(0,0,headsize) * Matrix::rotate(-M_PI/10, 0, 1, 0)*
        Matrix::translate(0,headsize/10,0);
      irSensorBank.registerSensor(sensor, objects[Head_comp], R, 1.0, RaySensor::drawAll);
    }




    // Left_Shoulder
    b = new Capsule(0.04,0.28);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(0.3094, 1.587, 0.0227) * pose );
    b->setColor(conf.trunkColor);
//     b->setMass(/*2*/.79, 0, 0, 0, 0.00056, 0.021, 0.021, 0, 0, 0);
    b->setMass(0.2*conf.massfactor);
    objects[Left_Shoulder]=b;

    // Left_Forearm
    b = new Capsule(0.035,0.28);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(0.5798, 1.5909, 0.024) * pose );
    b->setColor(conf.bodyColor);
//     b->setMass(1.21, 0, 0, 0, 0.00055, 0.0076, 0.0076, 0, 0, 0);
    b->setMass(0.121*conf.massfactor);
    objects[Left_Forearm]=b;

    // Left_Hand
    // b = new Cylinder(0.06,0.05);
    b = new Sphere(0.07);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0.7826, 1.5948, 0.024) * pose );
    b->setColor(conf.handColor);
//     b->setMass(0.55, 0, 0, 0, 0.00053, 0.047, 0.0016, 0, 0, 0);
    b->setMass(0.1*conf.massfactor*conf.relArmmass);
    objects[Left_Hand]=b;

    // Right_Shoulder
    b = new Capsule(0.04,0.28);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(-0.3094, 1.587, 0.0227) * pose );
//     b->setMass(/*2*/.79, 0, 0, 0, 0.00056, 0.021, 0.021, 0, 0, 0);
    b->setMass(0.2*conf.massfactor);
    b->setColor(conf.trunkColor);
    objects[Right_Shoulder]=b;

    // Right_Forearm
    b = new Capsule(0.035,0.28);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(-0.5798, 1.5909, 0.024) * pose );
    b->setColor(conf.bodyColor);
//     b->setMass(1.21, 0, 0, 0, 0.00055, 0.0076, 0.0076, 0, 0, 0);
    b->setMass(0.121*conf.massfactor);
    objects[Right_Forearm]=b;

    // Right_Hand
    // b = new Cylinder(0.06,0.05);
    b = new Sphere(0.07);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(-0.7826, 1.5948, 0.024) * pose );
    b->setColor(conf.handColor);
//     b->setMass(0.55, 0, 0, 0, 0.00053, 0.047, 0.0016, 0, 0, 0);
    b->setMass(.1*conf.massfactor*conf.relArmmass);
    objects[Right_Hand]=b;

    // Left_Thigh
    b = new Capsule(0.07,0.43);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0)* osg::Matrix::rotate(-M_PI/60,0,0,1) *
               osg::Matrix::translate(0.0949, 0.8525, 0.0253) * pose );
    b->setColor(conf.handColor);
//     b->setMass(8.35, 0, 0, 0, 0.145, 0.0085, 0.145, 0, 0, 0);
    b->setMass(.5*conf.massfactor);
    objects[Left_Thigh]=b;

    // Left_Shin
    b = new Capsule(0.06,0.35);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0.0702, 0.3988, 0.0357) * pose );
    b->setColor(conf.bodyColor);
    //    b->setMass(4.16, 0, 0, 0, 0.069, 0.0033, 0.069, 0, 0, 0);
    b->setMass(0.5*conf.massfactor);
    objects[Left_Shin]=b;

    // Left_Foot
    b = new Box(0.1,0.05,.3);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1, osgHandle.changeColor(conf.trunkColor));
    b->setPose(osg::Matrix::translate(0.0624, 0.1388, 0.0708) * pose );
    //    b->setMass(1.34, 0, 0, 0, 0.0056, 0.0056, 0.00036, 0, 0, 0);
    b->setMass(.5*conf.massfactor*conf.relFeetmass);
    objects[Left_Foot]=b;

    // Right_Thigh
    b = new Capsule(0.07,0.43);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0)* osg::Matrix::rotate(M_PI/60,0,0,1) *
               osg::Matrix::translate(-0.0949, 0.8525, 0.0253) * pose );
    b->setColor(conf.handColor);
    //    b->setMass(8.35, 0, 0, 0, 0.145, 0.0085, 0.145, 0, 0, 0);
    b->setMass(.5*conf.massfactor);
    objects[Right_Thigh]=b;

    // Right_Shin
    b = new Capsule(0.06,0.35);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(-0.0702, 0.3988, 0.0357) * pose );
    b->setColor(conf.bodyColor);
    //    b->setMass(4.16, 0, 0, 0, 0.069, 0.0033, 0.069, 0, 0, 0);
    b->setMass(0.5*conf.massfactor);
    objects[Right_Shin]=b;

    // Right_Foot
    b = new Box(0.1,0.05,.3);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1, osgHandle.changeColor(conf.trunkColor));
    b->setPose(osg::Matrix::translate(-0.0624, 0.1388, 0.0708) * pose );
    //    b->setMass(1.34, 0, 0, 0, 0.0056, 0.0056, 0.00036, 0, 0, 0);
    b->setMass(0.5*conf.massfactor*conf.relFeetmass);
    objects[Right_Foot]=b;


    // joint creation
    // Hip and Trunk_comp
    // j = new BallJoint(objects[Hip], objects[Trunk_comp],
    //                   Pos(0, 1.2516, 0.0552) * pose, Axis(0,0,1) * pose);
    //    uj = new UniversalJoint(objects[Hip], objects[Belly], Pos(0, 1.2, 0.0252) * pose,
    //                           Axis(0,0,1) * pose, Axis(0,1,0) * pose);
    j = new HingeJoint(objects[Hip], objects[Trunk_comp], Pos(0, 1.15, 0.02052) * pose, Axis(0,0,1) * pose);
    j->init(odeHandle, osgHandleJ, true, .205);
    joints.push_back(j);

    if(conf.useVelocityServos){
      pelvisservo = new OneAxisServoVel(odeHandle, j,
                                        -conf.pelvisJointLimit, conf.pelvisJointLimit, conf.pelvisPower,
                                        conf.pelvisDamping, conf.pelvisVelocity, conf.jointLimitFactor);
    } else {
      pelvisservo = new OneAxisServoCentered(j, -conf.pelvisJointLimit, conf.pelvisJointLimit, conf.pelvisPower,
                                             conf.pelvisDamping, 2, 20, conf.jointLimitFactor);
    }

    // Trunk_comp, Belly and Thorax
    if(conf.useBackJoint){

      j = new HingeJoint(objects[Belly], objects[Thorax],
                         (objects[Belly]->getPosition() + objects[Thorax]->getPosition())/2,
                         Axis(-1,0,0) * pose);
      j->init(odeHandle, osgHandleJ, true, 0.36);
      joints.push_back(j);

      if(conf.useVelocityServos)
        servo1 = new OneAxisServoVel(odeHandle, j, -conf.backJointLimit/2, conf.backJointLimit,
                                     conf.backPower, conf.backDamping, conf.backVelocity, conf.jointLimitFactor);
      else
        servo1 = new OneAxisServoCentered(j, -conf.backJointLimit/2, conf.backJointLimit,
                                          conf.backPower, conf.backDamping, 2, 20, conf.jointLimitFactor);
      backservos.push_back(servo1);

      j = new HingeJoint(objects[Trunk_comp], objects[Belly],
                         (objects[Trunk_comp]->getPosition() + objects[Belly]->getPosition())/2,
                         Axis(0,1,0) * pose);
      j->init(odeHandle, osgHandleJ, true, 0.2);
      joints.push_back(j);
      if(conf.useVelocityServos)
        servo1 = new OneAxisServoVel(odeHandle, j, -conf.backJointLimit, conf.backJointLimit,
                                        conf.backPower, conf.backDamping, conf.backVelocity, conf.jointLimitFactor);
      else
        servo1 = new OneAxisServoCentered(j, -conf.backJointLimit, conf.backJointLimit,
                                             conf.backPower, conf.backDamping, 2, 20, conf.jointLimitFactor);
      backservos.push_back(servo1);
    }else{
      fj = new FixedJoint(objects[Thorax], objects[Trunk_comp]);
      fj->init(odeHandle, osgHandleJ, false, 0.);
      joints.push_back(fj);
      fj = new FixedJoint(objects[Belly], objects[Trunk_comp]);
      fj->init(odeHandle, osgHandleJ, false, 0.);
      joints.push_back(fj);
    }


    // Pole and Trunk
    // j = new BallJoint(objects[Hip], objects[Trunk_comp], Pos(0, 1.2516, 0.0552) * pose, Axis(0,0,1) * pose);
     // uj = new UniversalJoint(objects[Pole], objects[Trunk_comp], Pos(0, 1.2516, 0.0552) * pose,
//                             Axis(0,0,1) * pose, Axis(0,1,0) * pose);
//      uj->init(odeHandle, osgHandleJ, false, 0.2);
//        joints.push_back(uj);

//     fj = new FixedJoint(objects[Pole], objects[Trunk_comp]);
//     fj->init(odeHandle, osgHandleJ, true, 0.12);
//     joints.push_back(fj);

//     fj = new FixedJoint(objects[Pole2], objects[Trunk_comp]);
//     fj->init(odeHandle, osgHandleJ, true, 0.12);
//     joints.push_back(fj);


    //   Neck and Thorax
    if(conf.movableHead){
      uj = new UniversalJoint(objects[Thorax], objects[Neck], Pos(0, 1.6442, 0.0188) * pose,
                              Axis(0,0,1) * pose, Axis(1,0,0) * pose);
      uj->init(odeHandle, osgHandleJ, true, 0.12);
      joints.push_back(uj);

      servo2 = conf.useVelocityServos
        ? new TwoAxisServoVel(odeHandle, uj, -conf.neckJointLimit, conf.neckJointLimit, conf.neckPower,
                              -conf.neckJointLimit, conf.neckJointLimit, conf.neckPower, conf.neckDamping,
                              conf.neckVelocity, conf.jointLimitFactor)
        : new TwoAxisServoCentered(uj, -conf.neckJointLimit, conf.neckJointLimit, conf.neckPower,
                                   -conf.neckJointLimit, conf.neckJointLimit, conf.neckPower,
                                   conf.neckDamping, 1, 20, conf.jointLimitFactor);
      headservos.push_back(servo2);
//     f = new AngularMotor2Axis(odeHandle, uj, conf.neckDamping, conf.neckDamping);
//     frictionmotors.push_back(f);
    }else{
      // to fix the head!
      fj = new FixedJoint(objects[Thorax], objects[Neck]);
      fj->init(odeHandle, osgHandleJ, false, 0.12);
      joints.push_back(fj);
    }

    // Head and Neck (substituted by transform)
//     fj = new FixedJoint(objects[Neck], objects[Head_comp]); // ,Pos(0, 1.7326, 0.0318) * pose);
//     fj->init(odeHandle, osgHandleJ, false);
//     joints.push_back(fj);

    // Thorax and Shoulders (Arms)
    uj = new UniversalJoint(objects[Thorax], objects[Left_Shoulder],
                            Pos(0.1768, 1.587, 0.0214) * pose,
                            Axis(0,0,1) * pose, Axis(0,1,0) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.12);
    joints.push_back(uj);

    servo2 =  conf.useVelocityServos
      ? new TwoAxisServoVel(odeHandle, uj, -conf.armJointLimit*.2, conf.armJointLimit, conf.armPower,
                            -conf.armJointLimit*.2, conf.armJointLimit, conf.armPower,
                            conf.armDamping, conf.armVelocity, conf.jointLimitFactor)
      : new TwoAxisServoCentered(uj, -conf.armJointLimit*.2, conf.armJointLimit, conf.armPower,
                                 -conf.armJointLimit*.2, conf.armJointLimit, conf.armPower,
                                 conf.armDamping, 2, 20, conf.jointLimitFactor);
    armservos.push_back(servo2);

    uj = new UniversalJoint(objects[Thorax], objects[Right_Shoulder],
                            Pos(-0.1768, 1.587, 0.0214) * pose,
                            Axis(0,0,-1) * pose, Axis(0,-1,0) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.12);
    joints.push_back(uj);

    if(conf.useVelocityServos)
      servo2 = new TwoAxisServoVel(odeHandle, uj, -conf.armJointLimit*.1, conf.armJointLimit, conf.armPower,
                                   -conf.armJointLimit*.1, conf.armJointLimit, conf.armPower,
                                   conf.armDamping, conf.armVelocity, conf.jointLimitFactor);
    else
      servo2 = new TwoAxisServoCentered(uj, -conf.armJointLimit*.1, conf.armJointLimit, conf.armPower,
                                        -conf.armJointLimit*.1, conf.armJointLimit, conf.armPower,
                                        conf.armDamping, 2, 20, conf.jointLimitFactor);
    armservos.push_back(servo2);

    // Arms and ForeArms

    // Fixed
   //  fj = new FixedJoint(objects[Left_Shoulder], objects[Left_Forearm]); // ,Pos(0.442, 1.587, 0.024) * pose);
//     fj->init(odeHandle, osgHandleJ, false);
//     joints.push_back(fj);
   //  fj = new FixedJoint(objects[Right_Shoulder], objects[Right_Forearm]); // ,Pos(-0.442, 1.587, 0.024) * pose);
//     fj->init(odeHandle, osgHandleJ, false);
//     joints.push_back(fj);

    j = new HingeJoint(objects[Left_Shoulder], objects[Left_Forearm],Pos(0.442, 1.587, 0.024) * pose,
                       Axis(0,1,0) * pose); // ,Pos(0.442, 1.587, 0.024) * pose);
    j->init(odeHandle, osgHandleJ, false);
    joints.push_back(j);
    //  servo1 = new OneAxisServo(j, -M_PI/10, M_PI/10, 20,0.1);
    if(conf.useVelocityServos)
      servo1 = new OneAxisServoVel(odeHandle, j, 0, conf.elbowJointLimit,
                                   conf.elbowPower, conf.elbowDamping, conf.elbowVelocity, conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -conf.elbowJointLimit*.2, conf.elbowJointLimit, conf.elbowPower, conf.elbowDamping,
                                        2, 20, conf.jointLimitFactor);

    arm1servos.push_back(servo1);

    j = new HingeJoint(objects[Right_Shoulder], objects[Right_Forearm],
                       Pos(-0.442, 1.587, 0.024) * pose,  Axis(0,-1,0) * pose);
    j->init(odeHandle, osgHandleJ, false);
    joints.push_back(j);
    // servo1 = new OneAxisServo(j, -M_PI/10, M_PI/10, 20,0.1);
    if(conf.useVelocityServos)
      servo1 = new OneAxisServoVel(odeHandle, j, 0, conf.elbowJointLimit,
                                   conf.elbowPower, conf.elbowDamping, conf.elbowVelocity, conf.jointLimitFactor);
    else
      servo1 = new  OneAxisServoCentered( j, -conf.elbowJointLimit*.2, conf.elbowJointLimit, conf.elbowPower, conf.elbowDamping,
                                          2, 20, conf.jointLimitFactor);

    arm1servos.push_back(servo1);

    if(conf.handsRotating){
      bj = new BallJoint(objects[Left_Forearm], objects[Left_Hand], objects[Left_Hand]->getPosition()); // ,Pos(0.7176, 1.5948, 0.024) * pose);
      bj->init(odeHandle, osgHandleJ, false);
      joints.push_back(bj);
      bj = new BallJoint(objects[Right_Forearm], objects[Right_Hand], objects[Right_Hand]->getPosition()); // ,Pos(-0.7176, 1.5948, 0.024) * pose);
      bj->init(odeHandle, osgHandleJ, false);
      joints.push_back(bj);
    }else{
      fj = new FixedJoint(objects[Left_Forearm], objects[Left_Hand]); // ,Pos(0.7176, 1.5948, 0.024) * pose);
      fj->init(odeHandle, osgHandleJ, false);
      joints.push_back(fj);
      fj = new FixedJoint(objects[Right_Forearm], objects[Right_Hand]); // ,Pos(-0.7176, 1.5948, 0.024) * pose);
      fj->init(odeHandle, osgHandleJ, false);
      joints.push_back(fj);
    }

    // TODO: substitute the servos with Muscles
    // Hip and Thighs
    uj = new UniversalJoint(objects[Hip], objects[Left_Thigh],
                            Pos(0.1118, 1.0904, 0.011) * pose,
                            Axis(1,0,0) * pose, Axis(0,0,-1) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.15);
    joints.push_back(uj);

    servo2 = conf.useVelocityServos
      ? new TwoAxisServoVel(odeHandle, uj, -conf.hipJointLimit*.4, conf.hipJointLimit, conf.hipPower,
                            -conf.hip2JointLimit*.4, conf.hip2JointLimit, conf.hip2Power,
                            conf.hipDamping, conf.hipVelocity, conf.jointLimitFactor)
      : new TwoAxisServoCentered(uj, -conf.hipJointLimit*.4,conf.hipJointLimit, conf.hipPower,
                                 -conf.hip2JointLimit*.4, conf.hip2JointLimit, conf.hip2Power, conf.hipDamping,
                                 2, 20, conf.jointLimitFactor);
    servo2->setDamping2(conf.hip2Damping);
    hipservos.push_back(servo2);

    uj = new UniversalJoint(objects[Hip], objects[Right_Thigh], Pos(-0.1118, 1.0904, 0.011) * pose,
                           Axis(1,0,0) * pose, Axis(0,0,1) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.15);
    joints.push_back(uj);

    servo2 =  conf.useVelocityServos
      ? new TwoAxisServoVel(odeHandle, uj, -conf.hipJointLimit*.2, conf.hipJointLimit, conf.hipPower,
                            -conf.hipJointLimit*.2, conf.hipJointLimit, conf.hipPower,
                            conf.hipDamping, conf.hipVelocity, conf.jointLimitFactor)
      : new TwoAxisServoCentered(uj, -conf.hipJointLimit*.2, conf.hipJointLimit, conf.hipPower,
                                 -conf.hip2JointLimit*.4, conf.hip2JointLimit, conf.hip2Power, conf.hipDamping,
                                 2, 20, conf.jointLimitFactor);
    servo2->setDamping2(conf.hip2Damping);
    hipservos.push_back(servo2);


    // Thighs and Shins (Knees)
    j = new HingeJoint(objects[Left_Thigh], objects[Left_Shin], Pos(0.078, 0.6146, 0.0396) * pose,
                       Axis(2,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true, 0.15);
    joints.push_back(j);


    if( conf.useVelocityServos )
      servo1 = new OneAxisServoVel(odeHandle, j, -conf.kneeJointLimit , conf.kneeJointLimit * .2,
                                   conf.kneePower, conf.kneeDamping, conf.kneeVelocity, conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -conf.kneeJointLimit, conf.kneeJointLimit * 0.2,
                                        conf.kneePower, conf.kneeDamping, 2, 20, conf.jointLimitFactor);
    kneeservos.push_back(servo1);

    j = new HingeJoint(objects[Right_Thigh], objects[Right_Shin], Pos(-0.078, 0.6146, 0.0396) * pose,
                       Axis(2,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true, 0.15);
    joints.push_back(j);

    if( conf.useVelocityServos )
      servo1 = new OneAxisServoVel(odeHandle, j, -conf.kneeJointLimit , conf.kneeJointLimit * .2,
                                   conf.kneePower, conf.kneeDamping, conf.kneeVelocity, conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -conf.kneeJointLimit, conf.kneeJointLimit  * .2,
                                        conf.kneePower, conf.kneeDamping, 2, 20, conf.jointLimitFactor);
    kneeservos.push_back(servo1);

    // Shins and Feet (Ankles)
    // fj = new FixedJoint(objects[Left_Shin], objects[Left_Foot]);
    j = new HingeJoint(objects[Left_Shin], objects[Left_Foot],
                       Pos(0.0624, 0.183, 0.0318) * pose,
                        Axis(1,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true,0.1);
    joints.push_back(j);

    if( conf.useVelocityServos )
      servo1 = new OneAxisServoVel(odeHandle, j, -conf.ankleJointLimit , conf.ankleJointLimit,
                                   conf.anklePower, conf.ankleDamping, conf.ankleVelocity, conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -conf.ankleJointLimit, conf.ankleJointLimit,
                                        conf.anklePower, conf.ankleDamping, 2, 20, conf.jointLimitFactor);
    ankleservos.push_back(servo1);

    j = new HingeJoint(objects[Right_Shin], objects[Right_Foot],
                       Pos(-0.0624, 0.183, 0.0318) * pose,
                        Axis(1,0,0) * pose);
    //  fj = new FixedJoint(objects[Right_Shin], objects[Right_Foot]);
    j->init(odeHandle, osgHandleJ, true, 0.1);
    joints.push_back(j);

    if( conf.useVelocityServos )
      servo1 =  new OneAxisServoVel(odeHandle, j, -conf.ankleJointLimit , conf.ankleJointLimit,
                                    conf.anklePower, conf.ankleDamping, conf.ankleVelocity, conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -conf.ankleJointLimit, conf.ankleJointLimit,
                                        conf.anklePower, conf.ankleDamping, 2, 20, conf.jointLimitFactor);
    ankleservos.push_back(servo1);

    // register ignored pairs
    odeHandle.addIgnoredPair(objects[Head_comp],objects[Thorax]);
    odeHandle.addIgnoredPair(objects[Head_trans],objects[Thorax]);
    odeHandle.addIgnoredPair(objects[Trunk_comp],objects[Thorax]);
    odeHandle.addIgnoredPair(objects[Left_Thigh],objects[Trunk_comp]);
    odeHandle.addIgnoredPair(objects[Right_Thigh],objects[Trunk_comp]);
    odeHandle.addIgnoredPair(objects[Left_Thigh],objects[Belly]);
    odeHandle.addIgnoredPair(objects[Right_Thigh],objects[Belly]);
    odeHandle.addIgnoredPair(objects[Right_Thigh],objects[Hip]);
    odeHandle.addIgnoredPair(objects[Left_Thigh],objects[Hip]);
    odeHandle.addIgnoredPair(objects[Left_Shin],objects[Trunk_comp]);
    odeHandle.addIgnoredPair(objects[Right_Shin],objects[Trunk_comp]);
    odeHandle.addIgnoredPair(objects[Left_Shin],objects[Belly]);
    odeHandle.addIgnoredPair(objects[Right_Shin],objects[Belly]);
    odeHandle.addIgnoredPair(objects[Left_Shin],objects[Thorax]);
    odeHandle.addIgnoredPair(objects[Right_Shin],objects[Thorax]);

    // odeHandle.addIgnoredPair(objects[Left_Thigh],objects[Right_Thigh]);
    //  odeHandle.addIgnoredPair(objects[Left_Shin],objects[Right_Shin]);
   // odeHandle.addIgnoredPair(objects[Left_Foot],objects[Right_Foot]);

    // we call setParam in order to set all the dampings and default values
    setParam("thisparamdoesnotexist",0);

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
//       FOREACH(vector<OneAxisServo*>, headservos, i){
    //   FOREACH(vector<TwoAxisServo*>, headservos, i){
//         if(*i) delete *i;
//       }
      FOREACH(vector<TwoAxisServo*>, armservos, i){
        if(*i) delete *i;
      }
      //      headservos.clear();

      if(pelvisservo) delete pelvisservo;
      FOREACH(vector<OneAxisServo*>, backservos, i){
        if(*i) delete *i;
      }

      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
        if(*i) delete *i;
      }
      joints.clear();

      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
        if(*i) delete *i;
      }
      objects.clear();
      irSensorBank.clear();
      odeHandle.removeSpace(odeHandle.space);
      dSpaceDestroy(odeHandle.space);
    }

    created=false;
  }


  bool Skeleton::setParam(const paramkey& key, paramval val, bool traverseChildren){
    bool rv = Configurable::setParam(key, val);

    // we just set all parameters independend of what was actually changed
    FOREACH(vector<TwoAxisServo*>, hipservos, i){
      if(*i) {
        (*i)->setPower( conf.hipPower, conf.hip2Power);
        (*i)->setDamping1(conf.hipDamping);
        (*i)->setDamping2(conf.hip2Damping);
        (*i)->setMaxVel(conf.hipVelocity);
        (*i)->setMinMax1(-conf.hipJointLimit*.2, +conf.hipJointLimit);
        (*i)->setMinMax2(-conf.hip2JointLimit*.4,+conf.hip2JointLimit);

      }
    }
    FOREACH(vector<TwoAxisServo*>, headservos, i){
      if(*i){
        (*i)->setPower(conf.neckPower, conf.neckPower);
        (*i)->setDamping1(conf.neckDamping);
        (*i)->setDamping2(conf.neckDamping);
        (*i)->setMaxVel(conf.neckVelocity);
        (*i)->setMinMax1(-conf.neckJointLimit, conf.neckJointLimit);
        (*i)->setMinMax2(-conf.neckJointLimit, conf.neckJointLimit);
      }
    }
    FOREACH(vector<OneAxisServo*>, kneeservos, i){
      if(*i){
        (*i)->setPower(conf.kneePower);
        (*i)->setDamping(conf.kneeDamping);
        (*i)->setMaxVel(conf.kneeVelocity);
        (*i)->setMinMax(-conf.kneeJointLimit, conf.kneeJointLimit*0.1);
      }
    }
    FOREACH(vector<OneAxisServo*>, ankleservos, i){
      if(*i){
        (*i)->setPower(conf.anklePower);
        (*i)->setDamping(conf.ankleDamping);
        (*i)->setMaxVel(conf.ankleVelocity);
        (*i)->setMinMax(-conf.ankleJointLimit, conf.ankleJointLimit*0.5);
      }
    }
    FOREACH(vector<TwoAxisServo*>, armservos, i){
      if(*i){
        (*i)->setPower(conf.armPower, conf.armPower);
        (*i)->setDamping1(conf.armDamping);
        (*i)->setDamping2(conf.armDamping);
        (*i)->setMaxVel(conf.armVelocity);
        (*i)->setMinMax1(-conf.armJointLimit, conf.armJointLimit);
        (*i)->setMinMax2(-conf.armJointLimit*.3, conf.armJointLimit);
      }
    }
    FOREACH(vector<OneAxisServo*>, arm1servos, i){
      if(*i){
        (*i)->setPower(conf.elbowPower);
        (*i)->setDamping(conf.elbowDamping);
        (*i)->setMaxVel(conf.elbowVelocity);
        (*i)->setMinMax(0, conf.elbowJointLimit);
      }
    }

    pelvisservo->setPower(conf.pelvisPower);
    pelvisservo->setDamping(conf.pelvisDamping);
    pelvisservo->setMaxVel(conf.pelvisVelocity);
    pelvisservo->setMinMax(-conf.pelvisJointLimit,+conf.pelvisJointLimit);

    int fst = true;
    FOREACH(vector<OneAxisServo*>, backservos, i){
      if(*i){
        (*i)->setPower(conf.backPower);
        (*i)->setDamping(conf.backDamping);
        (*i)->setMaxVel(conf.backVelocity);
        (*i)->setMinMax(fst? -conf.backJointLimit : -conf.backJointLimit,
                        conf.backJointLimit);
        fst = false;
      }
    }
    return rv;
  }

  Position Skeleton::getHeadPosition() {
    const Primitive* o = objects[Head_comp];
    // using the Geom has maybe the advantage to get the position of transform objects
    // (e.g. hand of muscledArm)
    if (o && o->getGeom())
      return Position(dGeomGetPosition(o->getGeom()));
    else if(o->getBody())
      return Position(dBodyGetPosition(o->getBody()));
    else return Position(0,0,0);
  }

  Position Skeleton::getTrunkPosition() {
    const Primitive* o = objects[Trunk_comp];
    // using the Geom has maybe the advantage to get the position of transform objects
    // (e.g. hand of muscledArm)
    if (o && o->getGeom())
      return Position(dGeomGetPosition(o->getGeom()));
    else if(o->getBody())
      return Position(dBodyGetPosition(o->getBody()));
    else return Position(0,0,0);
  }


}
