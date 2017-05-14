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
#include <ode_robots/twoaxisservo.h>
#include <ode_robots/angularmotor.h>
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
    : OdeRobot(odeHandle, osgHandle, name, "1.2"), Inspectable(name), conf(c)
  {
    // robot is not created till now
    created=false;

    //    this->osgHandle.color = Color(1.0, 1,1,1);
    // choose color here a pastel white is used
    //    this->osgHandle.color = Color(217/255.0, 209/255.0, 109/255.0, 1.0f);

    addParameter("powerfactor",    &conf.powerFactor,0,10, "global power factor for all motor");
    addParameter("relforce",    &conf.relForce,0,10, "factor for arm force");

    if(conf.useVelocityServos){
      addParameter("stiffnessfactor",  &conf.dampingFactor,0,100,
                   "global stiffness factor for all motor (0-100)");
    }else{
      addParameter("dampingfactor",  &conf.dampingFactor,0,10,
                   "global damping factor for all motor (0-10)");
    }

    if(!conf.onlyMainParameters){
      addParameter("hippower",   &conf.hipPower                   ,0,10);
      addParameter("hipdamping",   &conf.hipDamping               ,0,10);
      addParameter("hipvelocity", &conf.hipVelocity               ,0,10);
      addParameter("hipjointlimit",   &conf.hipJointLimit         ,0,10);
      addParameter("hip2power",   &conf.hip2Power                 ,0,10);
      addParameter("hip2damping",   &conf.hip2Damping             ,0,10);
      addParameter("hip2jointlimit",   &conf.hip2JointLimit       ,0,10);
      addParameter("neckpower",   &conf.neckPower                 ,0,10);
      addParameter("neckdamping",   &conf.neckDamping             ,0,10);
      addParameter("neckvelocity",   &conf.neckVelocity           ,0,10);
      addParameter("neckjointlimit",   &conf.neckJointLimit       ,0,10);
      addParameter("kneepower",   &conf.kneePower                 ,0,10);
      addParameter("kneedamping",   &conf.kneeDamping             ,0,10);
      addParameter("kneevelocity",   &conf.kneeVelocity           ,0,10);
      addParameter("kneejointlimit",   &conf.kneeJointLimit       ,0,10);
      addParameter("anklepower",   &conf.anklePower               ,0,10);
      addParameter("ankledamping",   &conf.ankleDamping           ,0,10);
      addParameter("anklevelocity",   &conf.ankleVelocity         ,0,10);
      addParameter("anklejointlimit",   &conf.ankleJointLimit     ,0,10);
      addParameter("armpower",   &conf.armPower                   ,0,10);
      addParameter("armdamping",   &conf.armDamping               ,0,10);
      addParameter("armvelocity",   &conf.armVelocity             ,0,10);
      addParameter("armjointlimit",   &conf.armJointLimit         ,0,10);
      addParameter("elbowpower",   &conf.elbowPower               ,0,10);
      addParameter("elbowdamping",   &conf.elbowDamping           ,0,10);
      addParameter("elbowvelocity",   &conf.elbowVelocity         ,0,10);
      addParameter("elbowjointlimit",   &conf.elbowJointLimit     ,0,10);
      addParameter("pelvispower",   &conf.pelvisPower             ,0,10);
      addParameter("pelvisdamping",   &conf.pelvisDamping         ,0,10);
      addParameter("pelvisvelocity",   &conf.pelvisVelocity       ,0,10);
      addParameter("pelvisjointlimit",   &conf.pelvisJointLimit   ,0,10);
      addParameter("backpower",   &conf.backPower                 ,0,10);
      addParameter("backdamping",   &conf.backDamping             ,0,10);
      addParameter("backvelocity",   &conf.backVelocity           ,0,10);
      addParameter("backjointlimit",   &conf.backJointLimit       ,0,10);
    }

    int n=0;
    addInspectableDescription("x[" + itos(n++) + "]","hip left sagital  (back-/front+)");
    if(!conf.onlyPrimaryFunctions)
      addInspectableDescription("x[" + itos(n++) + "]","hip left lateral  (in-/out+)");
    addInspectableDescription("x[" + itos(n++) + "]","hip right sagital (back-/front+)");
    if(!conf.onlyPrimaryFunctions)
      addInspectableDescription("x[" + itos(n++) + "]","hip right lateral (in-/out+)");

    addInspectableDescription("x[" + itos(n++) + "]","knee left (bend-/stretch+)");
    addInspectableDescription("x[" + itos(n++) + "]","knee right (bend-/stretch+)");

    addInspectableDescription("x[" + itos(n++) + "]","ankle left (bend-/stretch+)");
    addInspectableDescription("x[" + itos(n++) + "]","ankle right (bend-/stretch+)");

    addInspectableDescription("x[" + itos(n++) + "]","shoulder left lateral (up-/down+)");
    if(!conf.onlyPrimaryFunctions)
      addInspectableDescription("x[" + itos(n++) + "]","shoulder left sagital (out-/in+)");
    addInspectableDescription("x[" + itos(n++) + "]","shoulder right lateral (up-/down+)");
    if(!conf.onlyPrimaryFunctions)
      addInspectableDescription("x[" + itos(n++) + "]","shoulder right sagital (out-/in+)");

    addInspectableDescription("x[" + itos(n++) + "]","elbow left (bend-/stretch+)");
    addInspectableDescription("x[" + itos(n++) + "]","elbow right (bend-/stretch+)");

    if(!conf.onlyPrimaryFunctions)
      addInspectableDescription("x[" + itos(n++) + "]","pelvis (bend left-/right+)");
    if(conf.useBackJoint){
      if(conf.backSideBend && !conf.onlyPrimaryFunctions){
        addInspectableDescription("x[" + itos(n++) + "]","back (torsion)");
        addInspectableDescription("x[" + itos(n++) + "]","back (bend back-/front+)");
        addInspectableDescription("x[" + itos(n++) + "]","back (bend left-/right+)");
      }else{
        addInspectableDescription("x[" + itos(n++) + "]","back (bend back-/front+)");
        addInspectableDescription("x[" + itos(n++) + "]","back (torsion)");
      }
    }
    if(conf.movableHead && !conf.onlyPrimaryFunctions){
      addInspectableDescription("x[" + itos(n++) + "]","head (left-/right+)");
      addInspectableDescription("x[" + itos(n++) + "]","head (back-/front+)");

    }
  };

  int Skeleton::getMotorNumberIntern(){
    if(conf.onlyPrimaryFunctions)
      return hipservos.size() + kneeservos.size() + ankleservos.size() + armservos.size()+ 1/*pelvis*/ ;
    else
      return hipservos.size()*2 + kneeservos.size() + ankleservos.size() + armservos.size()*2 + arm1servos.size() +
        1/*pelvis*/+ backservos.size() + 2*backservos2.size() +2*headservos.size();
  };

  /* sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Skeleton::setMotorsIntern(const double* motors, int motornumber){
    assert(created); // robot must exist

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
        FOREACH(vector <TwoAxisServo*>, backservos2, s){
          (*s)->set(motors[n],motors[n+1]);
          n+=2;
        }
      }else{
        FOREACH(vector <OneAxisServo*>, backservos, s){
          (*s)->set(0);
        }
        FOREACH(vector <TwoAxisServo*>, backservos2, s){
          (*s)->set(0,0);
        }
      }
    }
    FOREACH(vector <TwoAxisServo*>, headservos, s){
      if(!conf.onlyPrimaryFunctions){
        (*s)->set(motors[n],motors[n+1]);
        n+=2;
      }else
        (*s)->set(0,0);
    }

    FOREACH(vector <AngularMotor*>, frictionmotors, s){
      // force both axis to have zero velocity
      (*s)->set(0, 0.0);
      (*s)->set(1, 0.0);
    }

    assert(min(motornumber, getMotorNumberIntern())==n);
  };

  int Skeleton::getSensorNumberIntern(){
    int numberSensors=0;

    if(conf.onlyPrimaryFunctions){
      numberSensors +=hipservos.size() + kneeservos.size() + ankleservos.size() +
        armservos.size() + arm1servos.size() + 1 /*pelvis*/;
    } else {
    //  return 1;
      numberSensors += hipservos.size()*2 + kneeservos.size() + ankleservos.size() +
        armservos.size()*2 + arm1servos.size() +
        1/*pelvis*/+ backservos.size() + 2*backservos2.size() + 2*headservos.size() ;
    }

//     // add new one!
//     // head and trunk position (z): +2
//     //    numberSensors+=2;

    return numberSensors;
  }

  /*****************************
GUIDE adding new sensors
1. in getSensorNumberIntern() Anzahl der Sensoren korrigieren: numberSensors+=1;
2. in getSensorsIntern().z;


   ****************************/

  /* returns actual sensorvaluess
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Skeleton::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created);
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
      if(conf.useBackJoint){            // 15 - 16 (or 15 (torsion))
        FOREACHC(vector <OneAxisServo*>, backservos, s){
          sensors[n]   = (*s)->get();
          n++;
        }
        FOREACHC(vector <TwoAxisServo*>, backservos2, s){ // (and 16 - 17 bend forward and sideways)
          sensors[n]   = (*s)->get1();
          sensors[n+1]   = (*s)->get2();
          n+=2;
        }
      }

      FOREACHC(vector <TwoAxisServo*>, headservos, s){ // 17-18
        sensors[n]   = (*s)->get1();
        sensors[n+1]   = (*s)->get2();
        n+=2;
      }
    //   // add z-headPosition as sensor and increment n!
    //   sensors[n++]=getHeadPosition().z;
    //    sensors[n++]=getTrunkPosition().z;

      matrix::Matrix x(n,1,sensors); // store sensor values
      assert(x.hasNormalEntries());


    assert(min(sensornumber, getSensorNumberIntern())==n);
    return n;
  };


  void Skeleton::placeIntern(const Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    //    Matrix p2;
    //    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8));
    create(pose);
  };


  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void Skeleton::create( const Matrix& pose ){
    if (created) {
      destroy();
    }

    // this marks constants that should not be changed at this position but
    // in notifyOnChange
    const double DONT = 1.0;

    odeHandle.createNewSimpleSpace(parentspace, false);
    osgHandle = osgHandle.changeColor(conf.bodyColor);
    OsgHandle osgHTrousers(osgHandle.changeColor(conf.trouserColor));
    OsgHandle osgHTrunk(osgHandle.changeColor(conf.trunkColor));


    OdeHandle ignoreColSpace(odeHandle);
    ignoreColSpace.createNewSimpleSpace(odeHandle.space, true);



    OsgHandle osgHandleJ = osgHandle.changeColor("joint");
    HingeJoint* j;
    UniversalJoint* uj;
    FixedJoint* fj;
    BallJoint*  bj;
    OneAxisServo* servo1;
    TwoAxisServo* servo2;
    Primitive* b ;
    //    AngularMotor* f;

    objects.clear();
    objects.resize(LastPart, 0);

    // this is taken from DANCE, therefore the body creation is rather static

    // body creation
    // Hip
    b = new Box(0.2,0.1,0.1);
    b->setTexture(conf.bodyTexture);
    b->init(ignoreColSpace, 1, osgHTrousers);
    b->setPose(osg::Matrix::translate(0, 1.131, 0.0052) * pose );
//    b->setMass(/*16*/.61, 0, 0, 0, 0.0996, 0.1284, 0.1882, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(.5*conf.massfactor);
    objects[Hip]=b;

    // Trunk_comp
    b = new Box(0.3,0.168,.19);
    //    b = new Box(0.3,0.45,.2);
    b->setTexture(conf.trunkTexture);
    b->init(ignoreColSpace, 1,osgHTrousers);
    b->setPose(osg::Matrix::translate(0, 1.177, 0.0201) * pose );
    //    b->setPose(osg::Matrix::translate(0, 1.39785, 0.0201) * pose );
//     b->setMass(/*29*/.27, 0, 0, 0, 0.498, 0.285, 0.568, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(.12*conf.massfactor);//.3
    objects[Trunk_comp]=b;

    // Belly
    //b = new Mesh("Meshes/skeleton/Trunk_comp_center.wrl",1);
    b = new Box(0.3,0.14,.19);
    //    b = new Box(0.3,0.45,.2);
    b->setTexture(conf.trunkTexture);
    b->init(ignoreColSpace, 1,osgHTrunk);
    b->setPose(osg::Matrix::translate(0, 1.33, 0.0201) * pose );
    //    b->setPose(osg::Matrix::translate(0, 1.39785, 0.0201) * pose );
//     b->setMass(/*29*/.27, 0, 0, 0, 0.498, 0.285, 0.568, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(.12*conf.massfactor);//.3
    objects[Belly]=b;

    // Thorax
    b = new Box(0.33,0.33,0.21); //.235);
    b->setTexture(conf.trunkTexture);
    b->init(ignoreColSpace, 1,osgHTrunk);
    b->setPose(osg::Matrix::translate(0, 1.50, 0.03/*0.035*/) * pose );
    if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(1.0*conf.massfactor);//.3
    objects[Thorax]=b;

    double headsize=0.1;

    //  Neck
    b = new Capsule(0.05,0.03+headsize);
    b->setTexture(conf.bodyTexture);
    b->init(ignoreColSpace, 1, osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0, 1.6884+headsize/2, 0.0253) * pose );
//     b->setMass(.1/*1*/, 0, 0, 0, 0.0003125, 0.0003125, 0.0003125, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(.05*conf.massfactor);//.01
    objects[Neck]=b;


    // Head_comp
    b = new Sphere(headsize);
    b->setTexture(conf.headTexture);
    // b->setPose(osg::Matrix::translate(0, 1.79, 0.063) * pose );
    //    b->init(ignoreColSpace, 1,osgHandle);
    // b->setMass(5.89, 0, 0, 0, 0.0413, 0.0306, 0.0329, 0, 0, 0);
//     b->setMass(.1, 0, 0, 0, 0.0413, 0.0306, 0.0329, 0, 0, 0);
//    b->setMass(0.03*conf.massfactor);
//    objects[Head_comp]=b;

    // TODO: sort out mass!

    // Connect Head and Neck
    Transform* t = new Transform(objects[Neck], b,
                                 osg::Matrix::translate(0, 0, -(.05)));
    t->init(ignoreColSpace, 1,osgHandle.changeColor(conf.headColor));
    objects[Head_comp] = t;


    if(conf.irSensors){
      // add Eyes ;-)
      RaySensor* sensor = new IRSensor(1,0.02 , 1.0, RaySensor::drawAll );
      Matrix R = Matrix::translate(0,0,headsize) * Matrix::rotate(M_PI/10, 0, 1, 0) *
        Matrix::translate(0,headsize/10,0);
      sensor->setInitData(odeHandle, osgHandle, R);
      addSensor(std::shared_ptr<RaySensor>(sensor), Attachment(Head_comp));

      sensor = new IRSensor(1,0.02, 1.0, RaySensor::drawAll);
      R = Matrix::translate(0,0,headsize) * Matrix::rotate(-M_PI/10, 0, 1, 0)*
        Matrix::translate(0,headsize/10,0);
      sensor->setInitData(odeHandle, osgHandle, R);
      addSensor(std::shared_ptr<RaySensor>(sensor), Attachment(Head_comp));
    }


    // Left_Shoulder
    b = new Capsule(0.04,0.28);
    b->setTexture(conf.bodyTexture);
    b->init(ignoreColSpace, 1,osgHTrunk);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(0.3094, 1.587, 0.0227) * pose );
//     b->setMass(/*2*/.79, 0, 0, 0, 0.00056, 0.021, 0.021, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(0.2*conf.massfactor);
    objects[Left_Shoulder]=b;

    // Left_Forearm
    b = new Capsule(0.035,0.28);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(0.5798, 1.5909, 0.024) * pose );
//     b->setMass(1.21, 0, 0, 0, 0.00055, 0.0076, 0.0076, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(0.121*conf.massfactor);
    objects[Left_Forearm]=b;

    // Left_Hand
    // b = new Cylinder(0.06,0.05);
    b = new Sphere(0.07);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHTrousers);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0.7826, 1.5948, 0.024) * pose );
//     b->setMass(0.55, 0, 0, 0, 0.00053, 0.047, 0.0016, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor*conf.relArmmass, true);
    else
      b->setMass(0.1*conf.massfactor*conf.relArmmass);

    objects[Left_Hand]=b;

    // Right_Shoulder
    b = new Capsule(0.04,0.28);
    b->setTexture(conf.bodyTexture);
    b->init(ignoreColSpace, 1,osgHTrunk);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(-0.3094, 1.587, 0.0227) * pose );
//     b->setMass(/*2*/.79, 0, 0, 0, 0.00056, 0.021, 0.021, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(0.2*conf.massfactor);
    b->setColor(conf.trunkColor);
    objects[Right_Shoulder]=b;

    // Right_Forearm
    b = new Capsule(0.035,0.28);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,0,1,0) * osg::Matrix::translate(-0.5798, 1.5909, 0.024) * pose );
//     b->setMass(1.21, 0, 0, 0, 0.00055, 0.0076, 0.0076, 0, 0, 0);
        if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(0.121*conf.massfactor);
    objects[Right_Forearm]=b;

    // Right_Hand
    // b = new Cylinder(0.06,0.05);
    b = new Sphere(0.07);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHTrousers);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(-0.7826, 1.5948, 0.024) * pose );
    b->setColor(conf.trouserColor);
//     b->setMass(0.55, 0, 0, 0, 0.00053, 0.047, 0.0016, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor*conf.relArmmass, true);
    else
      b->setMass(.1*conf.massfactor*conf.relArmmass);
    objects[Right_Hand]=b;

    // Left_Thigh
    b = new Capsule(0.07,0.43);
    b->setTexture(conf.bodyTexture);
    b->init(ignoreColSpace, 1,osgHTrousers);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0)* osg::Matrix::rotate(-M_PI/60,0,0,1) *
               osg::Matrix::translate(0.0949, 0.8525, 0.0253) * pose );
//     b->setMass(8.35, 0, 0, 0, 0.145, 0.0085, 0.145, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor*conf.relLegmass, true);
    else
      b->setMass(.5*conf.massfactor*conf.relLegmass);
    objects[Left_Thigh]=b;

    // Left_Shin
    b = new Capsule(0.06,0.35);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(0.0702, 0.3988, 0.0357) * pose );
    //    b->setMass(4.16, 0, 0, 0, 0.069, 0.0033, 0.069, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor*conf.relLegmass, true);
    else
      b->setMass(0.5*conf.massfactor*conf.relLegmass);
    objects[Left_Shin]=b;

    // Left_Foot
    b = new Box(0.1,0.05,.3);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1, osgHTrunk);
    b->setPose(osg::Matrix::translate(0.0624, 0.1388, 0.0708) * pose );
    //    b->setMass(1.34, 0, 0, 0, 0.0056, 0.0056, 0.00036, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor, true);
    else
      b->setMass(.5*conf.massfactor*conf.relFeetmass);
    objects[Left_Foot]=b;

    // Right_Thigh
    b = new Capsule(0.07,0.43);
    b->setTexture(conf.bodyTexture);
    b->init(ignoreColSpace, 1,osgHTrousers);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0)* osg::Matrix::rotate(M_PI/60,0,0,1) *
               osg::Matrix::translate(-0.0949, 0.8525, 0.0253) * pose );
    //    b->setMass(8.35, 0, 0, 0, 0.145, 0.0085, 0.145, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor*conf.relLegmass, true);
    else
      b->setMass(.5*conf.massfactor*conf.relLegmass);
    objects[Right_Thigh]=b;

    // Right_Shin
    b = new Capsule(0.06,0.35);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1,osgHandle);
    b->setPose(osg::Matrix::rotate(M_PI_2,1,0,0) * osg::Matrix::translate(-0.0702, 0.3988, 0.0357) * pose );
    //    b->setMass(4.16, 0, 0, 0, 0.069, 0.0033, 0.069, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor*conf.relLegmass, true);
    else
      b->setMass(0.5*conf.massfactor*conf.relLegmass);
    objects[Right_Shin]=b;

    // Right_Foot
    b = new Box(0.1,0.05,.3);
    b->setTexture(conf.bodyTexture);
    b->init(odeHandle, 1, osgHTrunk);
    b->setPose(osg::Matrix::translate(-0.0624, 0.1388, 0.0708) * pose );
    //    b->setMass(1.34, 0, 0, 0, 0.0056, 0.0056, 0.00036, 0, 0, 0);
    if(conf.useDensity)
      b->setMass(conf.massfactor*conf.relFeetmass, true);
    else
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
      pelvisservo = new OneAxisServoVel(odeHandle, j, -DONT, DONT, DONT, DONT, DONT,
                                        conf.jointLimitFactor);
    } else {
      pelvisservo = new OneAxisServoCentered(j, -DONT, DONT, DONT, DONT, 2, 20,
                                             conf.jointLimitFactor);
    }

    // Trunk_comp, Belly and Thorax
    if(conf.useBackJoint){

      if(!conf.backSideBend){
        j = new HingeJoint(objects[Belly], objects[Thorax],
                           (objects[Belly]->getPosition() + objects[Thorax]->getPosition())/2,
                           Axis(-1,0,0) * pose);
        j->init(odeHandle, osgHandleJ, true, 0.36);
        joints.push_back(j);

        if(conf.useVelocityServos)
          servo1 = new OneAxisServoVel(odeHandle, j, -DONT, DONT, DONT, DONT, DONT, conf.jointLimitFactor);
        else
          servo1 = new OneAxisServoCentered(j, -DONT, DONT, DONT, DONT, 2, 20, conf.jointLimitFactor);
        backbendindex=backservos.size();
        backservos.push_back(servo1);
      }else{
        uj = new UniversalJoint(objects[Belly], objects[Thorax],
                                (objects[Belly]->getPosition() + objects[Thorax]->getPosition())/2,
                                Axis(-1,0,0) * pose, Axis(0,0,-1) * pose);
        uj->init(odeHandle, osgHandleJ, true, 0.22);
        joints.push_back(uj);
        if(conf.useVelocityServos)
          servo2 = new TwoAxisServoVel(odeHandle, uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                                       DONT, DONT, conf.jointLimitFactor);
        else
          servo2 = new TwoAxisServoCentered(uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                                            DONT, 2, 20, conf.jointLimitFactor);
        backservos2.push_back(servo2);
      }

      j = new HingeJoint(objects[Trunk_comp], objects[Belly],
                         (objects[Trunk_comp]->getPosition() + objects[Belly]->getPosition())/2,
                         Axis(0,1,0) * pose);
      j->init(odeHandle, osgHandleJ, true, 0.2);
      joints.push_back(j);
      if(conf.useVelocityServos)
        servo1 = new OneAxisServoVel(odeHandle, j, -DONT, DONT, DONT, DONT, DONT,
                                     conf.jointLimitFactor);
      else
        servo1 = new OneAxisServoCentered(j, -DONT, DONT, DONT, DONT, 2, 20, conf.jointLimitFactor);
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
                              Axis(0,0,-1) * pose, Axis(-1,0,0) * pose);
      uj->init(odeHandle, osgHandleJ, true, 0.12);
      joints.push_back(uj);

      servo2 = conf.useVelocityServos
        ? new TwoAxisServoVel(odeHandle, uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                              DONT, DONT, conf.jointLimitFactor)
        : new TwoAxisServoCentered(uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                                 DONT, 2, 20, conf.jointLimitFactor);
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
      ? new TwoAxisServoVel(odeHandle, uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                              DONT, DONT, conf.jointLimitFactor)
      : new TwoAxisServoCentered(uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                                 DONT, 2, 20, conf.jointLimitFactor);
    armservos.push_back(servo2);

    uj = new UniversalJoint(objects[Thorax], objects[Right_Shoulder],
                            Pos(-0.1768, 1.587, 0.0214) * pose,
                            Axis(0,0,-1) * pose, Axis(0,-1,0) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.12);
    joints.push_back(uj);

    if(conf.useVelocityServos)
      servo2 = new TwoAxisServoVel(odeHandle, uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                              DONT, DONT, conf.jointLimitFactor);
    else
      servo2 = new TwoAxisServoCentered(uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                                        DONT, 2, 20, conf.jointLimitFactor);
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
      servo1 = new OneAxisServoVel(odeHandle, j, -DONT, DONT, DONT, DONT, DONT,
                                   conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -DONT, DONT, DONT, DONT,
                                        2, 20, conf.jointLimitFactor);

    arm1servos.push_back(servo1);

    j = new HingeJoint(objects[Right_Shoulder], objects[Right_Forearm],
                       Pos(-0.442, 1.587, 0.024) * pose,  Axis(0,-1,0) * pose);
    j->init(odeHandle, osgHandleJ, false);
    joints.push_back(j);
    // servo1 = new OneAxisServo(j, -M_PI/10, M_PI/10, 20,0.1);
    if(conf.useVelocityServos)
      servo1 = new OneAxisServoVel(odeHandle, j, -DONT, DONT, DONT, DONT, DONT,
                                   conf.jointLimitFactor);
    else
      servo1 = new  OneAxisServoCentered( j, -DONT, DONT, DONT, DONT,
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
      ? new TwoAxisServoVel(odeHandle, uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                              DONT, DONT, conf.jointLimitFactor)
      : new TwoAxisServoCentered(uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                                 DONT, 2, 20, conf.jointLimitFactor);
    servo2->setDamping2(conf.hip2Damping);
    hipservos.push_back(servo2);

    uj = new UniversalJoint(objects[Hip], objects[Right_Thigh], Pos(-0.1118, 1.0904, 0.011) * pose,
                           Axis(1,0,0) * pose, Axis(0,0,1) * pose);
    uj->init(odeHandle, osgHandleJ, true, 0.15);
    joints.push_back(uj);

    servo2 =  conf.useVelocityServos
      ? new TwoAxisServoVel(odeHandle, uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                              DONT, DONT, conf.jointLimitFactor)
      : new TwoAxisServoCentered(uj, -DONT, DONT, DONT, -DONT, DONT, DONT,
                                 DONT, 2, 20, conf.jointLimitFactor);
    servo2->setDamping2(conf.hip2Damping);
    hipservos.push_back(servo2);


    // Thighs and Shins (Knees)
    j = new HingeJoint(objects[Left_Thigh], objects[Left_Shin], Pos(0.078, 0.6146, 0.0396) * pose,
                       Axis(2,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true, 0.15);
    joints.push_back(j);


    if( conf.useVelocityServos )
      servo1 = new OneAxisServoVel(odeHandle, j, -DONT, DONT, DONT, DONT, DONT,
                                     conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -DONT, DONT, DONT, DONT, 2, 20,
                                        conf.jointLimitFactor);
    kneeservos.push_back(servo1);

    j = new HingeJoint(objects[Right_Thigh], objects[Right_Shin], Pos(-0.078, 0.6146, 0.0396) * pose,
                       Axis(2,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true, 0.15);
    joints.push_back(j);

    if( conf.useVelocityServos )
      servo1 = new OneAxisServoVel(odeHandle, j, -DONT, DONT, DONT, DONT, DONT,
                                     conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -DONT, DONT, DONT, DONT, 2, 20,
                                        conf.jointLimitFactor);
    kneeservos.push_back(servo1);

    // Shins and Feet (Ankles)
    // fj = new FixedJoint(objects[Left_Shin], objects[Left_Foot]);
    j = new HingeJoint(objects[Left_Shin], objects[Left_Foot],
                       Pos(0.0624, 0.183, 0.0318) * pose,
                        Axis(-1,0,0) * pose);
    j->init(odeHandle, osgHandleJ, true,0.1);
    joints.push_back(j);

    if( conf.useVelocityServos )
      servo1 = new OneAxisServoVel(odeHandle, j, -DONT, DONT, DONT, DONT, DONT,
                                   conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -DONT, DONT, DONT, DONT, 2, 20,
                                        conf.jointLimitFactor);
    ankleservos.push_back(servo1);

    j = new HingeJoint(objects[Right_Shin], objects[Right_Foot],
                       Pos(-0.0624, 0.183, 0.0318) * pose,
                        Axis(-1,0,0) * pose);
    //  fj = new FixedJoint(objects[Right_Shin], objects[Right_Foot]);
    j->init(odeHandle, osgHandleJ, true, 0.1);
    joints.push_back(j);

    if( conf.useVelocityServos )
      servo1 =  new OneAxisServoVel(odeHandle, j, -DONT, DONT, DONT, DONT, DONT,
                                     conf.jointLimitFactor);
    else
      servo1 = new OneAxisServoCentered(j, -DONT, DONT, DONT, DONT, 2, 20,
                                        conf.jointLimitFactor);
    ankleservos.push_back(servo1);


    // //// We dont need this anymore because we have different spaces
    // // register ignored pairs
    // odeHandle.addIgnoredPair(objects[Head_comp],objects[Thorax]);
    // odeHandle.addIgnoredPair(objects[Trunk_comp],objects[Thorax]);
    // odeHandle.addIgnoredPair(objects[Left_Thigh],objects[Trunk_comp]);
    // odeHandle.addIgnoredPair(objects[Right_Thigh],objects[Trunk_comp]);
    // odeHandle.addIgnoredPair(objects[Left_Thigh],objects[Belly]);
    // odeHandle.addIgnoredPair(objects[Right_Thigh],objects[Belly]);
    // odeHandle.addIgnoredPair(objects[Right_Thigh],objects[Hip]);
    // odeHandle.addIgnoredPair(objects[Left_Thigh],objects[Hip]);
    // odeHandle.addIgnoredPair(objects[Left_Shin],objects[Trunk_comp]);
    // odeHandle.addIgnoredPair(objects[Right_Shin],objects[Trunk_comp]);
    // odeHandle.addIgnoredPair(objects[Left_Shin],objects[Belly]);
    // odeHandle.addIgnoredPair(objects[Right_Shin],objects[Belly]);
    // odeHandle.addIgnoredPair(objects[Left_Shin],objects[Thorax]);
    // odeHandle.addIgnoredPair(objects[Right_Shin],objects[Thorax]);

    // odeHandle.addIgnoredPair(objects[Left_Thigh],objects[Right_Thigh]);
    //  odeHandle.addIgnoredPair(objects[Left_Shin],objects[Right_Shin]);
   // odeHandle.addIgnoredPair(objects[Left_Foot],objects[Right_Foot]);

    if(conf.useGripper){
      GripperConf gc=Gripper::getDefaultConf();
      gc.gripDuration=conf.gripDuration;
      gc.releaseDuration=conf.releaseDuration;
      gc.color=osgHandle.getColor("Inforot");
      gc.size=0.18; //0.19 //0.15
      gc.drawAtContactPoint=false; //true
      gc.forbitLastPrimitive=true;
      Gripper* g;
      gc.name = "Gripper left hand";
      g = new Gripper(gc);
      g->attach(objects[Left_Hand]);
      grippers.push_back(g);
      addConfigurable(g);
      gc.name = "Gripper right hand";
      g = new Gripper(gc);
      g->attach(objects[Right_Hand]);
      grippers.push_back(g);
      addConfigurable(g);
    }


    // we call setParam in order to set all the dampings and default values
    notifyOnChange("thisparamdoesnotexist");

    created=true;
  };

  /// returns the grippers
  GripperList& Skeleton::getGrippers(){
    return grippers;
  }


  /** destroys vehicle and space
   */
  void Skeleton::destroy(){
    if (created){

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
      FOREACH(vector<TwoAxisServo*>, headservos, i){
        if(*i) delete *i;
      }
      FOREACH(vector<TwoAxisServo*>, armservos, i){
        if(*i) delete *i;
      }
      armservos.clear();
      FOREACH(vector<OneAxisServo*>, arm1servos, i){
        if(*i) delete *i;
      }
      arm1servos.clear();

      if(pelvisservo) delete pelvisservo;
      FOREACH(vector<OneAxisServo*>, backservos, i){
        if(*i) delete *i;
      }
      backservos.clear();
      FOREACH(vector<TwoAxisServo*>, backservos2, i){
        if(*i) delete *i;
      }
      backservos2.clear();

      FOREACH(GripperList, grippers, i){
        if(*i) delete *i;
      }
      grippers.clear();

      cleanup();
      //      ignoreColSpace.deleteSpace();
      odeHandle.deleteSpace();


    }

    created=false;
  }

  void Skeleton::notifyOnChange(const paramkey& key){
    // we just set all parameters independend of what was actually changed
    FOREACH(vector<TwoAxisServo*>, hipservos, i){
      if(*i) {
        (*i)->setPower( conf.hipPower * conf.powerFactor, conf.hip2Power * conf.powerFactor);
        (*i)->setDamping1(conf.hipDamping * conf.dampingFactor);
        (*i)->setDamping2(conf.hip2Damping * conf.dampingFactor);
        (*i)->setMaxVel(conf.hipVelocity);
        (*i)->setMinMax1(-conf.hipJointLimit*.2, +conf.hipJointLimit);
        (*i)->setMinMax2(-conf.hip2JointLimit*.4,+conf.hip2JointLimit);

      }
    }
    FOREACH(vector<TwoAxisServo*>, headservos, i){
      if(*i){
        (*i)->setPower(conf.neckPower * conf.powerFactor, conf.neckPower * conf.powerFactor);
        (*i)->setDamping1(conf.neckDamping * conf.dampingFactor);
        (*i)->setDamping2(conf.neckDamping * conf.dampingFactor);
        (*i)->setMaxVel(conf.neckVelocity);
        (*i)->setMinMax1(-conf.neckJointLimit, conf.neckJointLimit);
        (*i)->setMinMax2(-conf.neckJointLimit, conf.neckJointLimit);
      }
    }
    FOREACH(vector<OneAxisServo*>, kneeservos, i){
      if(*i){
        (*i)->setPower(conf.kneePower * conf.powerFactor);
        (*i)->setDamping(conf.kneeDamping * conf.dampingFactor);
        (*i)->setMaxVel(conf.kneeVelocity);
        (*i)->setMinMax(-conf.kneeJointLimit, conf.kneeJointLimit*0.1);
      }
    }
    FOREACH(vector<OneAxisServo*>, ankleservos, i){
      if(*i){
        (*i)->setPower(conf.anklePower * conf.powerFactor);
        (*i)->setDamping(conf.ankleDamping * conf.dampingFactor);
        (*i)->setMaxVel(conf.ankleVelocity);
        (*i)->setMinMax(-conf.ankleJointLimit, conf.ankleJointLimit*0.5);
      }
    }
    FOREACH(vector<TwoAxisServo*>, armservos, i){
      if(*i){
        (*i)->setPower(conf.armPower * conf.powerFactor * conf.relForce,
                       conf.armPower * conf.powerFactor * conf.relForce);
        (*i)->setDamping1(conf.armDamping * conf.dampingFactor);
        (*i)->setDamping2(conf.armDamping * conf.dampingFactor);
        (*i)->setMaxVel(conf.armVelocity);
        (*i)->setMinMax1(-conf.armJointLimit, conf.armJointLimit);
        (*i)->setMinMax2(-conf.armJointLimit*.3, conf.armJointLimit);
      }
    }
    FOREACH(vector<OneAxisServo*>, arm1servos, i){
      if(*i){
        (*i)->setPower(conf.elbowPower * conf.powerFactor * conf.relForce);
        (*i)->setDamping(conf.elbowDamping * conf.dampingFactor);
        (*i)->setMaxVel(conf.elbowVelocity);
        (*i)->setMinMax(0, conf.elbowJointLimit);
      }
    }

    pelvisservo->setPower(conf.pelvisPower * conf.powerFactor);
    pelvisservo->setDamping(conf.pelvisDamping * conf.dampingFactor);
    pelvisservo->setMaxVel(conf.pelvisVelocity);
    pelvisservo->setMinMax(-conf.pelvisJointLimit,+conf.pelvisJointLimit);


    FOREACHI(vector<OneAxisServo*>, backservos, i, index){
      if(*i){
        (*i)->setPower(conf.backPower * conf.powerFactor);
        (*i)->setDamping(conf.backDamping * conf.dampingFactor);
        (*i)->setMaxVel(conf.backVelocity);
       if(index==backbendindex) //bend
          (*i)->setMinMax(-0.7*conf.backJointLimit, 1.5*conf.backJointLimit);
        else // torsion and sidebend
          (*i)->setMinMax(-conf.backJointLimit, conf.backJointLimit);
      }
    }
    FOREACH(vector<TwoAxisServo*>, backservos2, i){
      if(*i){
        (*i)->setPower(conf.backPower * conf.powerFactor, conf.backPower * conf.powerFactor);
        (*i)->setDamping1(conf.backDamping * conf.dampingFactor);
        (*i)->setDamping2(conf.backDamping * conf.dampingFactor);
        (*i)->setMaxVel(conf.backVelocity);
        (*i)->setMinMax1(-0.7*conf.backJointLimit, 1.5*conf.backJointLimit); // bend
        (*i)->setMinMax2(-conf.backJointLimit, conf.backJointLimit); // bend/sideways
      }
    }
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
