/***************************************************************************
 *   Copyright (C) 2010 by                                                 *
 *    Guillaume de Chambrier <s0672742@sms.ed.ac.uk>                       *
 *    martius@informatik.uni-leipzig.de                                    *
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
 **************************************************************************/

#include <cmath>
#include <assert.h>

// #include <ode/ode.h>
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>

// include sensors
#include <ode_robots/speedsensor.h>
#include <ode_robots/irsensor.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/twoaxisservo.h>

#include <ode_robots/mathutils.h>

// include header file
#include "hexapod.h"

// rotation and translation matrixes (to make the code shorter)
#define ROTM osg::Matrix::rotate
#define TRANSM osg::Matrix::translate

using namespace osg;
using namespace std;

namespace lpzrobots {

  int t = 1;
  int c = 1;

  // constructor:
  // - give handle for ODE and OSG stuff
  Hexapod::Hexapod(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                   const HexapodConf& c, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "HexaPod 0.9"), conf(c)
  {
    // robot is not created till now
    pos1d = new double[3];
    pos1 = new dReal();
    pos2 = new dReal();
    t = 0;
    massOfobject = new dMass();
    getPos1 = true;
    timeCounter = conf.T;
    energyOneStep = new double[1];
    energyOneStep[0] =  0.0;
    costOfTran = 0.0;
    created=false;
    check = false;
    recordGait=false;
    dones = new bool[6];
    conf.v[0] = 0;

    angles = new double[12];
    heights = new double[6];
    hcorrection = 0.20803;

    legContactArray = new Leg[6];
    legmass= (1.0 - conf.percentageBodyMass)*conf.mass/(conf.legNumber*2.0);

    addParameter("coxaPower", &conf.coxaPower);
    addParameter("coxaDamp", &conf.coxaDamping);
    addParameter("coxaJointLimitH", &conf.coxaJointLimitH);
    addParameter("coxaJointLimitV", &conf.coxaJointLimitV);
    addParameter("coxaSpeed", &conf.coxaSpeed);

    if(conf.useTebiaJoints){
      addParameter("tebiaPower",      &conf.tebiaPower);
      addParameter("tebiaDamp",       &conf.tebiaDamping);
      addParameter("tebiaJointLimit", &conf.tebiaJointLimit);
    }

    // name the sensors
    for(int n=0; n<conf.legNumber; n++){
      addInspectableDescription("x[" + itos (n*2) + "]",
                                "leg pair " + itos(n/2) + (n%2==0 ? " right" : " left")
                                + " up/down");
      addInspectableDescription("x[" + itos (n*2+1) + "]",
                                "leg pair " + itos(n/2) + (n%2==0 ? " right" : " left")
                                + " front/back");
    }

    // Georg: you can also add inspectables here to see them in the logfile/guilogger
    // e.g.
    addInspectableValue("Energy", &E_t, "Energy over several timesteps");

  };


  int Hexapod::getMotorNumber(){
    return  2*hipservos.size();
  };

  /* sets actual motorcommands
     @param motors motors scaled to [-1,1]
     @param motornumber length of the motor array
  */
  void Hexapod::setMotors(const motor* motors, int motornumber){
    assert(created); // robot must exist
    int len = min(motornumber, getMotorNumber())/2;

    for(int i = 0; i < len; i++){
      hipservos[i]->set(motors[2*i],motors[2*i+1]);
    }

    FOREACH(vector<OneAxisServo*>, tebiasprings, i){
      if(*i) (*i)->set(0);
    }
    FOREACH(vector<OneAxisServo*>, whiskersprings, i){
      if(*i) (*i)->set(0);
    }



  };

  int Hexapod::getSensorNumber(){
    return 2*hipservos.size() + irSensorBank.size();
  };

  /* returns actual sensorvalues
     @param sensors sensors scaled to [-1,1] (more or less)
     @param sensornumber length of the sensor array
     @return number of actually written sensors
  */
  int Hexapod::getSensors(sensor* sensors, int sensornumber){
    assert(created);
    int len = min(sensornumber, getSensorNumber() - irSensorBank.size())/2;

    for(int i = 0; i < len; i++){
      sensors[2*i]   = hipservos[i]->get1();
      sensors[2*i+1] = hipservos[i]->get2();
    }

    len = 2*len;



    if (conf.irFront || conf.irBack){
      len += irSensorBank.get(sensors+len, sensornumber-len);
    }

    return len;
  };


  void Hexapod::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    //    Matrix p2;
    //    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8));
    create(pose);
  };


  /**
   * updates the osg notes
   */
  void Hexapod::update(){
    assert(created); // robot must exist

    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }

    irSensorBank.update();
  };

  double Hexapod::outwardMechanicalPower(const dReal *torques,const dReal *angularV){

    double mechanicalPower = 0.0;

    for(int i = 0; i < 3; i++){
      mechanicalPower += torques[i]*angularV[i];
    }

    if(mechanicalPower <= 0.0){
      mechanicalPower = 0.0;
    }

    return mechanicalPower;
  }

  double Hexapod::energyConsumpThroughtHeatLoss(const dReal *torques){
    // Georg: This should be sqr(torques[0]) + sqr(torques[1]) + ..
    // torque can be negative!
    return pow(torques[0] + torques[1] + torques[2],2);
  }

  double Hexapod::energyConsumption(){

    const dReal *torques;
    const dReal *angularV;
    double gamma = 0.005;
    double e = 0.0;

    for(unsigned int i = 0; i < legs.size(); i++){
      torques = dBodyGetTorque(legs[i]->getBody());
      angularV = dBodyGetAngularVel(legs[i]->getBody());
      e += outwardMechanicalPower(torques,angularV) + gamma*energyConsumpThroughtHeatLoss(torques);
    }
    return e;
  }

  double Hexapod::getMassOfRobot(){

    double totalMass = 0.0;

    for(unsigned int i = 0; i < objects.size(); i++){
      dBodyGetMass(objects[i]->getBody(),massOfobject);
      totalMass += massOfobject->mass;
    }
    return totalMass;
  }

  double Hexapod::costOfTransport(double E, double W, double V, double T){
    return E/(W*V*T);
  }

  double Hexapod::round(double num, int x){

    return  ceil( ( num * pow( 10,x ) ) - 0.49 ) / pow( 10,x );

  }
  /** this function is called in each timestep. It should perform robot-internal checks,
      like space-internal collision detection, sensor resets/update etc.
      @param global structure that contains global data from the simulation environment
  */
  void Hexapod::doInternalStuff(GlobalData& global){
    irSensorBank.reset();

    energyOneStep[0] = energyConsumption();

    t = global.time;

    if(global.time <= timeCounter){
      E_t += energyOneStep[0];
    }

    if(getPos1){
      pos1 = dBodyGetPosition(trunk->getBody());
      pos1d[0] = pos1[0];
      pos1d[1] = pos1[1];
      pos1d[2] = pos1[2];
      getPos1 = false;
    }

    const dReal* velocity = dBodyGetLinearVel( trunk->getBody() );
    const double v = abs(velocity[0]);
    conf.v[0] = v;


    if(global.time >= timeCounter){

      pos2 = dBodyGetPosition(trunk->getBody());
      distance = sqrt(pow((pos2[0] - pos1d[0]),2) + pow((pos2[1] - pos1d[1]),2) + pow((pos2[2] - pos1d[2]),2));
      conf.v[0] = distance/conf.T;
      costOfTran = costOfTransport(E_t,getMassOfRobot(),conf.v[0],conf.T);
      //cout<< "cost of Transport: " << costOfTran << endl;
      timeCounter += conf.T;
      E_t = 0.0;
      getPos1 = true;
    }


    for(unsigned int i = 0; i < 6; i++){

      const dReal *position = dBodyGetPosition(legContactArray[i].bodyID);

      // cout<< dJointGetUniversalAngle1(joints[0]->getJoint()) * 180/M_PI  << endl;
      // cout<< dJointGetUniversalAngle2(joints[0]->getJoint())  * 180/M_PI<< endl;
      //  cout << dJointGetUniversalAngle1(legContactArray[i].joint) * 180/M_PI << endl;
      //  cout << dJointGetUniversalAngle2(legContactArray[i].joint) * 180/M_PI << endl;

      heights[i] = abs(round(position[2] -  hcorrection,3));
      angles[2*i]   = dJointGetUniversalAngle1(legContactArray[i].joint) * 180/M_PI ;
      angles[2*i+1] = dJointGetUniversalAngle2(legContactArray[i].joint) * 180/M_PI ;

    }

  }

  /** checks for internal collisions and treats them.
   *  In case of a treatment return true (collision will be ignored by other objects
   *  and the default routine)  else false (collision is passed to other objects and
   *  (if not treated) to the default routine).
   */
  bool Hexapod::collisionCallback(void *data, dGeomID o1, dGeomID o2){

    const int NUM_CONTACTS = 8;
    dContact contacts[NUM_CONTACTS];
    int numCollisions = dCollide(o1, o2, NUM_CONTACTS, &contacts[0].geom, sizeof(dContact));
    // Georg: this would also be possible by a special substance with callback.
    //  I will maybe implement a contact sensor anyway...

    //set all contacts to zero
    for(int j = 0; j < 6; j++) {
      conf.legContacts[j] = 0;
    }

    for(int i = 0; i < numCollisions; ++i)
      {
        dBodyID b1 =  dGeomGetBody(contacts[i].geom.g1);
        // Georg: are you sure that b1 is always the leg?
        if(legContactArray[0].bodyID == b1){conf.legContacts[0] = 1; }
        if(legContactArray[1].bodyID == b1){conf.legContacts[1] = 4; }
        if(legContactArray[2].bodyID == b1){conf.legContacts[2] = 2; }
        if(legContactArray[3].bodyID == b1){conf.legContacts[3] = 5; }
        if(legContactArray[4].bodyID == b1){conf.legContacts[4] = 3; }
        if(legContactArray[5].bodyID == b1){conf.legContacts[5] = 6; }

      }

    /*

    // cout<< "t: " << t << "   timeC: " << timeCounter << endl;
    if((t + 0.01) >= timeCounter){
    if(recordGait){
    cout<<"in here" << endl;
    fprintf(f,"%d,%d,%d,%d,%d,%d,%g;",conf.legContacts[0],conf.legContacts[2],conf.legContacts[4],conf.legContacts[1],conf.legContacts[3],conf.legContacts[5],t);
    fprintf(f,"\n");
    check = true;
    }else if(check == true && recordGait == false){
    fprintf(f,"]\n");
    fclose(f);
    check = false;
    }
    }*/



    return false;
  }


  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void Hexapod::create( const osg::Matrix& pose ){
    if (created) {
      destroy();
    }

    odeHandle.createNewSimpleSpace(parentspace,false);
    // color of joint axis and whiskers
    OsgHandle osgHandleJ = osgHandle.changeColor(Color(72./255.,16./255.,16./255.));
    TwoAxisServo* servo;
    OneAxisServo* spring;
    FixedJoint* fixedJoint;

    // create body
    double twidth = conf.size * conf.width ;// 1/1.5;
    double theight = conf.size * conf.height; // 1/4;
    trunk = new Box(conf.size, twidth, theight);
    trunk->setTexture("Images/toy_fur3.jpg");
    trunk->init(odeHandle, conf.mass*conf.percentageBodyMass, osgHandle);
    osg::Matrix trunkPos = TRANSM(0,0,conf.legLength)*pose;
    trunk->setPose(trunkPos);
    objects.push_back(trunk);


    osg::Matrix m0 = pose;

    if(conf.irSensors == true){
      for(int i = -1; i < 2; i+=2){

        irbox = new Box(0.1,0.1,0.1);
        irbox->setTexture("Images/toy_fur3.jpg");
        irbox->init(odeHandle, 0.00001, osgHandle);
        irbox->setPose(ROTM(M_PI/4,0,0,1) * TRANSM(i*conf.size/2,0,theight/2)*trunkPos);
        objects.push_back(irbox);
        fixedJoint = new FixedJoint(trunk,irbox);
        fixedJoint->init(odeHandle, osgHandleJ, true, 0.4);
        joints.push_back(fixedJoint);
      }

      for(int i = -1; i < 2; i+=2){

        irbox = new Box(0.1,0.1,0.15);
        irbox->setTexture("Images/toy_fur3.jpg");
        irbox->init(odeHandle, 0.00001, osgHandle);
        irbox->setPose(TRANSM(0,i*twidth/2,theight/2 + 0.05)*trunkPos);
        objects.push_back(irbox);
        fixedJoint = new FixedJoint(trunk,irbox);
        fixedJoint->init(odeHandle, osgHandleJ, true, 0.4);
        joints.push_back(fixedJoint);
      }


      irSensorBank.init(odeHandle, osgHandle);

      if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
             IRSensor* sensor = new IRSensor();
             irSensorBank.registerSensor(sensor, objects[2],
                                         Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
                                         Matrix::translate(1*0.05,0,0),
                                         conf.irRangeFront, RaySensor::drawAll);
               IRSensor* sensor2 = new IRSensor();
               irSensorBank.registerSensor(sensor2, objects[2],
                                    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
                                    Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
                                    Matrix::translate(0,-0.05,0),
                                    conf.irRangeFront, RaySensor::drawAll);

      }
      if (conf.irBack){ // add front left and front right infrared sensor to sensorbank if required

              IRSensor* sensor = new IRSensor();
               irSensorBank.registerSensor(sensor, objects[1],
                                           Matrix::rotate(1*-M_PI/2, Vec3(0,1,0)) *
                                           Matrix::translate(-1*0.05,0,0),
                                           conf.irRangeBack, RaySensor::drawAll);

        IRSensor* sensor2 = new IRSensor();
        irSensorBank.registerSensor(sensor2, objects[1],
                                    Matrix::rotate(1*-M_PI/2, Vec3(0,1,0)) *
                                    Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
                                    Matrix::translate(0,0.05,0),
                                    conf.irRangeBack, RaySensor::drawAll);
      }
      if(conf.irLeft){
        IRSensor* sensor = new IRSensor();
        irSensorBank.registerSensor(sensor, objects[3],
                                    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
                                    Matrix::rotate(-M_PI/2, Vec3(0,0,1)) *
                                    Matrix::translate(0,-0.05,0.05),
                                    conf.irRangeLeft, RaySensor::drawAll);

            /* IRSensor* sensor2 = new IRSensor();
           irSensorBank.registerSensor(sensor2, objects[3],
           Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
           Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
           Matrix::translate(0,-0.05,0),
           conf.irRangeLeft, RaySensor::drawAll);*/
      }
      if(conf.irRight){
        IRSensor* sensor = new IRSensor();
        irSensorBank.registerSensor(sensor, objects[4],
                                    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
                                    Matrix::rotate(M_PI/2, Vec3(0,0,1)) *
                                    Matrix::translate(0,0.05,0.05),
                                    conf.irRangeLeft, RaySensor::drawAll);


        /* IRSensor* sensor2 = new IRSensor();
           irSensorBank.registerSensor(sensor2, objects[4],
           Matrix::rotate(1*-M_PI/2, Vec3(0,1,0)) *
           Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
           Matrix::translate(0,0.05,0),
           conf.irRangeRight, RaySensor::drawAll);*/
      }
    }


    // legs  (counted from back to front)
    double legdist = conf.size*0.9 / (conf.legNumber/2-1);
    for ( int n = 0; n < conf.legNumber; n++ ) {

      int v = n;

      double l1 = conf.legLength*0.5;
      double t1 = conf.legLength/10;
      double l2 = conf.legLength*0.5;
      double t2 = conf.legLength/10;

      // upper limp
      Primitive* coxaThorax;
      Pos pos = Pos(-conf.size/(2+0.2) + ((int)n/2) * legdist,
                    n%2==0 ? - twidth/2 : twidth/2,
                    conf.legLength - theight/3);

      osg::Matrix m = ROTM(M_PI/2,v%2==0 ? -1 : 1,0,0) * TRANSM(pos) * pose;
      coxaThorax = new Capsule(t1, l1);
      coxaThorax->setTexture("Images/toy_fur3.jpg");
      coxaThorax->init(odeHandle, legmass, osgHandle);

      osg::Matrix m1 =  TRANSM(0,0,-l1/2)
        * ROTM(M_PI,0,0,v%2==0 ? -1 : 1)
        * ROTM(2*M_PI,0,v%2==0 ? -1 : 1,0) * m;

      coxaThorax->setPose(m1);
      thoraxPos.push_back(coxaThorax->getPosition());

      thorax.push_back(coxaThorax);

      objects.push_back(coxaThorax);
      legs.push_back(coxaThorax);
      // powered hip joint
      Pos nullpos(0,0,0);

      UniversalJoint* j
        = new UniversalJoint(trunk, coxaThorax, nullpos * m,
                             ROTM(M_PI,0,0,v%2==0 ? -1 : 1) * Axis(v%2==0 ? -1 : 1,0,0) * m,
                             ROTM(M_PI,0,0,v%2==0 ? -1 : 1) * Axis(0,1,0) * m);

      j->init(odeHandle, osgHandleJ, true, t1 * 2.1);
      joints.push_back(j);

      legContactArray[n].joint = j->getJoint();

      // values will be set in setParam later
      servo =  new TwoAxisServoVel(odeHandle, j,-1,1, 1, -1,1,1,0 );
      //     servo =  new UniversalServo(j,-1,1, 1, -1,1,1,0);
      hipservos.push_back(servo);

      // lower leg
      Primitive* tibia;
      tibia = new Capsule(t2, l2);
      tibia->setTexture("Images/toy_fur3.jpg");
      tibia->init(odeHandle, legmass, osgHandle);
      osg::Matrix m2 =   TRANSM(0,0,-l2/2) * ROTM(1.5,v%2==0 ? -1 : 1,0,0)
        * TRANSM(0,0,-l1/2) * m1;
      tibia->setPose(m2);
      objects.push_back(tibia);
      legs.push_back(tibia);

      legContactArray[n].legID = n;
      legContactArray[n].geomid = tibia->getGeom();
      legContactArray[n].bodyID = tibia->getBody();

      if(conf.useTebiaJoints){
      // springy knee joint
        HingeJoint* k = new HingeJoint(coxaThorax, tibia, Pos(0,0,-l1/2) * m1,
                                       Axis(v%2==0 ? -1 : 1,0,0) * m1);
        k->init(odeHandle, osgHandleJ, true, t1 * 2.1);
        // servo used as a spring
        spring = new HingeServo(k, -1, 1, 1, 0.01,0); // parameters are set later
        tebiasprings.push_back(spring);
        joints.push_back(k);
      }else{
        // fixed knee joint
        FixedJoint* k = new FixedJoint(coxaThorax, tibia);
        k->init(odeHandle, osgHandleJ, false, t1 * 2.1);
        joints.push_back(k);
      }
      // lower limp should not collide with body!
      odeHandle.addIgnoredPair(trunk,tibia);
      // Georg: we could also ignore all internal collisions (see createNewSimpleSpace above)
    }
    // New: wiskers
    for ( int n = -1; n < 2; n+=2 ) {
      double l1 = conf.legLength*0.5;
      double t1 = conf.legLength/30;

      Primitive* whisker;
      Pos pos = Pos(conf.size/(2)+t1,
                    n*twidth/4,
                    conf.legLength + theight/5);

      osg::Matrix m = ROTM(M_PI/10, n,0,0) * ROTM(M_PI/2+M_PI/10, 0,-1,0) * TRANSM(pos) * pose;
      whisker = new Capsule(t1, l1);
      whisker->init(odeHandle, legmass/10, osgHandleJ);
      osg::Matrix m1 = TRANSM(0,0,-l1/2) * m;
      whisker->setPose(m1);
      objects.push_back(whisker);

      //FixedJoint* k = new FixedJoint(trunk, whisker);
      //k->init(odeHandle, osgHandle, false, 0);
      HingeJoint* k = new HingeJoint(trunk, whisker, Pos(0,0,0) * m,
                                     Axis(1,0,0) * m);
      k->init(odeHandle, osgHandleJ, true, t1 * 2.1);
      // servo used as a spring
      spring = new HingeServo(k, -M_PI/6, M_PI/6, .1, 0.01,0);
      whiskersprings.push_back(spring);
      joints.push_back(k);

      Primitive* whisker2;
      whisker2 = new Capsule(t1/2, l1);
      whisker2->init(odeHandle, legmass/10, osgHandleJ);
      osg::Matrix m2 = TRANSM(0,0,-l1/2)
        * ROTM(M_PI/10, n,0,0)
        * ROTM(M_PI/10, 0,1,0) * TRANSM(0,0,-l1/2) * m1;
      whisker2->setPose(m2);
      objects.push_back(whisker2);

      //      k = new FixedJoint(whisker, whisker2);
      //      k->init(odeHandle, osgHandleJ, false, 0);
      k = new HingeJoint(whisker, whisker2, Pos(0,0,-l1/2) * m1,
                                     Axis(0,1,0) * m1);
      k->init(odeHandle, osgHandleJ, true, t1 * 2.1);
      // servo used as a spring
      spring = new HingeServo(k, -M_PI/6, M_PI/6, .05, 0.01,0);
      whiskersprings.push_back(spring);
      joints.push_back(k);


    }


    setParam("dummy",0); // apply all parameters.

    created=true;
  };


  /** destroys vehicle and space
   */
  void Hexapod::destroy(){
    if (created){
      //  odeHandle.removeIgnoredPair(bigboxtransform);
      odeHandle.removeIgnoredPair(trunk,headtrans);
      irSensorBank.clear();

      FOREACH(vector<TwoAxisServo*>, hipservos, i){
        if(*i) delete *i;
      }
      hipservos.clear();
      FOREACH(vector<OneAxisServo*>, tebiasprings, i){
        if(*i) delete *i;
      }
      tebiasprings.clear();
      FOREACH(vector<OneAxisServo*>, whiskersprings, i){
        if(*i) delete *i;
      }
      whiskersprings.clear();


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

  bool Hexapod::setParam(const paramkey& key, paramval val, bool traverseChildren){
    // the parameters are assigned here
    bool rv = Configurable::setParam(key, val);
    // we simply set all parameters here
    FOREACH(vector<TwoAxisServo*>, hipservos, i){
      if(*i){
        (*i)->setPower1(conf.coxaPower);
        (*i)->setPower2(conf.coxaPower);
        (*i)->setDamping(conf.coxaDamping);
        (*i)->setMinMax1(-conf.coxaJointLimitV,+conf.coxaJointLimitV);
        (*i)->setMinMax2(-conf.coxaJointLimitH,+conf.coxaJointLimitH);
        (*i)->setMaxVel(conf.coxaSpeed);
      }
    }
    FOREACH(vector<OneAxisServo*>, tebiasprings, i){
      if(*i){
        (*i)->setPower(conf.tebiaPower);
        (*i)->setDamping(conf.tebiaDamping);
        (*i)->setMinMax(-conf.tebiaJointLimit,+conf.tebiaJointLimit);
      }
    }
    return rv;


  }

}
