/***************************************************************************
 *   Copyright (C) 2012 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
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
 ***************************************************************************/
/*
Scenarios:
Crawling:
From start: (increasing the hip2joint makes the legs more appart, anklepower makes it more crawl)
./start_dbg -set "{epsh=0.05 backjointlimit=0.3 hip2jointlimit=1 armpower=20 anklepower=6 synboost=1.4 indnorm=0 urate=0.005}"
Crawling2 with Vision:
./start_dbg -two -senseother -set "{epsh=0.05 backjointlimit=0.3 hip2jointlimit=1 armpower=20 anklepower=6 synboost=1.4 indnorm=0 urate=0.005}"

Bungee:
./start_dbg -bungee -set "{epsh=0.07 powerfactor=0.3 synboost=0.96 urate=0.01 force=26.5}" -pause -f 1

2Bungee:
./start -2bungee -senseother  -set "{epsh=0.0 powerfactor=0.3 synboost=1.3 urate=0.02 indnorm=0}"
(with sine)
./start_dbg -2bungee -senseother -fixbungee -2ndsine -set "{epsh=0.0 powerfactor=0.3 synboost=1.3 urate=0.02 force=27 amplitude=0.7 period=70 phaseshift=0.2}" -pause -f 1

Trainer (robot at wheel)
./start_dbg -trainer -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 synboost=0.96}"

Trainer for hand and feet
 ./start_dbg -trainer -trainer4feet -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 synboost=0.96 anklejointlimit=0.7}"
with limited back/pelvis movement (no synchronization in any case)
./start_dbg -trainer -trainer4feet -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 synboost=0.96 anklejointlimit=0.7 backjointlimit=0.5 pelvisjointlimit=0.3}" -g 1


Trainer 2 Robots
./start_dbg -trainer2 -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 synboost=1}"
with vision
./start_dbg -trainer2 -senseother -visiondelay -set "{indnorm=0 epsh=0.0 noise=0.0 urate=0.05 synboost=1 delay=0 senseotherfactor=-1}"

Trainer 2 Robots Standing
./start_dbg -trainer2 -set "{indnorm=0 epsh=0.0 noise=0.01 urate=0.05 powerfactor=1.5 synboost=1.1}"????

2 Stools beside
./start -2stools -senseother -beside -f 1 -set "{epsh=0.05 synboost=1.3 indnorm=1}"
./start_dbg -2stools -beside -visiondelay -set "{epsh=0.05 synboost=1.0 delay=20}" -g 10

// retrieve times of parameter changes/ loading etc.
// grep -C 1 "# " Humanoid0_Normal_.log  | cut -f1-5 -d " "

*/

#include <stdio.h>

#include <selforg/noisegenerator.h>
#include <selforg/sinecontroller.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/forceboostwiring.h>
#include <selforg/derivativewiring.h>
#include <selforg/motornoisewiring.h>
#include <selforg/copywiring.h>
#include <selforg/ringbuffer.h>
#include <selforg/motorbabbler.h>

// #include <ode_robots/skeleton.h>
#include "skeleton.h"

#include <ode_robots/joint.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>
#include <ode_robots/delaysensor.h>
#include <ode_robots/relativepositionsensor.h>

#include <ode_robots/operators.h>
#include <ode_robots/tmpprimitive.h>


#include <selforg/soml.h>
#include <selforg/sox.h>
#include <selforg/dep.h>
#include "randomdyn.h"
#include <selforg/remotecontrolled.h>

#include "environment.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

#define ROBOTSTOREFILE "humanoid_initial.rob"

/// provides sensor values of other robot
class SensorsOther : public virtual Sensor, public Configurable {
public:
  SensorsOther(int number)
  : number(number), other(0), othersensors(0) {
    addParameterDef("senseotherfactor",&factor, 1, -2, 2, "factor for perception of sensor values of other robot");
    setName("SensorsOther");
    setBaseName("FromOther-");
  }

  virtual void setOther(AbstractRobot* other) {
    this->other=other;
    int len = other->getSensorNumber();
    othersensors = new double[len];
    memset(othersensors,0,sizeof(double)*len);
  };

  virtual void init(Primitive* own, Joint* joint = 0) override {
  };

  virtual bool sense(const GlobalData& globaldata)override {
    assert(other);
    other->getSensors(othersensors,other->getSensorNumber());
    return true;
  };

  virtual int getSensorNumber() const override { return number;};

  virtual int get(sensor* sensors, int length) const override {
    assert(length>=number);
    for(int k=0; k<number; k++){
      sensors[k]=factor*othersensors[k];
    }
    return number;
  }
  virtual std::list<sensor> getList() const override {
    return getListOfArray();
  };

protected:
  int number;
  AbstractRobot* other;
  double* othersensors;
  double factor;
};

enum SimType { Normal, TwoNormal, Rescue, Fight, FightFixed, Reck, Bungee, TwoBungee, Copy, Fly, Table, Trainer, TwoTrainer, Stool, TwoStools };
string typeToString(SimType t){
  switch(t){
  case Normal:
    return "Normal";
  case TwoNormal:
    return "TwoNormal";
  case Reck:
    return "Reck";
  case Rescue:
    return "Rescue";
  case Bungee:
    return "Bungee";
  case TwoBungee:
    return "TwoBungee";
  case Fight:
    return "Fight";
  case FightFixed:
    return "FightFixed";
  case Copy:
    return "Copy";
  case Fly:
    return "Fly";
  case Table:
    return "Table";
  case Trainer:
    return "Trainer";
  case TwoTrainer:
    return "TwoTrainer";
  case Stool:
    return "Stool";
  case TwoStools:
    return "TwoStools";
  }
  return "unknown";
}
double noise       = 0.0;
double cInit       =0;// -1.2;
double tilt        = 0.0;
double powerfactor = .6;//1.0;
string name        = "";
bool   useSox      = false;
bool   useSine     = false;
double noisecontrol= 0;
double feedbackstrength = 0;
double wheelmass        = 70;
int    s4avg       = 1;
char*  controllerfile[2]={0,0};
bool fighterRandom=false;
bool secondBlind  = false;
bool secondSine  = false;
bool senseOther  = false;
bool beside      = false;
bool visiondelay = false;
bool fixBungee   = false;
bool exterioception = false;
bool trainer4feet  = false;
bool trainersameside  = false;
bool trainer4feetsameside  = false;
bool noEigenvalues = false;
bool mergeSensors  = false;
int babbling=0;


class ThisSim : public Simulation {
public:
  SimType type;
  Env env;

  int  fixedSegm;
  Joint* fixator;
  Joint* leftHandJoint;
  Joint* rightHandJoint;

  PassiveCapsule* reck;
  Playground* playground;
  //  AbstractObstacle* playground;
  double hardness;
  Substance s;

  Joint* connector[4];
  matrix::Matrix conn_forces;


  double reckX;
  double reckY;
  double reckZ;

  double tableForce;

  ThisSim(SimType type)
    : type(type){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    if(type==Rescue)
      addColorAliasFile("overwriteGroundColor.txt");
    setGroundTexture("Images/whiteground.jpg");
    setTitle("Autonomous Learning (Der & Martius)");
    setCaption("LpzRobots");
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos (Pos(2.60209, 6.00217, 2.42803),  Pos(162.466, -13.1846, 0));

    setCameraMode(Static);

    int humanoids = 1;

    bool fixedInAir = false;
    fixator=0;
    fixedSegm  = -1;
    env.type=Env::Normal;
    global.odeConfig.setParam("noise",noise); //for more variety
    //    global.odeConfig.setParam("realtimefactor",0);
    global.odeConfig.setParam("simstepsize",0.01);
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("gravity", -9.81);
    global.odeConfig.addParameterDef("tableforce", &tableForce, 1000,0,5000,"Force to attach hands to table");

    switch(type){
    case Fly:
      global.odeConfig.setParam("gravity", 0);
    case Normal:
      env.roughness  = 2.5;
      break;
    case TwoNormal:
      env.roughness  = 2.5;
      humanoids      = 2;
      break;
    case Bungee:
      break;
    case TwoBungee:
      humanoids       = 2;
      break;
    case Reck:
      env.height=2.0;
      break;

    case Copy:
      humanoids       = 1;//2
    case Rescue:
      setCameraHomePos (Pos(1.97075, 4.99419, 2.03904),  Pos(159.579, -13.598, 0));
      //  env.type       = Env::OpenPit;
      if(type==Copy){
        env.type          = Env::Pit2;
        env.pit2Position  = Pos(2,0,0);
      } else {
        env.type       = Env::Pit;
      }
      env.pitsize    = 1.0;//.9;
      env.thickness  = .1;
      env.height     = 1.4;
      env.roughness  = 2.0;
      env.hardness   = 30;
      env.numSeeSaws = 0;

      // global.addTmpObject(new TmpDisplayItem(new OSGText("plasdpaldss",16),
      //                                        TRANSM(20,30,0),
      //                                        osgHandle.getColor("hud"))
      //                     , 60);

      break;
    case Fight:
    case FightFixed:
      env.type        = Env::Normal;
      global.odeConfig.setParam("gravity", type==FightFixed ? 0 : -9.81);
      env.widthground = 5;
      env.height      = 0.5;
      env.roughness   = 3;
      humanoids       = 2;
      env.numSeeSaws  = 0;
      break;
    case Table:
      // fixedInAir = true;
      env.type       = Env::Normal;
      env.withStool  = true;
      env.withTable  = true;
      env.roughness  = 2.5;
      break;
    case Stool:
      env.type       = Env::Normal;
      env.withStool  = true;
      env.roughness  = 2.5;
      break;
    case TwoStools:
      env.type       = Env::Normal;
      env.withStool  = true;
      env.withStool2 = true;
      env.roughness  = 2.5;
      humanoids      = 2;
      if(beside) env.stool2Pose = TRANSM(Pos(1.6, -0.85, 0.2+0.4));
      break;
    case Trainer:
      env.type       = Env::Normal;
      env.withStool  = true;
      env.withTrainer= true;
      env.withTrainer4Feet = trainer4feet;
      env.wheelOpposite = !trainersameside;
      env.wheel4FeetOpposite = !trainer4feetsameside;
      env.roughness  = 2.5;
      env.wheelmass  = wheelmass;
      break;
    case TwoTrainer:
      env.type       = Env::Normal;
      env.withStool  = true;
      env.withStool2 = true;
      env.withTrainer2 = true;
      env.wheelOpposite = !trainersameside;
      env.stool2Pose = TRANSM(Pos(1.0, -0.85, 0.2+0.4));
      env.roughness  = 2.5;
      env.wheelmass  = wheelmass;
      humanoids      = 2;
      break;
    }

    env.create(odeHandle, osgHandle, global);

    env.numSpheres  = 0;
    env.numBoxes    = 0;
    env.numCapsules = 0;
    env.placeObstacles(odeHandle, osgHandle, global);

    global.configs.push_back(&env);
    //    global.configs.push_back(this);

    leftHandJoint = rightHandJoint = 0;
    reck = 0;
    reckX = reckY = reckZ = 0;

    std::vector<std::shared_ptr<SensorsOther> > sensorothers;

    for (int i=0; i< humanoids; i++){ //Several humans
      bool reckturner = (type==Reck);
      bool useOtherPerception=false;
      if (i>0) reckturner=false;

      // normal servos
      // SkeletonConf conf = Skeleton::getDefaultConf();
      // velocity servos
      SkeletonConf conf = Skeleton::getDefaultConfVelServos();

      OsgHandle skelOsgHandle=osgHandle.changeColorSet((type==Copy && i==1) ? 5 : i);
      double initHeight=0.8;

      conf.useBackJoint = true;
      conf.powerFactor = powerfactor;
      conf.dampingFactor = .0;
      conf.pelvisJointLimit = M_PI/6; // new
      conf.elbowJointLimit = 1.6;     // new

      conf.backSideBend = true;

      switch(type){
      case Fly:
        initHeight = 1;
        break;
      case Normal:
      case TwoNormal:
        //       conf.powerFactor = 1.5;
        initHeight = 0.45;
        break;
      case Reck:
        //       conf.powerFactor = 0.3;
        conf.relForce = 2;
        conf.handsRotating = true;
        initHeight = 1.45;
        //       conf.armPower = 30;
        break;
      case Copy:
      case Rescue:
        // conf.powerFactor = 1.25;
        break;
      case Bungee:
      case TwoBungee:
        // conf.powerFactor = .2;
        initHeight = 0.1;
        conf.powerFactor = 0.3;
        break;
      case Fight:
        // conf.powerFactor = 1.0;
        conf.useGripper=true;
        conf.gripDuration = 10;
        conf.releaseDuration = 5;
        conf.handsRotating = false;
        break;
      case FightFixed:
        conf.powerFactor = 0.2;
        // conf.useGripper=true;
        // conf.gripDuration = 10000;
        conf.handsRotating = false;
        conn_forces.set(4,1);
        break;
      case Table:
      case Stool:
      case TwoStools:
      case Trainer:
      case TwoTrainer:
        //       conf.powerFactor = 1.5;
        initHeight = 0.1;
        conf.handsRotating = false; // (type==Trainer || type==TwoTrainer);
        conf.useBackJoint = true;
        conf.backSideBend = true;
        fixedInAir = true;
        fixedSegm = Skeleton::Hip;
        break;
      }

      OdeHandle skelHandle=odeHandle;
      // skelHandle.substance.toMetal(1);
      //     skelHandle.substance.toPlastic(.5);//TEST sonst 40
      // skelHandle.substance.toRubber(5.00);//TEST sonst 40


      Skeleton* human = new Skeleton(skelHandle, skelOsgHandle,conf,
                                     "Humanoid" + itos(i) + "_" + typeToString(type) + "_" + ::name);
      if(type==Table){

        human->addSensor(std::make_shared<RelativePositionSensor>(2.0, 1.0, Sensor::XY),
                           Attachment(Skeleton::Left_Hand));
        human->addSensor(std::make_shared<RelativePositionSensor>(2.0, 1.0, Sensor::XY),
                           Attachment(Skeleton::Right_Hand));
        human->addSensor(std::make_shared<SpeedSensor>(10.0, SpeedSensor::RotationalRel, Sensor::Z),
                           Attachment(Skeleton::Thorax));
      }
      if(senseOther && !(secondBlind && i==1)){
        auto s = std::make_shared<SensorsOther>(18);
        sensorothers.push_back(s);
        if(visiondelay){
          auto sd = std::make_shared<DelaySensor>(s);
          human->addSensor(sd);
          human->addConfigurable(sd.get());
        }else{
          human->addSensor(s);
          human->addConfigurable(s.get());
        }
        useOtherPerception=true;
      }
      if(exterioception){
        human->addSensor(std::make_shared<SpeedSensor>(10.0, SpeedSensor::RotationalRel, Sensor::X),
                         Attachment(Skeleton::Hip));
 //        human->addSensor(std::make_shared<SpeedSensor>(1.0, SpeedSensor::TranslationalRel, Sensor::Y),
 //                          Attachment(Skeleton::Thorax));
 //       human->addSensor(std::make_shared<RelativePositionSensor>(1.0, 1.0, Sensor::Z),
        //                         Attachment(Skeleton::Hip));
        //        human->addSensor(std::make_shared<AxisOrientationSensor>(AxisOrientationSensor::ZProjection, Sensor::Z),
        //                         Attachment(Skeleton::Hip));
      }

      if(type==Copy || type==TwoBungee || type==TwoNormal){
        human->place( ROTM(M_PI_2,1,0,0) * ROTM( M_PI,0,0,1) // * ROTM( i%2==0 ? M_PI/5 : 0,0,0,1)
                              * TRANSM(2*i, 0, initHeight));
        // human->place( ROTM(M_PI_2,1,0,0) * TRANSM(1, 0, initHeight) * ROTM( i*M_PI,0,0,1)); // symmetric
      }else if((i==0 && env.withStool) || (i==1 && env.withStool2)) {
        human->place( ROTM(M_PI_2,1,0,0) * ROTM( M_PI,0,0,1)
                      * TRANSM(0, 0.15,-0.62)*(i==0 ? env.stoolPose : env.stool2Pose));
      } else{
        human->place( ROTM(M_PI_2,1,0,0)*ROTM( i%2==0 ? M_PI : 0,0,0,1)
                      //*TRANSM(.2*i,2*i,.841/*7*/ +2*i));
                      * ROTM( type ==FightFixed ? (i-0.5)*0.18 : tilt ,1,0,0)
                      * TRANSM(type==FightFixed ? 0 : 1*i,
                               type==FightFixed ? 0.35*i : 0.10*i, initHeight));
      }
      if(env.wheel4hands) human->addInspectable(env.wheel4hands);
      if(env.wheel4feet)  human->addInspectable(env.wheel4feet);

      if( fixedInAir){
        human->fixate(global, fixedSegm, 0);
        // Primitive* trunk = fixedSegm == -1 ? human->getMainPrimitive() : human->getAllPrimitives()[fixedSegm];

      }else if(reckturner){
        Primitive* leftHand = human->getAllPrimitives()[Skeleton::Left_Hand];
        Primitive* rightHand = human->getAllPrimitives()[Skeleton::Right_Hand];
        // reckX=leftHand->getPosition().x();
        reckY=leftHand->getPosition().y();
        createOrMoveReck(odeHandle, osgHandle.changeColor("wall"), global,
                         leftHand->getPosition().z());

        leftHandJoint = new SliderJoint(leftHand, reck->getMainPrimitive(), leftHand->getPosition(), Axis(1,0,0));
        leftHandJoint->init(odeHandle, osgHandle,false);
        rightHandJoint = new SliderJoint(rightHand, reck->getMainPrimitive(), rightHand->getPosition(), Axis(1,0,0));
        rightHandJoint->init(odeHandle, osgHandle,false);

        //       reck = new OSGCapsule(0.02,env.widthground/2);
        //       reck->init(osgHandle.changeColor("Silbergrau"), OSGPrimitive::Low);
        //       reck->setMatrix(ROTM(M_PI_2,0,1,0) * TRANSM((leftHand->getPosition() + rightHand->getPosition())*0.5));
      }

      // create pointer to one2onewiring
      AbstractWiring* wiring;
      if(noisecontrol>0){       wiring = new MotorNoiseWiring(new ColorUniformNoise(noisecontrol),1.0);
      }else
        wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      //ForceBoostWiring* wiring = new ForceBoostWiring(new ColorUniformNoise(0.1), 0.0);

      if(useOtherPerception && mergeSensors){
        CopyWiring::Assignment sa;
        CopyWiring::Assignment ma;
        sa.resize(18);
        ma.resize(18);
        for(int i=0; i<18;i++){
          sa[i]={i,18+i};
          ma[i]={i};
        }
        wiring = new CopyWiring(sa,ma,0);
      }


      AbstractController* controller;
      if(type==Copy && i>0){ // second robot gets remote controlled
        controller = new RemoteControlled();
      }else{
        if(useSox){
          SoxConf sc = Sox::getDefaultConf();
          sc.onlyMainParameters = false;
          sc.initFeedbackStrength=cInit;
          sc.useExtendedModel=true;
          controller = new Sox(sc);
          controller->setParam("Logarithmic",1);
          controller->setParam("epsC",0.0);
          controller->setParam("epsA",0.0);
          controller->setParam("sense",1.5);
        }else if(useSine || (i==1 && secondSine)){
          controller = new SineController();
          controller->setParam("amplitude",0.8);
        }else{
          DEPConf pc = DEP::getDefaultConf();
          pc.initFeedbackStrength=feedbackstrength;
          pc.useExtendedModel=false;
          pc.calcEigenvalues = !noEigenvalues;
          if(babbling) pc.initModel=false;

          controller = new DEP(pc);
          // DEP
          controller->setParam("epsC", 0.01);
          controller->setParam("epsA", 0.0);
          controller->setParam("test1",0.0);
          controller->setParam("test2",0.0);
          controller->setParam("urate",0.05);
          controller->setParam("synboost",1.4);
          controller->setParam("s4avg",s4avg);
          controller->setParam("evinterval",10);
          controller->setParam("epsh", 0);
          //     controller->setParam("sense",2);
        }
        if(fighterRandom && i==1){
          RandomDynConf conf = RandomDyn::getDefaultConf();
          conf.noiseGenC = new ColorNormalNoise(0.001);
          conf.noiseGenh = new ColorNormalNoise(0.01);
          controller = new RandomDyn(conf);
          controller->setParam("sigmaC",0.0001);
          controller->setParam("sigmah",0.0001);
        }
        if(noisecontrol>0){
          controller = new SineController();
          controller->setParam("amplitude",0);
        }
      }


      // create pointer to agent
      // initialize pointer with controller, robot and wiring
      // push agent in globel list of agents
      //      OdeAgent* agent = new OdeAgent(global, i==0 ? plotoptions : list<PlotOption>());
      OdeAgent* agent = new OdeAgent(global);

      if(i==0 && type==FightFixed)
        agent->addInspectableMatrix("forces",&conn_forces);

      agent->init(controller, human, wiring);
      //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps

      if(babbling){
        MotorBabbler* babbler=new MotorBabbler();
        babbler->setParam("minperiod",15);
        babbler->setParam("maxperiod",200);
        babbler->setParam("amplitude",1);
        babbler->setParam("resample",200);
        global.configs.push_back(babbler);
        agent->startMotorBabblingMode(babbling,babbler);
        agent->fixateRobot(global,-1,babbling/50);
      }

      switch(type){
      case Normal:
      case TwoNormal:
        agent->addOperator(new LimitOrientationOperator(Axis(0,0,-1), Axis(0,0,1),
                                                        M_PI/2.0, 60));
      case Fly:

      case Reck:
      case Copy:
      case Rescue:
        break;
      case Bungee:
      case TwoBungee: {
        controller->setParam("synboost",0.9);
        Pos p = human->getPosition();
        // agent->addOperator(new LimitOrientationOperator(Axis(0,0,-1), Axis(0,0,1),
        //                                                        M_PI/2.0, 50));
        p.z()=5;
        agent->addOperator(new PullToPointOperator(p,25,true,
                                                   fixBungee ?
                                                   PullToPointOperator::XYZ :
                                                   PullToPointOperator::Z,
                                                   0, 0.1, true));
        break;
      }
      case Fight:
        //agent->addOperator(new BoxRingOperator(Pos(0,0,2), 1.5, 0.2, 200, true));
        agent->addOperator(new BoxRingOperator(Pos(0,0,1.2), env.widthground/2.0,
                                               0.2, 200, false));
        // agent->addOperator(new PullToPointOperator(Pos(0,0,0.5),3,false,
        //                                            PullToPointOperator::XYZ,
        //                                            1, 0.0));
        break;
      case FightFixed:
        agent->addOperator(new BoxRingOperator(Pos(0,0,1.2), env.widthground/2.0,
                                               0.2, 1, false));
        break;
      case Table:
        break;
      case Stool:
        break;
      case TwoStools:
      case TwoTrainer:
        break;
      case Trainer:
        OdeHandle myHandle(odeHandle);
        env.wheel4hands->registerHand(human->getAllPrimitives()[Skeleton::Right_Hand], myHandle,0);
        env.wheel4hands->registerHand(human->getAllPrimitives()[Skeleton::Left_Hand], myHandle,1);
        env.wheel4hands->registerHand(human->getAllPrimitives()[Skeleton::Right_Forearm], myHandle,0);
        env.wheel4hands->registerHand(human->getAllPrimitives()[Skeleton::Left_Forearm], myHandle,1);
        if(env.wheel4feet){
          env.wheel4feet->registerHand(human->getAllPrimitives()[Skeleton::Right_Shin], myHandle,0);
          env.wheel4feet->registerHand(human->getAllPrimitives()[Skeleton::Left_Shin], myHandle,1);

        }

        break;
      }

      if(controllerfile[i]){
        if(controller->restoreFromFile(controllerfile[i])){
          printf("restoring controller: %s\n", controllerfile[i]);
        } else{
          fprintf(stderr,"restoring of controller failed!\n");
          exit(1);
        }
      }


      // save robot
      if(i==0) human->storeToFile(ROBOTSTOREFILE);

      global.configs.push_back(agent);
      global.agents.push_back(agent);

      //
    }// Several humans end

    // connect mutual SensorsOther
    if(sensorothers.size()>=1 && global.agents.size()>1){
      Skeleton* h1 = dynamic_cast<Skeleton*>(global.agents[0]->getRobot());
      Skeleton* h2 = dynamic_cast<Skeleton*>(global.agents[1]->getRobot());
      sensorothers[0]->setOther(h2);
      if(sensorothers.size()>1) sensorothers[1]->setOther(h1);
    }

    // connect gripper
    connector[0]=connector[1]=connector[2]=connector[3]=0;
    if(type==Fight || type == FightFixed ){
      Skeleton* h1 = dynamic_cast<Skeleton*>(global.agents[0]->getRobot());
      Skeleton* h2 = dynamic_cast<Skeleton*>(global.agents[1]->getRobot());
      if(h1 && h2){
        if(type==Fight){  // connect grippers
          FOREACH(GripperList, h1->getGrippers(), g){
            (*g)->addGrippables(h2->getAllPrimitives());
          }
          FOREACH(GripperList, h2->getGrippers(), g){
            (*g)->addGrippables(h1->getAllPrimitives());
          }
        }else{ // connect hands and feet manually
          OdeHandle myOdeHandle(odeHandle);
          Primitive *p1, *p2;
          p1=h1->getAllPrimitives()[Skeleton::Left_Hand];
          p2=h2->getAllPrimitives()[Skeleton::Right_Hand];
          connector[0] = new BallJoint(p1,p2,(p1->getPosition() + p2->getPosition())/2);
          connector[0]->init(odeHandle, osgHandle, false);
          myOdeHandle.addIgnoredPair(p1,h2->getAllPrimitives()[Skeleton::Right_Forearm]);
          myOdeHandle.addIgnoredPair(p2,h1->getAllPrimitives()[Skeleton::Left_Forearm]);
          p1=h1->getAllPrimitives()[Skeleton::Right_Hand];
          p2=h2->getAllPrimitives()[Skeleton::Left_Hand];
          connector[1] = new BallJoint(p1,p2,(p1->getPosition() + p2->getPosition())/2);
          connector[1]->init(odeHandle, osgHandle, false);
          myOdeHandle.addIgnoredPair(p1,h2->getAllPrimitives()[Skeleton::Left_Forearm]);
          myOdeHandle.addIgnoredPair(p2,h1->getAllPrimitives()[Skeleton::Right_Forearm]);
          p1=h1->getAllPrimitives()[Skeleton::Left_Foot];
          p2=h2->getAllPrimitives()[Skeleton::Right_Foot];
          connector[2] = new BallJoint(p1,p2,(p1->getPosition() + p2->getPosition())/2);
          connector[2]->init(odeHandle, osgHandle, false);
          p1=h1->getAllPrimitives()[Skeleton::Right_Foot];
          p2=h2->getAllPrimitives()[Skeleton::Left_Foot];
          connector[3] = new BallJoint(p1,p2,(p1->getPosition() + p2->getPosition())/2);
          connector[3]->init(odeHandle, osgHandle, false);
          for(int k=0; k<4; k++){
            dJointFeedback* fb=new dJointFeedback;
            dJointSetFeedback(connector[k]->getJoint(),fb);
          }
        }
      }else{
        fprintf(stderr,"Cannot convert Humanoids!");
      }
    }
  };


  void createOrMoveReck(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                        GlobalData& global, double amount){
    if(type==Reck) {
      if(fixator) delete fixator;
      if(!reck){
        reck = new PassiveCapsule(odeHandle, osgHandle,
                                  0.02,env.widthground-0.2, 1.0);
        reck->setPose(ROTM(M_PI_2,0,1,0)*TRANSM(reckX, reckY,0));
        global.obstacles.push_back(reck);
        reckZ = amount;
      }else{
        reckZ += amount;
      }

      Primitive* r = reck->getMainPrimitive();
      r->setPose(ROTM(M_PI_2,0,1,0)*TRANSM(reckX,reckY,reckZ));
      fixator = new FixedJoint(r, global.environment);
      fixator->init(odeHandle, osgHandle, false);
    }
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    if(reck){
      au.addKeyboardMouseBinding("Sim: b/B","move high bar down/up");
    }
  }

  virtual void addCallback(GlobalData& global, bool draw, bool pause, bool control) {
    env.update();
    if(control){
      if (type==Copy){
        if(global.agents.size()>1){
          Teachable* c = dynamic_cast<Teachable*>(global.agents[0]->getController());
          assert(c);
          RemoteControlled* rc = dynamic_cast<RemoteControlled*>(global.agents[1]->getController());
          assert(rc);

          rc->remoteControl(c->getLastMotorValues());
        }
      }
      if(type==FightFixed){
        for(int k=0; k<4; k++){
          dJointFeedback* fb = dJointGetFeedback(connector[k]->getJoint());
          conn_forces.val(k,0) = Pos(fb->f1).length();
        }
      }
    }
    if(type==Table){
      Skeleton* h = dynamic_cast<Skeleton*>(global.agents[0]->getRobot());
      if(h){
        int index[2] = {Skeleton::Right_Hand, Skeleton::Left_Hand};
        for(int i : index){
          Primitive* table = env.table->getMainPrimitive();
          Primitive* hand = h->getAllPrimitives()[i];
          Pos tablepos = table->getPosition();
          tablepos.z() += 0.05;
          Pos dir = tablepos-hand->getPosition();
          // range
          // if(fabs(dir.x())< 0.85) dir.x()=0;
          // if(fabs(dir.y())< 0.45) dir.y()=0;
          if(dir.z()>0){
            dir.x()=0;
            dir.y()=0;
            dir.z()*=tableForce;
            hand->applyForce(dir);
            cout << "push hand  " << i << endl;
          }
          //            double strength = dir.length()*tableForce;
          //            hand->applyForce(dir);
        }
      }
    }else if(type==Trainer || type==TwoTrainer){
      Skeleton* h = dynamic_cast<Skeleton*>(global.agents[0]->getRobot());
      if(h){
        env.wheel4hands->attract(h->getAllPrimitives()[Skeleton::Right_Hand],0, 0,
                                 odeHandle, osgHandle, global.time);
        env.wheel4hands->attract(h->getAllPrimitives()[Skeleton::Left_Hand], 1, 0,
                                 odeHandle, osgHandle, global.time);
        if(env.wheel4feet){
          env.wheel4feet->attract(h->getAllPrimitives()[Skeleton::Right_Foot],0, 0,
                                  odeHandle, osgHandle, global.time);
          env.wheel4feet->attract(h->getAllPrimitives()[Skeleton::Left_Foot], 1, 0,
                                  odeHandle, osgHandle, global.time);

        }
      }
      if(type==TwoTrainer){
        h = dynamic_cast<Skeleton*>(global.agents[1]->getRobot());
        if(h){
          env.wheel4hands->attract(h->getAllPrimitives()[Skeleton::Right_Hand],0, 1, odeHandle, osgHandle, global.time);
          env.wheel4hands->attract(h->getAllPrimitives()[Skeleton::Left_Hand], 1, 1, odeHandle, osgHandle, global.time);
        }
      }
    }

  };

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       GlobalData& global, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'x':{
          OdeAgent* agent = getWatchedAgent();
          if(agent) {
            if(!agent->unfixateRobot(global)){
              agent->fixateRobot(global, fixedSegm, 0);
            }
          }
          //          globalData.agents[0]->setParam("pelvispower",0.0);
          return true;
        }
        case 'X':
          env.removeStools(global);
          for(auto& a: global.agents){
            a->fixateRobot(global, Skeleton::Left_Foot, 0);
          }
          return true;
        case 'b':
          createOrMoveReck(odeHandle, osgHandle, globalData, -0.1);
          return true;
        case 'B':
          createOrMoveReck(odeHandle, osgHandle, globalData,  0.1);
          return true;
        case 'l':
          {
            globalData.agents[0]->getRobot()->restoreFromFile(ROBOTSTOREFILE);
            globalData.agents[0]->getWiring()->reset();
          }
          return true;
        case 'd':
        case 'D':
          if(env.wheel4hands){
            env.wheel4hands->wheel->applyTorque(((char)key == 'D' ? -1 : 1)*env.wheel4hands->mass
                                                *(env.wheel4hands->forTwo? 3 : 1),0,0);
            global.addTmpObject(new TmpDisplayItem(new OSGCapsule(env.wheel4hands->cranklength/4,
                                                                  env.wheel4hands->width*1.5),
                                                   env.wheel4hands->wheel->getPose(),
                                                   Color(((char)key == 'D' ? 0 : 1),
                                                         ((char)key == 'D' ? 1 : 0),0)),
                                0.5);
          }
          return true;
        case 'k':
        case 'K':
          if(env.wheel4feet){
            env.wheel4feet->wheel->applyTorque(((char)key == 'K' ? -1 : 1)*env.wheel4feet->mass,0,0);
            global.addTmpObject(new TmpDisplayItem(new OSGCapsule(env.wheel4feet->cranklength/4,
                                                                  env.wheel4feet->width*1.5),
                                                   env.wheel4feet->wheel->getPose(),
                                                   Color(((char)key == 'K' ? 0 : 1),
                                                         ((char)key == 'K' ? 1 : 0),0)),
                                0.5);
          }
          return true;
        case 'i':
          if(playground) {
            s.hardness*=1.5;
            cout << "hardness " << s.hardness << endl;

            playground->setSubstance(s);
          }
          return true;
        case 'j':
          if(playground) {
            s.hardness/=1.5;
            cout << "hardness " << s.hardness << endl;
            playground->setSubstance(s);
          }
          return true;
        default:
          return false;
        }
    }

    return false;
  }

  virtual void usage() const {
    printf("\t-rescue\thumanoid in a pit\n");
    printf("\t-reck\thumanoid at a reck bar\n");
    printf("\t-fight\ttwo humanoid in a fighting arena\n");
    printf("\t-fightfixed\ttwo humanoid in a fighting arena, grasping their hands permanently\n");
    printf("\t-bungee\ta weak humanoid attached to bungee\n");
    printf("\t-copy\tfight situation where one robot gets remoted controlled\n");
    printf("\t-noise strength\tnoise strength (def: 0.01)\n");
    printf("\t-cInit strength\tinitial value for Controller diagonal (def: 1.0)\n");
    printf("\t-tilt angle\tangle in rad by which to tilt the robot initialially (def: 0.0)\n");
    printf("\t-powerfactor factor\tpowerfactor of robot (def: 1.0)\n");
    printf("\t-name NAME\tname of experiment (def: )\n");
    printf("\t-sox\tuse SOX controller (Homeokinesis) instead of DEP\n");
    printf("\t-loadcontroler file \tload the controller at startup\n");
    printf("\t-loadcontroler1 file \tload the controller for second humanoid at startup\n");
    printf("\t-noisecontrol tau \tuse noise as motor commands with correlation length 1/tau \n");
    printf("\t-fighterrandom\tsecond fighter has random dynamics \n");
  };

};

int main (int argc, char **argv)
{
  SimType type=Normal;
  if (Simulation::contains(argv, argc, "-two")) {
    type= TwoNormal;
  }
  if (Simulation::contains(argv, argc, "-rescue")) {
    type= Rescue;
  }
  if (Simulation::contains(argv, argc, "-reck")) {
    type= Reck;
  }
  if (Simulation::contains(argv, argc, "-fight")) {
    type= Fight;
  }
  if (Simulation::contains(argv, argc, "-fightfixed")) {
    type= FightFixed;
  }
  if (Simulation::contains(argv, argc, "-table")) {
    type= Table;
  }
  if (Simulation::contains(argv, argc, "-trainer")) {
    type= Trainer;
  }
  if (Simulation::contains(argv, argc, "-beside")) {
    beside=true;
  }
  if (Simulation::contains(argv, argc, "-trainer2")) {
    type= TwoTrainer;
  }
  if (Simulation::contains(argv, argc, "-stool")) {
    type= Stool;
  }
  if (Simulation::contains(argv, argc, "-2stools")) {
    type= TwoStools;
  }
  senseOther     = Simulation::contains(argv, argc, "-senseother");
  secondBlind    = Simulation::contains(argv, argc, "-2ndblind");
  secondSine     = Simulation::contains(argv, argc, "-2ndsine");
  visiondelay    = Simulation::contains(argv, argc, "-visiondelay");
  senseOther     = Simulation::contains(argv, argc, "-senseother");
  fixBungee      = Simulation::contains(argv, argc, "-fixbungee");
  exterioception = Simulation::contains(argv, argc, "-exterioception");
  trainer4feet   = Simulation::contains(argv, argc, "-trainer4feet");
  trainersameside   = Simulation::contains(argv, argc, "-trainersameside");
  trainer4feetsameside   = Simulation::contains(argv, argc, "-trainer4feetsameside");
  noEigenvalues  = Simulation::contains(argv, argc, "-noeigenvalues");
  // the sensors of other are added to the own sensors
  mergeSensors   = Simulation::contains(argv, argc, "-mergesensors");
  if (Simulation::contains(argv, argc, "-bungee")) {
    type= Bungee;
  }
  if (Simulation::contains(argv, argc, "-2bungee")) {
    type= TwoBungee;
  }

  if (Simulation::contains(argv, argc, "-copy")) {
    type= Copy;
  }
  if (Simulation::contains(argv, argc, "-fly")) {
    type= Fly;
  }
  int index = Simulation::contains(argv, argc, "-noise");
  if (index>0 && index<argc) {
    noise=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-cInit");
  if (index>0 && index<argc) {
    cInit=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-name");
  if (index>0 && index<argc) {
    name=string(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-tilt");
  if (index>0 && index<argc) {
    tilt=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-powerfactor");
  if (index>0 && index<argc) {
    powerfactor=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-noisecontrol");
  if (index>0 && index<argc) {
    noisecontrol=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-feedbackstrength");
  if (index>0 && index<argc) {
    feedbackstrength=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-s4avg");
  if (index>0 && index<argc) {
    s4avg=atof(argv[index]);
  }
  index = Simulation::contains(argv, argc, "-loadcontroller");
  if (index>0 && index<argc) {
    controllerfile[0]=argv[index];
  }
  index = Simulation::contains(argv, argc, "-loadcontroller1");
  if (index>0 && index<argc) {
    controllerfile[1]=argv[index];
  }
  index = Simulation::contains(argv, argc, "-wheelmass");
  if (index>0 && index<argc) {
    wheelmass=atof(argv[index]);
  }
  index = Simulation::contains(argv,argc,"-babbling");
  if(index >0 && argc>index){
    babbling=atoi(argv[index]);
  }

  useSox = (Simulation::contains(argv, argc, "-sox"));
  useSine = (Simulation::contains(argv, argc, "-sine"));
  fighterRandom= (Simulation::contains(argv, argc, "-fighterrandom"));

  ThisSim sim(type);
  return sim.run(argc, argv) ? 0 : 1;

}



// some commands for video production
// for F in comparison_*; do pushd `pwd`; cd $F/humanoid*src; feedfile.pl 1 < *.log | matrixviz ; popd; done
// for F in comparison_*; do pushd `pwd`; cd $F/; T=`ls -d humanoid*src/`; cd $T/comparison*/Humanoid0_Bungee_; encodevideo.sh  CC/fr ../../../${T%_src/}_CC;  popd; done
