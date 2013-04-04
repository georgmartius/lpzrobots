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
#include <stdio.h>

#include <selforg/noisegenerator.h>
#include <selforg/sinecontroller.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/forceboostwiring.h>
#include <selforg/derivativewiring.h>

#include <ode_robots/skeleton.h>

#include <ode_robots/joint.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/addsensors2robotadapter.h>

#include <ode_robots/operators.h>
#include <ode_robots/tmpprimitive.h>


#include <selforg/soml.h>
#include <selforg/sox.h>
//#include <selforg/pimax.h>
#include <selforg/remotecontrolled.h>
#include "pimax.h"

#include "environment.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

#define ROBOTSTOREFILE "humanoid_initial.rob"

enum SimType { Normal, Rescue, Fight, Reck, Bungee, Copy };
string typeToString(SimType t){
  switch(t){
  case Normal:
    return "Normal";
  case Reck:
    return "Reck";
  case Rescue:
    return "Rescue";
  case Bungee:
    return "Bungee";
  case Fight:
    return "Fight";
  case Copy:
    return "Copy";
  }
  return "unknown";
}
double noise=0.01;
double cInit=1.0;
string name="";

class ThisSim : public Simulation {
public:
  SimType type;
  Env env;

  Joint* fixator;
  Joint* reckLeft;
  Joint* reckRight;
  PassiveCapsule* reck;
  Playground* playground;
  //  AbstractObstacle* playground;
  double hardness;
  Substance s;

  ThisSim(SimType type)
    : type(type){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    if(type==Rescue)
      addColorAliasFile("overwriteGroundColor.txt");
    setGroundTexture("Images/whiteground.jpg");
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos (Pos(3.96562, 9.00244, 2.74255),  Pos(157.862, -12.7123, 0));

    setCameraMode(Static);

    int humanoids = 1;
    bool useExtendedModel = false;

    bool fixedInAir = false;
    env.type=Env::Normal;
    global.odeConfig.setParam("noise",noise); //for more variety
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.01);
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("gravity", -6);

    switch(type){
    case Normal:
      fixedInAir = false;
      global.odeConfig.setParam("controlinterval",3);
      env.numSeeSaws = 0;
      env.roughness  = 2.5;
      break;
    case Bungee:
      break;
    case Reck:
      env.height=2.0;
      break;
    case Copy:
      humanoids       = 2;
    case Rescue:
      global.odeConfig.setParam("controlinterval",2);
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
      env.type        = Env::Normal;
      global.odeConfig.setParam("controlinterval",3);
      global.odeConfig.setParam("gravity", -4);
      env.widthground = 5;
      env.height      = 0.5;
      env.roughness   = 3;
      humanoids       = 2;
      env.numSeeSaws  = 0;
      break;
    }

    env.create(odeHandle, osgHandle, global);

    env.numSpheres  = 0;
    env.numBoxes    = 0;
    env.numCapsules = 0;
    env.placeObstacles(odeHandle, osgHandle, global);

    global.configs.push_back(&env);
    //    global.configs.push_back(this);

    fixator=0;
    reckLeft = reckRight = 0;
    reck = 0;

   for (int i=0; i< humanoids; i++){ //Several humans
     bool reckturner = (type==Reck);
     if (i>0) reckturner=false;

     // normal servos
     // SkeletonConf conf = Skeleton::getDefaultConf();
     // velocity servos
     SkeletonConf conf = Skeleton::getDefaultConfVelServos();

     OsgHandle skelOsgHandle=osgHandle.changeColorSet((type==Copy && i==1) ? 5 : i);
     double initHeight=0.8;

     conf.useBackJoint = true;
     conf.powerFactor = 1;

     switch(type){
     case Normal:
       conf.powerFactor = 1.5;
       conf.dampingFactor = .0;
       useExtendedModel = false;
       initHeight = 0.5;
       break;
     case Reck:
       conf.powerFactor = 0.3;
       conf.relForce = 2;
       conf.dampingFactor = .0;
       conf.handsRotating = true;
       initHeight = 0.45;
       //       conf.armPower = 30;
       break;
     case Copy:
     case Rescue:
       conf.powerFactor = 1.25;
       conf.dampingFactor = .0;
       break;
     case Bungee:
       conf.powerFactor = .2;
       conf.dampingFactor = .0;
       break;
     case Fight:
       conf.powerFactor = 1.0;
       conf.useGripper=true;
       conf.dampingFactor = .0;
       conf.gripDuration = 10;
       conf.releaseDuration = 5;
       break;
     }

     OdeHandle skelHandle=odeHandle;
     // skelHandle.substance.toMetal(1);
     //     skelHandle.substance.toPlastic(.5);//TEST sonst 40
     // skelHandle.substance.toRubber(5.00);//TEST sonst 40


     Skeleton* human0 = new Skeleton(skelHandle, skelOsgHandle,conf,
                                     "Humanoid" + itos(i) + "_" + typeToString(type) + "_" + ::name);
     // to add sensors use these lines
     //std::list<Sensor*> sensors;
     // additional sensor
     //     sensors.push_back(new AxisOrientationSensor(AxisOrientationSensor::OnlyZAxis));
     //     AddSensors2RobotAdapter* human =
     //     new AddSensors2RobotAdapter(skelHandle, osgHandle, human0, sensors);

     OdeRobot* human = human0;
     if(type==Copy){
       human->place( ROTM(M_PI_2,1,0,0) * ROTM( i%2==0 ? M_PI/5 : 0,0,0,1)
                     * TRANSM(2*i, 0, initHeight));
     }else{
       human->place( ROTM(M_PI_2,1,0,0)*ROTM( i%2==0 ? M_PI : 0,0,0,1)
                     //*TRANSM(.2*i,2*i,.841/*7*/ +2*i));
                     * TRANSM(1*i, 0.10*i, initHeight));
     }

     if( fixedInAir){
       Primitive* trunk = human->getMainPrimitive();

       fixator = new FixedJoint(trunk, global.environment);
       //       // fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) ,                    Axis(0,0,1), Axis(0,1,0));
       fixator->init(odeHandle, osgHandle);
     }else if(reckturner){
       Primitive* leftHand = human->getAllPrimitives()[Skeleton::Left_Hand];
       Primitive* rightHand = human->getAllPrimitives()[Skeleton::Right_Hand];

       createOrMoveReck(odeHandle, osgHandle.changeColor("wall"), global,
                        leftHand->getPosition().z());

       reckLeft = new SliderJoint(leftHand, reck->getMainPrimitive(), leftHand->getPosition(), Axis(1,0,0));
       reckLeft->init(odeHandle, osgHandle,false);
       reckRight = new SliderJoint(rightHand, reck->getMainPrimitive(), rightHand->getPosition(), Axis(1,0,0));
       reckRight->init(odeHandle, osgHandle,false);

       //       reck = new OSGCapsule(0.02,env.widthground/2);
       //       reck->init(osgHandle.changeColor("Silbergrau"), OSGPrimitive::Low);
       //       reck->setMatrix(ROTM(M_PI_2,0,1,0) * TRANSM((leftHand->getPosition() + rightHand->getPosition())*0.5));
     }

     // create pointer to one2onewiring
     One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
     //ForceBoostWiring* wiring = new ForceBoostWiring(new ColorUniformNoise(0.1), 0.0);


     AbstractController* controller;
     if(type==Copy && i>0){ // second robot gets remote controlled
       controller = new RemoteControlled();
     }else{
       //         controller = new Sox(cInit, useExtendedModel);
       PiMaxConf pc = PiMax::getDefaultConf();
       pc.onlyMainParameters = false;
       pc.initFeedbackStrength=cInit;
       controller = new PiMax(pc);
       //       controller = new SineController();
     }
     // pimax
     controller->setParam("matrics",1);
     controller->setParam("epsC",0.02);
     controller->setParam("sense",2);
     controller->setParam("s4avg",1);
     controller->setParam("s4delay",1);

     switch(type){
     case Normal:
       break;
     case Reck:
       break;
     case Rescue:
       break;
     case Bungee:
       break;
     case Fight:
     case Copy:
       controller->setParam("damping",0.0003);
       break;
     }



     // create pointer to agent
     // initialize pointer with controller, robot and wiring
     // push agent in globel list of agents
     OdeAgent* agent = new OdeAgent(global, i==0 ? plotoptions : list<PlotOption>());
     agent->init(controller, human, wiring);
     //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps

    // agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1),
    //                                                 M_PI*0.4, 1));
     switch(type){
     case Normal:
       break;
     case Reck:
       break;
     case Copy:
     case Rescue:
       break;
     case Bungee:
       agent->addOperator(new PullToPointOperator(Pos(0,0,3),30,true,
                                                  PullToPointOperator::Z,
                                                  0, 0.1, true));
       break;
     case Fight:
       //agent->addOperator(new BoxRingOperator(Pos(0,0,2), 1.5, 0.2, 200, true));
       agent->addOperator(new BoxRingOperator(Pos(0,0,1.2), env.widthground/2.0,
                                              0.2, 200, false));
       // agent->addOperator(new PullToPointOperator(Pos(0,0,0.5),3,false,
       //                                            PullToPointOperator::XYZ,
       //                                            1, 0.0));

       break;
     }

     // save robot
     if(i==0) human->storeToFile(ROBOTSTOREFILE);

     global.configs.push_back(agent);
     global.agents.push_back(agent);

     //
   }// Several humans end

   // connect grippers
   if(type==Fight ){
     Skeleton* h1 = dynamic_cast<Skeleton*>(global.agents[0]->getRobot());
     Skeleton* h2 = dynamic_cast<Skeleton*>(global.agents[1]->getRobot());
     if(h1 && h2){
        FOREACH(GripperList, h1->getGrippers(), g){
          (*g)->addGrippables(h2->getAllPrimitives());
        }
        FOREACH(GripperList, h2->getGrippers(), g){
          (*g)->addGrippables(h1->getAllPrimitives());
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
                                  0.02,env.widthground, 1.0);
        reck->setPose(ROTM(M_PI_2,0,1,0));
        global.obstacles.push_back(reck);

      }
      Primitive* r = reck->getMainPrimitive();
      r->setPosition(r->getPosition()+Pos(0.,0,amount));
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
    if(control && type==Copy){
      if(global.agents.size()>1){
        Teachable* c = dynamic_cast<Teachable*>(global.agents[0]->getController());
        assert(c);
        RemoteControlled* rc = dynamic_cast<RemoteControlled*>(global.agents[1]->getController());
        assert(rc);

        rc->remoteControl(c->getLastMotorValues());
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
        case 'X':
        case 'x':
          if(fixator) delete fixator;
          fixator=0;
          //          globalData.agents[0]->setParam("pelvispower",0.0);
          return true;
          break;
        case 'b':
          createOrMoveReck(odeHandle, osgHandle, globalData, -0.1);
          return true;
          break;
        case 'B':
          createOrMoveReck(odeHandle, osgHandle, globalData,  0.1);
          return true;
          break;
        case 'l':
          {
            globalData.agents[0]->getRobot()->restoreFromFile(ROBOTSTOREFILE);
            globalData.agents[0]->getWiring()->reset();
          }
          return true;
          break;
        case 'i':
          if(playground) {
            s.hardness*=1.5;
            cout << "hardness " << s.hardness << endl;
            playground->setSubstance(s);
          }
          return true;
          break;
        case 'j':
          if(playground) {
            s.hardness/=1.5;
            cout << "hardness " << s.hardness << endl;
            playground->setSubstance(s);
          }
          return true;
          break;
        default:
          return false;
          break;
        }
    }
    return false;
  }

  virtual void usage() const {
    printf("\t-rescue\thumanoid in a pit\n");
    printf("\t-reck\thumanoid at a reck bar\n");
    printf("\t-fight\ttwo humanoid in a fighting arena\n");
    printf("\t-bungee\ta weak humanoid attached to bungee\n");
    printf("\t-copy\tfight situation where one robot gets remoted controlled\n");
    printf("\t-noise strength\tnoise strength (def: 0.01)\n");
    printf("\t-cInit strength\tinitial value for Controller diagonal (def: 1.0)\n");
    printf("\t-name NAME\tname of experiment (def: )\n");
  };

};

int main (int argc, char **argv)
{
  SimType type=Normal;
  if (Simulation::contains(argv, argc, "-rescue")) {
    type= Rescue;
  }
  if (Simulation::contains(argv, argc, "-reck")) {
    type= Reck;
  }
  if (Simulation::contains(argv, argc, "-fight")) {
    type= Fight;
  }
  if (Simulation::contains(argv, argc, "-bungee")) {
    type= Bungee;
  }
  if (Simulation::contains(argv, argc, "-copy")) {
    type= Copy;
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

  ThisSim sim(type);
  return sim.run(argc, argv) ? 0 : 1;

}


// ./start -r 1111 -noise 0.01 -cInit 0.95 -f 100 ntst -simtime 10 -name "control"
// ./start -r 2222 -noise 0.01 -cInit 0.95 -f 100 ntst -simtime 10 -name "differentseed"
// ./start -r 1111 -noise 0.01 -cInit 0.95 -f 100 ntst -simtime 10 -name "" -rescue
// ./start -r 2222 -noise 0.01 -cInit 0.95 -f 100 ntst -simtime 10 -name "differentseed" -rescue
