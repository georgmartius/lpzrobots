/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *
 ***************************************************************************/
#include <stdio.h>
#include <selforg/noisegenerator.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>

// controller
//#include <selforg/sox.h>
#include "soml.h"

#include <selforg/motorbabbler.h>
#include <selforg/derlininvert.h>
#include <selforg/dercontroller.h>
#include <selforg/sinecontroller.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include <ode_robots/joint.h>

// used arena
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivebox.h>

#include <ode_robots/sliderwheelie.h>
#include <ode_robots/plattfussschlange.h>
#include <ode_robots/schlangeservo.h>
#include <ode_robots/schlangeservo2.h>
// used robot
#include "skeleton.h" // if robot is local
//#include <ode_robots/skeleton.h>
#include "hexapod.h" // if robot is local



// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:
  enum Grounds { Normal, Octa, Pit, Uterus, Stacked };

  // playground parameter
  constexpr static double widthground = 25.85;// 100; //1.3;
  constexpr static double heightground = .8;// 1.2;
  constexpr static double diamOcta = 2;
  constexpr static double pitsize = 1;
  constexpr static double pitheight = 2;
  constexpr static double uterussize = 1;

  OdeRobot* robot;


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos (Pos(3.46321, 10.6081, 2.74255),  Pos(161.796, -3.69849, 0));

  // Set number of robots:
    // int plattfuesse = 1;
    int flatsnakes  = 0;
    int snakes = 0;
    //int sphericalsIR = 0;
    //int sphericalsXYZ = 0;
    //int hurlings = 0;
    //int cigars = 0;
    int wheelies = 0;
    int humanoids = 1;
       // int barrel=1;
    // int dogs = 0;

    robot=0;

    bool fixedInAir = true;
    reckturner = false;
    // Playground types
    addParameterDef("centerforce", &centerforce, 7.0);//2.0
    addParameterDef("forwardforce", &forwardforce, 0.0);
    center = osg::Vec3(0,0,3);
    forcepoint = 0;
    global.configs.push_back(this);

    fixator=0;
    reckLeft = reckRight = 0;
    reck = 0;

    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",5);//4);
    global.odeConfig.setParam("noise",0.0);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.004);//0.004);
    global.odeConfig.setParam("gravity", -6);
    //    global.odeConfig.setParam("cameraspeed", 250);
    //  int chessTexture = dsRegisterTexture("chess.ppm");


    //************************* SELECT PLAYGROUND HERE ******************?
    setupPlaygrounds(odeHandle, osgHandle, global,  Normal);



   for (int i=0; i< humanoids; i++){ //Several humans
     if (i>0) reckturner=false;

     //       ZweiBeinerConf conf = ZweiBeiner::getDefaultConf();

     //       ZweiBeiner* human = new ZweiBeiner(odeHandle, osgHandle,conf, "Humanoid");
     //       human->place(osg::Matrix::translate(4*i,0,2));
     //       global.configs.push_back(human);

     //       Primitive* trunk = human->getMainPrimitive();
     //       fixator = new FixedJoint(trunk, global.environment);
     //       fixator->init(odeHandle, osgHandle);

     // normal servos
     //SkeletonConf conf = Skeleton::getDefaultConf();
     // velocity servos
     SkeletonConf conf = Skeleton::getDefaultConfVelServos();

     // conf.massfactor   = 1;
     // conf.relLegmass = 1;
     // conf.relFeetmass = 1;
     // conf.relArmmass = 1;//1.0;

     //       conf.ankleJointLimit=0.001; //!
     //     conf.pelvisPower=20;
     // if(reckturner)      conf.armPower = 30;
     conf.onlyPrimaryFunctions=false;
     conf.powerFactor = .15 ; //.15;// .95;//.65;//5;
     if (reckturner) conf.powerFactor *=.2;
     if (i==0)
       conf.trunkColor=Color(0.1, 0.3, 0.8);
     else
       conf.trunkColor=Color(0.75, 0.1, 0.1);
     if(reckturner)
       conf.handsRotating = true;

     conf.useBackJoint = true;
     conf.jointLimitFactor = 1;

     //     conf.irSensors = true;

     //    conf.bodyTexture="Images/whitemetal_farbig_small.rgb";
     //     conf.bodyColor=Color(1.0,1.0,0.0);
     OdeHandle skelHandle=odeHandle;
     // skelHandle.substance.toMetal(1);
    //     skelHandle.substance.toPlastic(.5);//TEST sonst 40
     // skelHandle.substance.toRubber(5.00);//TEST sonst 40
     Skeleton* human = new Skeleton(skelHandle, osgHandle,conf, "Humanoid");
     //     human->addSensor(new AxisOrientationSensor(AxisOrientationSensor::OnlyZAxis))
     human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)
                  //   *osg::Matrix::translate(-.2 +2.9*i,0,1));
                  *osg::Matrix::translate(.2*i,2*i,.841/*7*/ +2*i));


     if( fixedInAir){
       Primitive* trunk = human->getMainPrimitive();

       fixator = new FixedJoint(trunk, global.environment);
       //       // fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) ,                    Axis(0,0,1), Axis(0,1,0));
       fixator->init(odeHandle, osgHandle);
     }else if(reckturner){
       Primitive* leftHand = human->getAllPrimitives()[Skeleton::Left_Hand];
       Primitive* rightHand = human->getAllPrimitives()[Skeleton::Right_Hand];

       reckLeft = new SliderJoint(leftHand, global.environment, leftHand->getPosition(), Axis(1,0,0));
       reckLeft->init(odeHandle, osgHandle,false);
       reckRight = new SliderJoint(rightHand, global.environment, rightHand->getPosition(), Axis(1,0,0));
       reckRight->init(odeHandle, osgHandle,false);
     }



     SoMLConf sc = SoML::getDefaultConf();

     sc.useHiddenContr=true;
     sc.useHiddenModel=true;
     sc.someInternalParams=true;
     sc.useS=true;
     AbstractController* controller = new SoML(sc);
     controller->setParam("epsC",0.05);
     controller->setParam("epsA",0.05);
     controller->setParam("harmony",0.0);
     controller->setParam("s4avg",3.0);

     //     AbstractController* controller = new BasicController(cc);
     //   AbstractController* controller = new SineController(1<<14); // only motor 14
     //controller = new MotorBabbler();
     // controller = new SineController(/*,SineController::Impulse*/);
     // controller->setParam("phaseshift",0);
     // controller->setParam("period",100);
     // controller->setParam("amplitude",.7);


     // create pointer to one2onewiring
     One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
     // DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
     // c.useId = true;
     // c.useFirstD = false;
     // DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise(.2) );

     OdeAgent* agent = new OdeAgent(global);
     agent->init(controller, human, wiring);
     // agent->startMotorBabblingMode(5000);
     //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
     global.agents.push_back(agent);
     global.configs.push_back(agent);
   }// Several humans end





    //****** SNAKES **********/
    //creation of normal   snakes
   for(int i=0; i<snakes; i++){

     //****************/
     SchlangeConf conf = Schlange::getDefaultConf();
     double snakesize = .6;
     conf.segmMass   = 1.8;
     conf.segmLength= 1.0 * snakesize;// 0.8;
     conf.segmDia=.15 *snakesize;
     conf.motorPower= 2 * snakesize;
     conf.segmNumber = 12+1*i;//-i/2;
     // conf.jointLimit=conf.jointLimit*3;
     conf.jointLimit=conf.jointLimit* 1.6;
     conf.frictionJoint=0.001;
     //PlattfussSchlange* schlange1;
     SchlangeServo2* schlange1;
     if (i==0) {
       schlange1 =

         new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.1, 0.3, 0.8)),
                              //  new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(1.0, 1.0, 1.0)),
                              conf, "S1");
     } else {
       schlange1 =
         new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.1, 0.3, 0.8)),
                              // new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(0.8, 0.4, .3)),
                              conf, "S2");
     }
     //Positionieren und rotieren
     schlange1->place(osg::Matrix::rotate(M_PI/2,0, 1, 0)*
                      osg::Matrix::translate(21 -.7*i,21-2*i,.001+(i+1)*(.2+conf.segmNumber)/2.0/*+2*/));
     // osg::Matrix::translate(5-i,2 + i*2,height+2));
     schlange1->setTexture("Images/whitemetal_farbig_small.rgb");
     if (i==0) {
       schlange1->setHeadColor(Color(1.0,0,0));
     } else {
       schlange1->setHeadColor(Color(0,1.0,0));
     }
     //   Primitive* trunk = schlange1->getMainPrimitive();

     //fixator = new FixedJoint(trunk, global.environment);
     //  fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) ,                    Axis(0,0,1), Axis(0,1,0));
     //       fixator->init(odeHandle, osgHandle);


     //      AbstractController *controller = new InvertMotorNStep();
     // DerBigControllerConf cconf = DerBigController::getDefaultConf();
     // DerControllerConf cconf = DerController::getDefaultConf();
     DerLinInvertConf cconf = DerLinInvert::getDefaultConf();
     // cconf.useS=true;
     cconf.cInit=1.0;

     vector<Layer> layers;
     //    layers.push_back(Layer(10,0.5,FeedForwardNN::tanh)); // hidden layer
     //  size of output layer is automatically set
     layers.push_back(Layer(1,1,FeedForwardNN::linear));
     MultiLayerFFNN* net = new MultiLayerFFNN(0.0, layers, false);// false means no bypass.
     cconf.model=net;


     //   layers.clear();
     //                     layers.push_back(Layer(8,0.5,FeedForwardNN::tanh)); // hidden layer
     //           // size of output layer is automatically set
     //           layers.push_back(Layer(1,0.5,FeedForwardNN::tanh));
     //           MultiLayerFFNN* sat = new MultiLayerFFNN(1.0, layers, false);
     //           cconf.sat   = sat;
     cconf.useS=false;

     // AbstractController *controller = new DerBigController(cconf);
     //AbstractController *controller = new DerController(cconf);
     //  controller->setParam("fantcontrol",200);
     // controller->setParam("fantcontrollen",50);
     //AbstractController *controller = new SineController();
     AbstractController* controller = new DerLinInvert(cconf);

     AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); //Only this line for one2Onewiring
     //  DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
     //       c.useId = true;
     //       c.useFirstD = false;
     //       DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );

     OdeAgent* agent = new OdeAgent(global);
     agent->init(controller, schlange1, wiring);
     global.agents.push_back(agent);
     global.configs.push_back(agent);

     controller->setParam("steps",1);
     controller->setParam("noise",0.03);
     controller->setParam("epsC",0.05);
     controller->setParam("epsA",0.03);
     controller->setParam("adaptrate",0.0);//0.005);
     controller->setParam("rootE",3);
     controller->setParam("logaE",0);
     controller->setParam("epsSat",0.02);
     controller->setParam("weighting",1);
     controller->setParam("zetaupdate",1.0);

     // controller->setParam("desens",0.0);
     controller->setParam("s4delay",2.0);
     controller->setParam("s4avg",3.0);

     controller->setParam("factorB",.03);
     controller->setParam("noiseB",0.0);

     controller->setParam("frictionjoint",0.001);
     controller->setParam("frictionground",0.08);
     controller->setParam("teacher", 0.0);
     controller->setParam("dampA",0.001);
     controller->setParam("dampC",0.00001);

   }//creation of snakes End





   //****** FLAT SNAKES **********/
   //creation of flatsnakes
   double height =.1;
   for(int i=0; i<flatsnakes; i++){

     //****************/
     SchlangeConf conf = Schlange::getDefaultConf();
     conf.segmMass   = .2;
     conf.segmLength=.8;
     conf.segmDia=.2;
     conf.motorPower=.5;
     conf.segmNumber = 5+12*i;//-i/2;
     // conf.jointLimit=conf.jointLimit*3;
     conf.jointLimit=conf.jointLimit*2.0;
     conf.frictionJoint=0.01;
     PlattfussSchlange* schlange1;
     //  SchlangeServo* schlange1;
     if (i==0) {
       schlange1 =
         //  new SchlangeServo ( odeHandle, osgHandle.changeColor(Color(0.0, 0.8, 0.6)),
         new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(1.0, 1.0, 1.0)),
                                 conf, "S1");
     } else {
       schlange1 =
         // new SchlangeServo ( odeHandle, osgHandle.changeColor(Color(0.1, 0.8, 0.2)),
         new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(0.8, 0.4, .3)),
                                 conf, "S2");
     }
     //Positionieren und rotieren
     schlange1->place(// osg::Matrix::rotate(M_PI/2, 0, 1, 0)*
                      // osg::Matrix::translate(-.7+0.7*i,0,(i+1)*(.2+conf.segmNumber)/2.0/*+2*/));
                      osg::Matrix::translate(-i, 5 + i*2,height+2));
     schlange1->setTexture("Images/whitemetal_farbig_small.rgb");
     if (i==0) {
       schlange1->setHeadColor(Color(1.0,0,0));
     } else {
       schlange1->setHeadColor(Color(0,1.0,0));
     }


     DerLinInvertConf cc = DerLinInvert::getDefaultConf();
     // AbstractController* controller = new DerLinInvert(cc);

     //     //    InvertMotorBigModelConf cc = InvertMotorBigModel::getDefaultConf();
     vector<Layer> layers;
     // layers.push_back(Layer(20,0.5,FeedForwardNN::tanh)); // hidden layer
     // size of output layer is automatically set
     layers.push_back(Layer(1,1,FeedForwardNN::linear));
     MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass.
     // cc.model=net;
     cc.model=net;
     // cc.useS=true;
     cc.useS=false;
     AbstractController* controller = new DerLinInvert(cc);

     //   //      AbstractController *controller = new InvertMotorNStep();
     //       DerBigControllerConf cconf = DerBigController::getDefaultConf();
     //       cconf.cInit=.7;
     //       AbstractController *controller = new DerBigController(cconf);
     //AbstractController *controller = new SineController();

     AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); //Only this line for one2Onewiring
     //  DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
     //       c.useId = true;
     //       c.useFirstD = false;
     //       DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );

     OdeAgent* agent = new OdeAgent(global);
     agent->init(controller, schlange1, wiring);
     global.agents.push_back(agent);
     global.configs.push_back(agent);

     controller->setParam("steps",1);
     controller->setParam("epsC",0.001);
     controller->setParam("epsA",0.0);
     controller->setParam("adaptrate",0.0);//0.005);
     controller->setParam("rootE",3);
     controller->setParam("logaE",0);

     // controller->setParam("desens",0.0);
     controller->setParam("s4delay",1.0);
     controller->setParam("s4avg",1.0);

     controller->setParam("factorB",0.03);
     controller->setParam("noiseB",0.0);

     controller->setParam("frictionjoint",0.01);
     controller->setParam("teacher", 0.0);

   }//creation of flatsnakes End


   /******* S L I D E R - W H E E L I E *********/
   for(int i=0; i < wheelies; i++) {
     SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
     mySliderWheelieConf.segmNumber=12;
     mySliderWheelieConf.motorPower=0.4;
     mySliderWheelieConf.frictionGround=0.8;
     mySliderWheelieConf.sliderLength=.8;
     mySliderWheelieConf.segmLength=0.6;
     OdeRobot* robot = new SliderWheelie(odeHandle, osgHandle,
                                         mySliderWheelieConf, "sliderWheelie1");
     double height = 2;
     robot->place(Pos(3,-3, height));

     //   DerBigControllerConf cconf = DerBigController::getDefaultConf();
     //       cconf.cInit=1.0;


     //           vector<Layer> layers;
     //            layers.push_back(Layer(10,0.5,FeedForwardNN::tanh)); // hidden layer
     //            //  size of output layer is automatically set
     //           layers.push_back(Layer(1,1,FeedForwardNN::linear));
     //           MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass.
     //           cconf.model=net;
     //AbstractController *controller = new DerController(cconf);
     AbstractController *controller = new DerController();


     // InvertMotorNStepConf sliderinvertnconf = InvertMotorNStep::getDefaultConf();
     // sliderinvertnconf.cInit=1;
     // AbstractController* controller = new InvertMotorNStep(sliderinvertnconf);
     //slidercontroller = new SineController();
     controller->setParam("steps",1);
     controller->setParam("factorB",0.03);
     controller->setParam("epsC",.1);
     controller->setParam("epsA",.0);
     controller->setParam("teacher",.0);
     controller->setParam("rootE",3);

     DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
     c.useId = true;
     c.useFirstD = false;
     AbstractWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise(0.1) );
     //     sliderwiring = new One2OneWiring(new ColorUniformNoise(0.1));
     OdeAgent* agent = new OdeAgent(global);
     agent->init(controller, robot, wiring);
     global.agents.push_back(agent);
     global.configs.push_back(agent);
   }//Creation of wheelies end

  }
  //*************HAND*************


  //HAND ENDE ********************+

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    //     if(draw){
    //       if(reck) reck->update();
    //       if(reckLeft) reckLeft->update();
    //       if(reckRight) reckRight->update();
    //     }

    if(globalData.time>1 && reckturner==true && reck==0){
      double reckLength=20;
      reck = new Capsule(0.01,reckLength);
      reck->init(odeHandle, 0,osgHandle);
      reck->setPose(osg::Matrix::rotate(M_PI/2.0,0,1,0)
                    * osg::Matrix::translate((reckLeft->getAnchor()+reckRight->getAnchor())*0.5)
                    );
      // disable collisions between hands and pole
      globalData.odeConfig.odeHandle.addIgnoredPair(reck, reckLeft->getPart1());
      globalData.odeConfig.odeHandle.addIgnoredPair(reck, reckRight->getPart1());

      fixator = new FixedJoint(reck, globalData.environment);
      fixator->init(odeHandle, osgHandle);
      Primitive* reckEnds = new Sphere(0.1);
      reckEnds->init(odeHandle,0,osgHandle);
      reckEnds->setPose(reck->getPose()*osg::Matrix::translate(reckLength/2,0,0));
      fixator = new FixedJoint(reckEnds, globalData.environment);
      fixator->init(odeHandle, osgHandle);
      reckEnds = new Sphere(0.1);
      reckEnds->init(odeHandle,0,osgHandle);
      reckEnds->setPose(reck->getPose()*osg::Matrix::translate(-reckLength/2,0,0));
      fixator = new FixedJoint(reckEnds, globalData.environment);
      fixator->init(odeHandle, osgHandle);
    }
    if(control &&!pause){
      if(globalData.sim_step % int(10.0/globalData.odeConfig.simStepSize)==0){
        if(robot) robot->storeToFile("robot_10.rob");
      }
      if(globalData.sim_step % int(100.0/globalData.odeConfig.simStepSize)==0){
        if(robot) robot->storeToFile("robot_100.rob");
      }

      if(centerforce!=0){
        // force to center
        FOREACH(vector<OdeAgent*> , globalData.agents, a){
          Primitive* body = (*a)->getRobot()->getMainPrimitive();
          osg::Vec3 pos = body->getPosition();
          osg::Vec3 d = (center - pos);
          dBodyAddForce(body->getBody(), d.x()*centerforce, d.y()*centerforce,  d.z()*centerforce*50);//1.8
        }
      }
      if(forwardforce!=0){
        // force forwards
        FOREACH(vector<OdeAgent*> , globalData.agents, a){
          Primitive* body = (*a)->getRobot()->getMainPrimitive();
          osg::Matrix pose = body->getPose();
          // transform a local point ahead of robot into global coords
          // note that the internal corrd of the main primitive has z towards the front
          Pos point = (Pos(0,0,1)*pose );
          point.z()=pose.getTrans().z();  // only use x,y component (this can be commented out)
          Pos d = (point - pose.getTrans());
          d.normalize();

          dBodyAddForce(body->getBody(),
                        d.x()*forwardforce, d.y()*forwardforce, d.z()*forwardforce);
          if(!forcepoint){
            forcepoint = new Sphere(0.1);
            forcepoint->init(odeHandle, 0, osgHandle /*osgHandle.changeAlpha(0.4)*/,
                             Primitive::Geom | Primitive::Draw);
          }
          forcepoint->setPosition(point);
          forcepoint->update();
        }
      }
    }
   //  if(control &&!pause && centerforce!=0){
//       // force to center
//       FOREACH(vector<OdeAgent*> , globalData.agents, a){
//         Primitive* body = (*a)->getRobot()->getMainPrimitive();
//         osg::Vec3 pos = body->getPosition();
//         osg::Vec3 d = (center - pos);
//         dBodyAddForce(body->getBody(), d.x()*centerforce, d.y()*centerforce,  d.z()*centerforce*.8);
//       }
//     }
//       if(forwardforce!=0){
//         // force forwards
//         FOREACH(vector<OdeAgent*> , globalData.agents, a){
//           Primitive* body = (*a)->getRobot()->getMainPrimitive();
//           osg::Matrix pose = body->getPose();
//           // transform a local point ahead of robot into global coords
//           // note that the internal corrd of the main primitive has z towards the front
//           Pos point = (Pos(0,0,1)*pose );
//           point.z()=pose.getTrans().z();  // only use x,y component (this can be commented out)
//           Pos d = (point - pose.getTrans());
//           d.normalize();

//           dBodyAddForce(body->getBody(),
//                         d.x()*forwardforce, d.y()*forwardforce, d.z()*forwardforce);
//           if(!forcepoint){
//             forcepoint = new Sphere(0.1);
//             forcepoint->init(odeHandle, 0, osgHandle /*osgHandle.changeAlpha(0.4)*/,
//                              Primitive::Geom | Primitive::Draw);
//           }
//           forcepoint->setPosition(point);
//           forcepoint->update();
//         }
//       }
  };

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    Substance s;
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'x':
          if(fixator) delete fixator;
          fixator=0;
          return true;
          break;
        case 'i':
          if(playground) {
            s = playground->getSubstance();
            s.hardness*=1.5;
            cout << "hardness " << s.hardness << endl;
            playground->setSubstance(s);
          }
          return true;
          break;
        case 'j':
          if(playground) {
            s = playground->getSubstance();
            s.hardness/=1.5;
            cout << "hardness " << s.hardness << endl;
            playground->setSubstance(s);
          }
          return true;
          break;
        case 's':
          if(robot) {
            robot->storeToFile("myrobot.rob");
          }
          return true;
          break;
        case 'r':
          if(robot) {
            robot->restoreFromFile("myrobot.rob");
          }
          return true;
          break;
        case '0':
          if(robot) {
            robot->restoreFromFile("robot_10.rob");
          }
          return true;
          break;
        case '9':
          if(robot) {
            robot->restoreFromFile("robot_100.rob");
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

  void setupPlaygrounds(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, Grounds ground){
    switch (ground){
    case Normal:
      {
        playground = new Playground(odeHandle, osgHandle,osg::Vec3(widthground, 0.208, heightground));
        playground->setColor(Color(1.,1.,1.,.99));
        //     playground->setTexture("Images/really_white.rgb");
        //     playground->setGroundTexture("Images/dusty.rgb");
        //playground->setGroundColor(Color(1.,1.,1.,1.));
        playground->setPosition(osg::Vec3(0,0,.1));
        //      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(1.0875, 8.8, 1.3975));
        //       playground->setColor(Color(0.88f,0.4f,0.26f,1));
        // playground->setPosition(osg::Vec3(20,20,.5));
        Substance substance;
        substance.toRubber(5);
        //   substance.toMetal(1);
        playground->setGroundSubstance(substance);
        global.obstacles.push_back(playground);
        /*    double xboxes=0.0;
              double yboxes=0.0;*/
        double xboxes=0;//15;//19.0;
        double yboxes=0;//15;
        double boxdis=.9;//.45;//1.6;
        for (double j=0.0;j<xboxes;j++)
          for(double i=0.0; i<yboxes; i++) {
            double xsize= .6;//1.0;
            double ysize= .5;//.25;
            double zsize=.4;
            PassiveBox* b =
              new PassiveBox(odeHandle,
                             osgHandle, osg::Vec3(xsize,ysize,zsize),0.0);
            b->setPosition(Pos(20+boxdis*(i-(xboxes-1)/2.0),20+boxdis*(j-(yboxes-1)/2.0), 0.01));
            //         b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
            //         b->setTexture("Images/light_chess.rgb");
            global.obstacles.push_back(b);
          }
        break;
      }
    case Octa:
      {
        playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(diamOcta, 0.2,/*Height*/ 10), 12,false);
        playground->setTexture("Images/really_white.rgb");
        playground->setColor(Color(0.4,0.8,0.4,0.2));
        playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
        global.obstacles.push_back(playground);
        break;
      }
    case Pit:
      {
        // we stack two playgrounds in each other.
        // The outer one is hard and the inner one is softer
        int anzgrounds=2;
        Substance soft = Substance::getRubber(5);
        double thicknessSoft = 0.1;
        for (int i=0; i< anzgrounds; i++){
          OdeHandle myHandle = odeHandle;
          if(i==0){
            myHandle.substance = soft;
          }else{
            myHandle.substance.toMetal(1);
          }
          Playground* playground = new Playground(myHandle, osgHandle,
                                                  osg::Vec3(pitsize+2*thicknessSoft*i, thicknessSoft + 12*i, pitheight),
                                                  1, i==(anzgrounds-1));
          if(i==(anzgrounds-1)){ // set ground also to the soft substance
            playground->setGroundSubstance(soft);
          }
          if(i==0) this->playground=playground;
          playground->setColor(Color(0.5,0.1,0.1,i==0 ? 0 : .99)); // inner wall invisible
          playground->setPosition(osg::Vec3(0,0,thicknessSoft)); // playground positionieren und generieren
          global.obstacles.push_back(playground);
        }

        break;
      }
    case Uterus:
      {
        // we stack two playgrounds in each other.
        // The outer one is hard (and invisible) and the inner one is soft
        int anzgrounds=2;
        // this is the utterus imitation: high slip, medium roughness, high elasticity, soft
        Substance uterus(0.2/*roughness*/, 0.1 /*slip*/,
                         .5 /*hardness*/, 0.95 /*elasticity*/);
        double thickness = 0.4;
        for (int i=0; i< anzgrounds; i++){
          OdeHandle myHandle = odeHandle;
          if(i==0){
            myHandle.substance = uterus;
          }else{
            myHandle.substance.toMetal(.2);
          }
          Playground* playground = new Playground(myHandle, osgHandle,
                                                  osg::Vec3(uterussize+2*thickness*i,
                                                            i==0 ? thickness : .5, pitheight),
                                                  1, i==0);
          playground->setTexture("Images/dusty.rgb");
          if(i==0){ // set ground also to the soft substance
            playground->setGroundSubstance(uterus);
          }
          if(i==0) this->playground=playground;
          playground->setColor(Color(0.5,0.1,0.1,i==0? .2 : 0)); // outer ground is not visible (alpha=0)
          playground->setPosition(osg::Vec3(0,0,i==0? thickness : 0 )); // playground positionieren und generieren
          global.obstacles.push_back(playground);
        }

        break;
      }
    case Stacked:
      {
        int anzgrounds=2;
        for (int i=0; i< anzgrounds; i++){
          playground = new Playground(odeHandle, osgHandle, osg::Vec3(10+4*i, .2, .95+0.15*i), 1, i==(anzgrounds-1));
          OdeHandle myhandle = odeHandle;
          //      myhandle.substance.toFoam(10);
          // playground = new Playground(myhandle, osgHandle, osg::Vec3(/*base length=*/50.5,/*wall = */.1, /*height=*/1));
          playground->setPosition(osg::Vec3(0,0,0.2)); // playground positionieren und generieren

          global.obstacles.push_back(playground);
        }
        break;
      }
    }

  }

  Joint* fixator;
  Joint* reckLeft;
  Joint* reckRight;
  Primitive* reck;
  //  Playground* playground;
  AbstractGround* playground;
  double hardness;
  bool reckturner;
  osg::Vec3 center;
  double centerforce;
  double forwardforce;

  Primitive* forcepoint;


};


int main (int argc, char **argv)
{
  ThisSim sim;
  sim.setCaption("lpzrobots Simulator             playfulmachines.com");
  return sim.run(argc, argv) ? 0 : 1;

}
