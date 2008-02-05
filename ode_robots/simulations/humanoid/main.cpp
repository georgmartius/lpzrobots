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
 *   $Log$
 *   Revision 1.2  2008-02-05 07:58:09  der
 *   neue Konfiguration playground
 *
 *   Revision 1.1  2008/01/29 09:52:16  der
 *   first version
 *
 *   Revision 1.3  2007/11/21 13:18:10  der
 *   ralfs aenderungen
 *
 *   Revision 1.2  2007/07/31 08:35:52  martius
 *   tried wrl mesh
 *
 *   Revision 1.1  2007/07/17 07:25:26  martius
 *   first attempt to build a two legged robot (humanoid)
 *
 *   Revision 1.7  2007/07/03 13:06:10  martius
 *   *** empty log message ***
 *
 *   Revision 1.6  2007/04/03 16:35:43  der
 *   *** empty log message ***
 *
 *   Revision 1.5  2007/04/03 11:27:07  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2007/03/16 11:36:10  martius
 *   soft playground
 *
 *   Revision 1.3  2007/02/23 09:30:41  der
 *   *** empty log message ***
 *
 *   Revision 1.2  2007/02/12 13:30:40  martius
 *   dog looks allready nicer
 *
 *   Revision 1.1  2007/01/26 12:07:09  martius
 *   orientationsensor added
 *
 *   Revision 1.2  2006/07/14 12:23:55  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.2  2006/06/25 17:01:57  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.1.2.1  2006/06/10 20:12:45  martius
 *   simulation for uwo (unknown walking object)
 *
 *
 ***************************************************************************/
#include <stdio.h>

// include ode library
#include <ode/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include "simulation.h"

// include agent (class for holding a robot, a controller and a wiring)
#include "odeagent.h"

// used wiring
#include <selforg/one2onewiring.h>

// used robot
#include "skeleton.h"
//#include "derskeleton.h"

// used arena
#include "playground.h"
// used passive spheres
#include "passivesphere.h"
#include "passivemesh.h"
#include "joint.h"

// used controller
//#include <selforg/invertnchannelcontroller.h>
#include <selforg/derbigcontroller.h>
#include <selforg/dercontroller.h>
#include <selforg/invertmotorbigmodel.h>
#include <selforg/multilayerffnn.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/derpseudosensor.h>
/************/

#include "playground.h"
#include "terrainground.h"
#include "octaplayground.h"
#include "sliderwheelie.h"
#include "nimm2.h"
#include "plattfussschlange.h"

#include "sliderwheelie.h"
#include "schlangeservo.h"
#include "schlangeservo2.h"
#include <selforg/derivativewiring.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:


  Joint* fixator;
  AbstractObstacle* playground; 
  double hardness;
  Substance s;
  

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(-1.66705, 4.47234, 3.86184),  Pos(-158.908, -10.5863, 0));

  // Set number of robots:


    // int plattfuesse = 1; 
     int flatsnakes  = 0;
    int snakes = 0;
    //int sphericalsIR = 0;
    //int sphericalsXYZ = 0;
    //int hurlings = 0;
    //int cigars = 0;
    int wheelies = 0;
    int humanoids=2;
    int barrel=0;


    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("noiseY",0.03); 
    global.odeConfig.setParam("noise",0.05); 
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.007);
        global.odeConfig.setParam("gravity", -4);
    //    global.odeConfig.setParam("cameraspeed", 250);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the 
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground: 
    //   setGeometry(double length, double width, double	height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)
	//    s.toMetal(20);
     // Neuer Playground klein innen 
	//   double diam=.5, internaldiameter=.9*diam, offset=1.0*internaldiameter; 

//       OctaPlayground* playground2 = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter.8*/100.3*diam, 0.2,/*Height*/ 10), 12,false);
//           playground2->setTexture("Images/whitemetal_farbig.rgb");
//         playground2->setColor(Color(0.4,0.8,0.4,0.1));
//          playground2->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
//         global.obstacles.push_back(playground2);
// 	playground->setGroundSubstance(s);

   //  int anzgrounds=2;
//     for (int i=0; i< anzgrounds; i++){
//       //    playground = new Playground(odeHandle, osgHandle, osg::Vec3(30+4*i, .2, .95+0.15*i), 1, i==(anzgrounds-1));
//       playground = new Playground(odeHandle, osgHandle, osg::Vec3(50+4*i, .2, 3.95+0.15*i), 1, i==(anzgrounds-1));
//       OdeHandle myhandle = odeHandle;
//       //      myhandle.substance.toFoam(10);
//       // playground = new Playground(myhandle, osgHandle, osg::Vec3(/*base length=*/50.5,/*wall = */.1, /*height=*/1));
//       playground->setPosition(osg::Vec3(0,0,0.2)); // playground positionieren und generieren
//       playground->setSubstance(s);
//       // playground->setPosition(osg::Vec3(i,-i,0)); // playground positionieren und generieren
//     //global.obstacles.push_back(playground);
//     }
//     global.obstacles.push_back(playground);

    int anzgrounds=1;
    for (int i=0; i< anzgrounds; i++){
      playground = new Playground(odeHandle, osgHandle, osg::Vec3(1.8+4*i, 2, .75+0.15*i), 1, i==(anzgrounds-1));
      OdeHandle myhandle = odeHandle;
      //      myhandle.substance.toFoam(10);
      // playground = new Playground(myhandle, osgHandle, osg::Vec3(/*base length=*/50.5,/*wall = */.1, /*height=*/1));
         playground->setColor(Color(0.9,0.1,0.1,0.99));
      playground->setPosition(osg::Vec3(0,0,0.2)); // playground positionieren und generieren
      s.toRubber(100);
      playground->setSubstance(s);
      // playground->setPosition(osg::Vec3(i,-i,0)); // playground positionieren und generieren
    //global.obstacles.push_back(playground);
    }
    global.obstacles.push_back(playground);

//     // 
//     AbstractObstacle* m = new PassiveMesh(odeHandle, osgHandle, "Meshes/skeleton/Chest_center.wrl",0.4,1);
//     m->setPosition(osg::Vec3(1,1,1)); 
//     global.obstacles.push_back(m);
    
    
    

    for (int i=0; i< humanoids/*2*/; i++){ //Several humans

//       ZweiBeinerConf conf = ZweiBeiner::getDefaultConf();
      
//       ZweiBeiner* human = new ZweiBeiner(odeHandle, osgHandle,conf, "Humanoid");     
//       human->place(osg::Matrix::translate(4*i,0,2));
//       global.configs.push_back(human);
      
//       Primitive* trunk = human->getMainPrimitive();
//       fixator = new FixedJoint(trunk, global.environment);
//       fixator->init(odeHandle, osgHandle);

      SkeletonConf conf = Skeleton::getDefaultConf();
      
      double powerfactor = 1.8;//.3 

       conf.bodyMass   = 0.1;
       conf.relLegmass = 5;
      conf.relFeetmass = 1;
      conf.relArmmass = .1;//1.0;

       conf.hipJointLimit=1.2; //!
       conf.kneeJointLimit=1.911; //!
       conf.hip2JointLimit=.9; //!
       conf.armJointLimit=1.2; //!
//       conf.ankleJointLimit=0.001; //!
       conf.pelvisJointLimit=.5; //!    
            conf.hipPower=20 * powerfactor;   
            conf.hip2Power=20 * powerfactor;      //5
            conf.pelvisPower=20 * powerfactor;
      conf.kneePower= 5 * powerfactor;
      conf.anklePower= 2 * powerfactor;
      conf.armPower = 15 * powerfactor;//5
      OdeHandle skelHandle=odeHandle;
      // skelHandle.substance.toMetal(1);
      skelHandle.substance.toRubber(60);//TEST sonst 40
      Skeleton* human = new Skeleton(skelHandle, osgHandle,conf, "Humanoid");           
      human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)
		   *osg::Matrix::translate(-.2 +0.8*i,0,.1+1*i));
      global.configs.push_back(human);
      
      Primitive* trunk = human->getMainPrimitive();
      
      // fixator = new FixedJoint(trunk, global.environment);
       //    fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) , 		   Axis(0,0,1), Axis(0,1,0));
      // fixator->init(odeHandle, osgHandle);

      // create pointer to controller
      // push controller in global list of configurables
      //AbstractController *controller = new SineController();
      //controller->setParam("sinerate",50);
      //controller->setParam("phaseshift",0);
      //    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
      //  cc.useS=false;
      //   AbstractController *controller = new InvertMotorNStep(cc);
      //     DerBigControllerConf cc = DerBigController::getDefaultConf();

           DerPseudoSensorConf cc = DerPseudoSensor::getDefaultConf();    
	   // AbstractController* controller = new DerPseudoSensor(cc);

  //     //    InvertMotorBigModelConf cc = InvertMotorBigModel::getDefaultConf();
          vector<Layer> layers;
          // layers.push_back(Layer(20,0.5,FeedForwardNN::tanh)); // hidden layer
          // size of output layer is automatically set
          layers.push_back(Layer(1,1,FeedForwardNN::linear)); 
          MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass. 
	  // cc.model=net;
          cc.model=net;
          cc.useS=false;
          //cc.useS=false;   
	   AbstractController* controller = new DerPseudoSensor(cc);

      //     // AbstractController* controller = new DerBigController(cc);
      // 	// AbstractController* controller = new InvertMotorBigModel(cc);
      //     controller->setParam("adaptrate",0);
      //     controller->setParam("rootE",3);
      //     controller->setParam("epsC",0.1);
      //     controller->setParam("epsA",0.0);
      //     controller->setParam("steps",1);
      //     controller->setParam("s4avg",2);
      //     controller->setParam("teacher",0);
      //     controller->setParam("dampS",0.0001);
      //     controller->setParam("dampA",0.00003);
      //     //    controller->setParam("kwta",4);
      //     //    controller->setParam("inhibition",0.01);
      
      global.configs.push_back(controller);
      
      // create pointer to one2onewiring
        One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      // DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
//       c.useId = true;
//       c.useFirstD = falfrictiongroundse;
//       DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );
      
      

      // create pointer to agent
      // initialize pointer with controller, robot and wiring
      // push agent in globel list of agents
      OdeAgent* agent = new OdeAgent(plotoptions);
      agent->init(controller, human, wiring);
      //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
      global.agents.push_back(agent);
      
      //  showParams(global.configs);
    }// Several humans end
    


    //****** SNAKES **********/
    //creation of normal   snakes 
    for(int i=0; i<snakes; i++){

      //****************/
      SchlangeConf conf = Schlange::getDefaultConf();
      double snakesize = .6;
      conf.segmMass   = .8;
      conf.segmLength= .8 * snakesize;// 0.8;
        conf.segmDia=.15 *snakesize;
      conf.motorPower= 2 * snakesize;
      conf.segmNumber = 12+2*i;//-i/2; 
      // conf.jointLimit=conf.jointLimit*3;
      conf.jointLimit=conf.jointLimit* 1.6;
      conf.frictionGround=0.3;// +((double)i)/100;
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
		        osg::Matrix::translate(-.7+0.7*i,0,1+(i+1)*(.2+conf.segmNumber)/2.0/*+2*/));
		       // osg::Matrix::translate(5-i,2 + i*2,height+2));
      schlange1->setTexture("Images/whitemetal_farbig_small.rgb");
      if (i==0) {
	schlange1->setHeadColor(Color(1.0,0,0));
      } else {
	schlange1->setHeadColor(Color(0,1.0,0));
      }
   Primitive* trunk = schlange1->getMainPrimitive();
      
   //fixator = new FixedJoint(trunk, global.environment);
       //  fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) , 		   Axis(0,0,1), Axis(0,1,0));
//       fixator->init(odeHandle, osgHandle);


      //      AbstractController *controller = new InvertMotorNStep(); 
      // DerBigControllerConf cconf = DerBigController::getDefaultConf();
      // DerControllerConf cconf = DerController::getDefaultConf();
	    DerPseudoSensorConf cconf = DerPseudoSensor::getDefaultConf();
      // cconf.useS=true;
      cconf.cInit=1.0;

          vector<Layer> layers;
	  //    layers.push_back(Layer(10,0.5,FeedForwardNN::tanh)); // hidden layer
	   //  size of output layer is automatically set
          layers.push_back(Layer(1,1,FeedForwardNN::linear)); 
          MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass. 
          cconf.model=net;
          cconf.useS=false;

      // AbstractController *controller = new DerBigController(cconf); 
      //AbstractController *controller = new DerController(cconf); 
      //  controller->setParam("fantcontrol",200);
      // controller->setParam("fantcontrollen",50);
      //AbstractController *controller = new SineController();  
        AbstractController* controller = new DerPseudoSensor(cconf);
  
        AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); //Only this line for one2Onewiring
     //  DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
//       c.useId = true;
//       c.useFirstD = false;
//       DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );
      
      OdeAgent* agent = new OdeAgent(plotoptions);
      agent->init(controller, schlange1, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(schlange1);
  
 
      controller->setParam("steps",1);
      controller->setParam("epsC",0.001);
      controller->setParam("epsA",0.0);
      controller->setParam("adaptrate",0.0);//0.005);
      controller->setParam("rootE",3); 
      controller->setParam("logaE",0);

      // controller->setParam("desens",0.0);
      controller->setParam("s4delay",2.0);
      controller->setParam("s4avg",2.0);
    
      controller->setParam("factorB",0.0); 
      controller->setParam("noiseB",0.0);

      controller->setParam("frictionjoint",0.01);
      controller->setParam("frictionground",0.03);
      controller->setParam("teacher", 0.0); 
          controller->setParam("dampA",0.0001);
          controller->setParam("dampC",0.000001);
    
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
      conf.frictionGround=0.04;// +((double)i)/100;
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
 

           DerPseudoSensorConf cc = DerPseudoSensor::getDefaultConf();    
	   // AbstractController* controller = new DerPseudoSensor(cc);

  //     //    InvertMotorBigModelConf cc = InvertMotorBigModel::getDefaultConf();
          vector<Layer> layers;
          // layers.push_back(Layer(20,0.5,FeedForwardNN::tanh)); // hidden layer
          // size of output layer is automatically set
          layers.push_back(Layer(1,1,FeedForwardNN::linear)); 
          MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass. 
	  // cc.model=net;
          cc.model=net;
          cc.useS=false;
          //cc.useS=false;   
	   AbstractController* controller = new DerPseudoSensor(cc);

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
      
      OdeAgent* agent = new OdeAgent(plotoptions);
      agent->init(controller, schlange1, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(schlange1);
  
 
      controller->setParam("steps",1);
      controller->setParam("epsC",0.001);
      controller->setParam("epsA",0.0);
      controller->setParam("adaptrate",0.0);//0.005);
      controller->setParam("rootE",3);
      controller->setParam("logaE",0);

      // controller->setParam("desens",0.0);
      controller->setParam("s4delay",1.0);
      controller->setParam("s4avg",1.0);
    
      controller->setParam("factorB",0.0); 
      controller->setParam("noiseB",0.0);

      controller->setParam("frictionjoint",0.01);
      controller->setParam("teacher", 0.0); 
    
    }//creation of flatsnakes End

    
      /******* S L I D E R - W H E E L I E *********/
    for(int i=0; i < wheelies; i++) {      
      SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
      mySliderWheelieConf.segmNumber=12;
      mySliderWheelieConf.jointLimit=M_PI/2;
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
// 	   //  size of output layer is automatically set
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
      controller->setParam("factorB",0);
      controller->setParam("epsC",.1);
      controller->setParam("epsA",.0);
      controller->setParam("teacher",.0);
      controller->setParam("rootE",3);
      
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = true;
      c.useFirstD = false;
      AbstractWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise(0.1) );
      //     sliderwiring = new One2OneWiring(new ColorUniformNoise(0.1));
      OdeAgent* agent = new OdeAgent(plotoptions);
      agent->init(controller, robot, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(robot);        
    }//Creationn of wheelies end




    showParams(global.configs);
  }


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
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
};


int main (int argc, char **argv)
{ 
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}
 
