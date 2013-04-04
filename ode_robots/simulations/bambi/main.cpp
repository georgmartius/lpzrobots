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
 *   Revision 1.10  2011-11-10 16:28:48  der
 *   liftup operator
 *
 *   Revision 1.9  2011/10/27 15:54:36  martius
 *   new build system with -config shell script and configurator intragration
 *
 *   Revision 1.8  2011/10/14 09:36:18  martius
 *   snakes have no frictionGround parameter anymore, since it was not used,
 *    use the substances now
 *
 *   Revision 1.7  2011/06/03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.6  2010/11/05 13:54:05  martius
 *   store and restore for robots implemented
 *
 *   Revision 1.5  2010/10/20 15:23:54  der
 *   minor changes
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
 *   Revision 1.17  2009/08/09 20:19:57  der
 *   From PC home
 *
 *   Revision 1.16  2009/05/11 17:01:20  martius
 *   new velocity servos implemented
 *   reorganized parameters, now also neck and elbows are configurable
 *
 *   Revision 1.15  2009/04/23 14:33:20  der
 *   Small changes
 *
 *   Revision 1.14  2009/03/30 13:57:40  martius
 *   ground texture removed
 *
 *   Revision 1.13  2009/01/20 17:29:52  martius
 *   cvs commit
 *
 *   Revision 1.12  2008/11/14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.11  2008/11/11 19:40:58  der
 *   small changes
 *
 *   Revision 1.10  2008/06/20 14:03:00  guettler
 *   reckturner
 *
 *   Revision 1.9  2008/05/27 13:25:12  guettler
 *   powerfactor moved to skeleton
 *
 *   Revision 1.8  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.7  2008/03/26 09:25:44  der
 *   small changes
 *
 *   Revision 1.6  2008/03/14 08:04:23  der
 *   Some changes in main and skeleton (with new outfit)
 *
 *   Revision 1.5  2008/02/28 07:42:31  der
 *   some small changes
 *
 *   Revision 1.4  2008/02/08 13:35:09  der
 *   satelite teaching
 *
 *   Revision 1.3  2008/02/07 14:25:02  der
 *   added setTexture and setColor for skeleton
 *
 *   Revision 1.2  2008/02/05 07:58:09  der
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
#include <ode-dbl/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
//#include <simulation.h>
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

// used robot
#include "skeleton.h"
//#include "derskeleton.h"

// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivemesh.h>
#include <ode_robots/joint.h>
#include <ode_robots/axisorientationsensor.h>

// used controller
//#include <selforg/invertnchannelcontroller.h>
#include <selforg/derbigcontroller.h>
#include <selforg/dercontroller.h>
#include <selforg/invertmotorbigmodel.h>
#include <selforg/multilayerffnn.h>
#include <selforg/elman.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/derpseudosensor.h>
#include <selforg/derlininvert.h>
/************/

#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/plattfussschlange.h>
#include <ode_robots/addsensors2robotadapter.h>

#include <ode_robots/passivebox.h>

#include <ode_robots/sliderwheelie.h>
#include <ode_robots/schlangeservo.h>
#include <ode_robots/schlangeservo2.h>
#include <selforg/derivativewiring.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:


  Joint* fixator;
  Joint* reckLeft;
  Joint* reckRight;
  Primitive* reck;
  Playground* playground;
  //  AbstractObstacle* playground;
  double hardness;
  Substance s;
  bool reckturner;
  osg::Vec3 center;
  double centerforce;
  double forwardforce;

  Primitive* forcepoint;


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos (Pos(22.4468, 25.8365, 2.74255),  Pos(157.862, -12.7123, 0));

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


    bool fixedInAir = true;
    reckturner = false;
    // Playground types
    bool narrow = true;
    double widthground = 10.85;// 100; //1.3;
    double heightground = .8;// 1.2;
    addParameterDef("centerforce", &centerforce, .0);//2.0
    addParameterDef("forwardforce", &forwardforce, 0.0);
    center = osg::Vec3(20,20,3);
    forcepoint = 0;
    global.configs.push_back(this);

    fixator=0;
    reckLeft = reckRight = 0;
    reck = 0;

    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",2);//4);
    global.odeConfig.setParam("noise",0.0);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.004);//0.004);
        global.odeConfig.setParam("gravity", -6);
    //    global.odeConfig.setParam("cameraspeed", 250);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double        height)
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
//         playground->setGroundSubstance(s);

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

//     int anzgrounds=1;
//     for (int armPower = 15;//5i=0; i< anzgrounds; i++){
//       playground = new Playground(odeHandle, osgHandle, osg::Vec3(10.05+4*i, 10, .75+0.15*i), 1, i==(anzgrounds-1));
//       OdeHandle myhandle = odeHandle;
//       //      myhandle.substance.toFoam(10);
//       // playground = new Playground(myhandle, osgHandle, osg::Vec3(/*base length=*/50.5,/*wall = */.1, /*height=*/1));
//          playground->setColor(Color(0.9,0.1,0.1,0.99));
//       playground->setPosition(osg::Vec3(0,0,0.2)); // playground positionieren und generieren
//       //s.toRubber(100);
//            s.toPlastic(90);
//       playground->setSubstance(s);
//       // playground->setPosition(osg::Vec3(i,-i,0)); // playground positionieren und generieren
//     //global.obstacles.push_back(playground);
//     }
//     global.obstacles.push_back(playground);

// //     //
// //     AbstractObstacle* m = new PassiveMesh(odeHandle, osgHandle, "Meshes/skeleton/Chest_center.wrl",0.4,1);
// //     m->setPosition(osg::Vec3(1,1,1));
// //     global.obstacles.push_back(m);

   if(narrow){
     Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(widthground, 0.208, heightground));
     playground->setColor(Color(1.,1.,1.,.99));
     //     playground->setTexture("Images/really_white.rgb");
     //     playground->setGroundTexture("Images/dusty.rgb");
     //playground->setGroundColor(Color(1.,1.,1.,1.));
     playground->setPosition(osg::Vec3(20,20,.1));
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
     double xboxes=15;//19.0;
     double yboxes=15;
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
   }


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
     SkeletonConf conf = Skeleton::getDefaultConf();
     // velocity servos
     //SkeletonConf conf = Skeleton::getDefaultConfVelServos();

     conf.massfactor   = 1;
     conf.relLegmass = 1;
     conf.relFeetmass = 1;
     conf.relArmmass = 1;//1.0;

     //       conf.ankleJointLimit=0.001; //!
     //     conf.pelvisPower=20;
     // if(reckturner)      conf.armPower = 30;

     conf.powerfactor = .15;// .95;//.65;//5;
     if (reckturner) conf.powerfactor *=.2;
     if (i==0)
       conf.trunkColor=Color(0.1, 0.3, 0.8);
     else
       conf.trunkColor=Color(0.75, 0.1, 0.1);
     if(reckturner)
       conf.handsRotating = true;

     conf.useBackJoint = true;
     conf.jointLimitFactor = 1.4;

     //     conf.irSensors = true;

     //    conf.bodyTexture="Images/whitemetal_farbig_small.rgb";
     //     conf.bodyColor=Color(1.0,1.0,0.0);
     std::list<Sensor*> sensors;
     // additional sensor
     //     sensors.push_back(new AxisOrientationSensor(AxisOrientationSensor::OnlyZAxis));
     OdeHandle skelHandle=odeHandle;
     // skelHandle.substance.toMetal(1);
    //     skelHandle.substance.toPlastic(.5);//TEST sonst 40
     // skelHandle.substance.toRubber(5.00);//TEST sonst 40
     Skeleton* human0 = new Skeleton(skelHandle, osgHandle,conf, "Humanoid");
     AddSensors2RobotAdapter* human =
       new AddSensors2RobotAdapter(skelHandle, osgHandle, human0, sensors);
     human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)
                  //   *osg::Matrix::translate(-.2 +2.9*i,0,1));
                  *osg::Matrix::translate(.2*i+20,2*i+20,.841/*7*/ +2*i));
     global.configs.push_back(human0);


     if( fixedInAir){
       Primitive* trunk = human->getMainPrimitive();

       fixator = new FixedJoint(trunk, global.environment);
       //       // fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) ,                    Axis(0,0,1), Axis(0,1,0));
       fixator->init(odeHandle, osgHandle);
     }else if(reckturner){
       Primitive* leftHand = human0->getAllPrimitives()[Skeleton::Left_Hand];
       Primitive* rightHand = human0->getAllPrimitives()[Skeleton::Right_Hand];

       reckLeft = new SliderJoint(leftHand, global.environment, leftHand->getPosition(), Axis(1,0,0));
       reckLeft->init(odeHandle, osgHandle,false);
       reckRight = new SliderJoint(rightHand, global.environment, rightHand->getPosition(), Axis(1,0,0));
       reckRight->init(odeHandle, osgHandle,false);
     }



     // create pointer to controller
     // push controller in global list of configurables

     DerLinInvertConf cc = DerLinInvert::getDefaultConf();
     //           BasicControllerConf cc = BasicController::getDefaultConf();
     // AbstractController* controller = new DerLinInvert(cc);

     cc.cInit= .001;// 1.05;//1.005;
     cc.someInternalParams = true;

     vector<Layer> layers;
     layers.push_back(Layer(20,0.5,FeedForwardNN::tanh)); // hidden layer
     // size of output layer is automatically set
     layers.push_back(Layer(1,1,FeedForwardNN::linear));
     MultiLayerFFNN* net = new MultiLayerFFNN(0.0, layers, false);// false means no bypass.
     cc.model = net;

     //           layers.clear();
     //           layers.push_back(Layer(3,0.5,FeedForwardNN::tanhr)); // hidden layer
     //           // size of output layer is automatically set
     //           layers.push_back(Layer(1,0.5,FeedForwardNN::tanhr));
     //           MultiLayerFFNN* sat = new MultiLayerFFNN(1.0, layers, false);
     //           cc.sat   = sat;

     //Elman Net
     layers.clear();
     layers.push_back(Layer(40,0.5,Elman::tanhr)); // hidden layer
     // size of output layer is automatically set
     layers.push_back(Layer(1,0.5,Elman::tanh));
     Elman* sat = new Elman(1, layers,true,true, false);
     cc.sat   = sat;

   cc.useS=true;
    //       cc.useS=false;
    AbstractController* controller = new DerLinInvert(cc);
     //     AbstractController* controller = new BasicController(cc);
   //   AbstractController* controller = new SineController(1<<14); // only motor 14
    //   AbstractController* controller = new SineController(); // only motor 14
     //AbstractController* controller = new SineController(0,SineController::Impulse);
   //AbstractController* controller = new SineController((~0),SineController::Impulse);
     //AbstractController* controller = new SineController(3<<17,SineController::Impulse);
     //     controller->setParam("period",1000);
     //     controller->setParam("phaseshift",0);


     //     controller->setParam("adaptrate",0);
     //     controller->setParam("rootE",3);
     controller->setParam("epsC",0.03);
     controller->setParam("epsSat",0.02);
     controller->setParam("epsA",0.02);
     controller->setParam("steps",1);
     controller->setParam("s4avg",2);
     controller->setParam("s4delay",2);
     controller->setParam("teacher",0.0);
     controller->setParam("dampC",0.00001);
     controller->setParam("dampS",0.00001);
     controller->setParam("dampA",0.0001);
     controller->setParam("factorB",.03);
     controller->setParam("gamma",.1);
     // controller->setParam("weighting",1);
     controller->setParam("noiseY",0.03);
     controller->setParam("zetaupdate",0);
     controller->setParam("PIDint",.3);
          controller->setParam("intstate",1);

     global.configs.push_back(controller);

     // create pointer to one2onewiring
     One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
     // DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
//      c.useId = true;
//      c.useFirstD = false;
//      DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise(.2) );

     // create pointer to agent
     // initialize pointer with controller, robot and wiring
     // push agent in globel list of agents
     OdeAgent* agent = new OdeAgent(i==0 ? plotoptions : list<PlotOption>());
     agent->init(controller, human, wiring);
     //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
     global.agents.push_back(agent);

     //
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

     //Elman Net
     layers.clear();
     // layers.push_back(Layer(8,0.5,Elman::tanhr)); // hidden layer
     // size of output layer is automatically set
     layers.push_back(Layer(1,0.5,Elman::tanh));
     Elman* sat = new Elman(1, layers,true,true, false);
     cconf.sat   = sat;

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
     global.configs.push_back(controller);
     global.configs.push_back(schlange1);


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
     global.configs.push_back(controller);
     global.configs.push_back(robot);
   }//Creationn of wheelies end





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
      if(centerforce!=0){
        // force to center
        FOREACH(vector<OdeAgent*> , globalData.agents, a){
          Primitive* body = (*a)->getRobot()->getMainPrimitive();
          osg::Vec3 pos = body->getPosition();
          osg::Vec3 d = (center - pos);
          dBodyAddForce(body->getBody(), d.x()*centerforce, d.y()*centerforce,  d.z()*centerforce*10.8);//1.8
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
  //  sim.setGroundTexture("Images/whiteground.rgb");
  return sim.run(argc, argv) ? 0 : 1;

}

