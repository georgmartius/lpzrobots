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
 *   Revision 1.26  2011-10-14 09:36:18  martius
 *   snakes have no frictionGround parameter anymore, since it was not used,
 *    use the substances now
 *
 *   Revision 1.25  2011/06/03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.24  2011/06/01 22:02:56  martius
 *   getAllPrimitives changed to vector return type
 *   inspectables infolines are printed without name again (for guilogger)
 *
 *   Revision 1.23  2010/11/05 13:54:05  martius
 *   store and restore for robots implemented
 *
 *   Revision 1.22  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.21  2009/10/09 17:18:03  martius
 *   removed orientationsensors for now to enable configurable and inspectable
 *
 *   Revision 1.20  2009/09/18 15:07:18  martius
 *   forward force
 *
 *   Revision 1.19  2009/09/17 09:10:53  martius
 *   force forward implemented
 *
 *   Revision 1.18  2009/08/10 14:59:19  der
 *   copy of local controller version.
 *   new skeleton from bambi
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
#include "derlininvertmpi.h"
#include <selforg/derlinunivers.h>
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

//***********HAND




//*********HAND

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


  //   int plattfuesse = 1;
     int flatsnakes  = 0;
     int snakes = 0;
     //int sphericalsIR = 0;
    //int sphericalsXYZ = 0;
    //int hurlings = 0;
    //int cigars = 0;
    int wheelies = 0;
    int humanoids=1;
    //    int barrel=0;
    // int dogs = 0;

    addParameterDef("centerforce", &centerforce, 0.0);
    addParameterDef("forwardforce", &forwardforce, 0.0);
    global.configs.push_back(this);

    bool fixedInAir = false;
    reckturner = false;
    // Playground types
    bool narrow = false;

    fixator=0;
    reckLeft = reckRight = 0;
    reck = 0;

    center = osg::Vec3(20,20,3);
    forcepoint = 0;

    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",5);//4);
    global.odeConfig.setParam("noiseY",.1);
    global.odeConfig.setParam("noise",0.0);
    //>>>>>>> 1.13
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.01);//0.004);

        global.odeConfig.setParam("gravity", -7);

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
        double diam = 1;
      OctaPlayground* playground =
new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter.8*/.9*diam, .2,/*Height=*/ 1.05), 8,true);
     //      playground->setTexture("Images/whitemetal_farbig.rgb");
//         playground->setColor(Color(0.4,0.8,0.4,0.1));
//          playground->setPosition(osg::Vec3(20,20,0)); // playground positionieren und generieren
      //   global.obstacles.push_back(playground2);
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
// <<<<<<< main.cpp


//      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(1.08, 8.2, 1.05)); playground->setColor(Color(0.88f,0.4f,0.26f,0.9999f));
//      playground->setPosition(osg::Vec3(0,0,.1));
// =======

   if(narrow){
   //   Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(30.0875, 0.08, 1.3975));
     //   playground->setColor(Color(0.2f,0.2f,0.22f,0.99));
     playground->setColor(Color(0.9,0.1,0.1,0.99));
     //   playground->setTexture("Images/really_white.rgb");
     playground->setPosition(osg::Vec3(20,20,.1));
     //      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(1.0875, 8.8, 1.3975));
     //      playground->setColor(Color(0.88f,0.4f,0.26f,1));
     playground->setPosition(osg::Vec3(20,20,.1));
//>>>>>>> 1.13
     Substance substance;
// <<<<<<< main.cpp
//      substance.toPlastic(40.0);//10
//      substance.toRubber(20);//neue Substance for robot.
//     playground->setGroundSubstance(substance);
//     global.obstacles.push_back(playground);
// /*    double xboxes=0.0;
//     double yboxes=0.0;*/
//     double xboxes=0;//19;//19.0;
//     double yboxes=0;//19;
//     double boxdis=.9;//.45;//1.6;
//     for (double j=0.0;j<xboxes;j++)
//       for(double i=0.0; i<yboxes; i++) {
//         double xsize= .35;//1.0;
//         double ysize= .25;//.25;
//         double zsize=.9;
//         PassiveBox* b =
//           new PassiveBox(odeHandle,
//                          osgHandle, osg::Vec3(xsize,ysize,zsize),0.0);
//         b->setPosition(Pos(boxdis*(i-(xboxes-1)/2.0),boxdis*(j-(yboxes-1)/2.0), 0.01));
//         b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
//         b->setTexture("Images/light_chess.rgb");
//         global.obstacles.push_back(b);
//       }
// =======
     // substance.toPlastic(2.0); //GROUND
       substance.toRubber(40.0);
     //   substancetoMetal(.6);
     playground->setGroundSubstance(substance);
     global.obstacles.push_back(playground);
     /*    double xboxes=0.0;
           double yboxes=0.0;*/
     double xboxes=0;//15;//19.0;
     double yboxes=2;//15;
     double boxdis=1.0;//.9;//.45;//1.63
     for (double i=0.0;i<xboxes;i++)
       for(double j=0.0; j<yboxes; j++) {
         double xsize= .15;//1.0;
         double ysize= .15;//.25;
         double zsize= -i*.01+.5;
        //  PassiveBox* b =
//            new PassiveBox(odeHandle,
//                           osgHandle, osg::Vec3(xsize,ysize,zsize),.9*zsize+.03*i);
         PassiveSphere* b =
           new PassiveSphere(odeHandle,
                             osgHandle,zsize,/*mass=*/.4);//, osg::Vec3(xsize,ysize,zsize),.9*zsize+.03*i);
         b->setPosition(Pos(18.5+(xsize*.02+boxdis)*(i-(xboxes-1)/2.0),20-.0008+(ysize*1.1+boxdis)*(j-(yboxes-1)/2.0),
 (1+i+j)*(zsize+.3)*2));
         b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
         b->setTexture("Images/light_chess.rgb");
         global.obstacles.push_back(b);
       }
   } //narrow ???
   //>>>>>>> 1.13


   for (int i=0; i< humanoids; i++){ //Several humans

     if (i>0) reckturner=false;

     //       ZweiBeinerConf conf = ZweiBeiner::getDefaultConf();
           //       ZweiBeiner* human = new ZweiBeiner(odeHandle, osgHandle,conf, "Humanoid");
     //       human->place(osg::Matrix::translate(4*i,0,2));
     //       global.configs.push_back(human);

     //       Primitive* trunk = human->getMainPrimitive();
     //       fixator = new FixedJoint(trunk, global.environment);
     //       fixator->init(odeHandle, osgHandle);

     SkeletonConf conf = Skeleton::getDefaultConf();

     conf.massfactor   = 1;
     conf.relLegmass = 5;
     conf.relFeetmass = 1;
     conf.relArmmass = 1;//1.0;

     conf.hipJointLimit= 2.9;//2.0; //!
     conf.kneeJointLimit=2.5;//1.911; //!
     conf.hip2JointLimit=.9;//.9; //!0
     conf.armJointLimit=M_PI/1.9;//2;//2.0; //!
     conf.pelvisJointLimit=.3; //!
     conf.hipPower=40;//100;
     conf.hip2Power=50;      //5
     conf.pelvisPower=20;//20;
     conf.kneePower= 25;
     conf.anklePower = 5;
     conf.armPower = 20;//5
     if(reckturner)      conf.armPower = 30;
     conf.powerfactor = .3;// .95;//.65;//5;
     if (reckturner) conf.powerfactor *=.2;
     if (i==0)
       conf.trunkColor=Color(0.1, 0.3, 0.8);
     else        conf.trunkColor=Color(0.9, 0.0, 0.1);
     if(reckturner)
       conf.handsRotating = true;

     conf.useBackJoint = true;

     //    conf.bodyTexture="Images/whitemetal_farbig_small.rgb";
     //     conf.bodyColor=Color(1.0,1.0,0.0);
     std::list<Sensor*> sensors;
     sensors.push_back(new AxisOrientationSensor(AxisOrientationSensor::OnlyZAxis));
     OdeHandle skelHandle=odeHandle;//BODY
     // skelHandle.substance.toMetal(1);
     //  skelHandle.substance.toPlastic(40);//TEST sonst 40
     skelHandle.substance.toRubber(10);//TEST sonst 40
     Skeleton* human0 = new Skeleton(skelHandle, osgHandle,conf, "Humanoid");
//      AddSensors2RobotAdapter* human =
//        new AddSensors2RobotAdapter(skelHandle, osgHandle, human0, sensors);
     OdeRobot* human=human0;
     human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)
                  //   *osg::Matrix::translate(-.2 +2.9*i,0,1));
                  *osg::Matrix::translate(.02*i+19.9,.08*i+19.9,1.0/*7*/ +2.5*i));
     global.configs.push_back(human);


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
     //>>>>>>> 1.13



     // create pointer to controller
     // push controller in global list of configurables
     //       AbstractController *controller = new SineController();
     //       controller->setParam("sinerate",50);
     //       controller->setParam("phaseshift",0);

     DerLinInvertMPIConf cc = DerLinInvertMPI::getDefaultConf();
     //           BasicControllerConf cc = BasicController::getDefaultConf();
     //  AbstractController* controller = new DerLinInvertMPI(cc);

// <<<<<<< main.cpp
//       cc.cInit=.8;

// =======
     cc.cInit= -1.15;// Gehtnicht. TODO
     //>>>>>>> 1.13

// <<<<<<< main.cpp
//           vector<Layer> layers;
//            layers.push_back(Layer(20,0.5,FeedForwardNN::tanh)); // hidden layer
//           // size of output layer is automatically set
//           layers.push_back(Layer(1,1,FeedForwardNN::linear));
//           MultiLayerFFNN* net = new MultiLayerFFNN(0.0, layers, false);// false means no bypass.
//           cc.model = net;

//           layers.clear();
//           layers.push_back(Layer(10,0.5,FeedForwardNN::tanhr)); // hidden layer
//           // size of output layer is automatically set
//           layers.push_back(Layer(1,0.5,FeedForwardNN::tanhr));
//           MultiLayerFFNN* sat = new MultiLayerFFNN(1.0, layers, false);
//           cc.sat   = sat;

// //           //Elman Net
// //           layers.clear();
// //           layers.push_back(Layer(40,0.5,Elman::tanhr)); // hidden layer
// //           // size of output layer is automatically set
// //           layers.push_back(Layer(1,0.5,Elman::tanh));
// //           Elman* sat = new Elman(1, layers,true,true, false);
// //           cc.sat   = sat;

//           cc.useS=false;
//           AbstractController* controller = new DerLinInvertMPI(cc);
//           //  AbstractController* controller = new BasicController(cc);
// =======
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
     layers.push_back(Layer(40,0.5,Elman::tanh));//r)); // hidden layer
     // size of output layer is automatically set
     layers.push_back(Layer(1,0.5,Elman::tanh));
     Elman* sat = new Elman(1, layers,true,true, false);
     cc.sat   = sat;

     cc.useS=false;
     AbstractController* controller = new DerLinInvertMPI(cc);
     //  AbstractController* controller = new BasicController(cc);
     //>>>>>>> 1.13


// <<<<<<< main.cpp
//       //     controller->setParam("adaptrate",0);
//       //     controller->setParam("rootE",3);
//            controller->setParam("epsC",.1);
//            controller->setParam("epsSat",0.02);
//            controller->setParam("epsA",0.01);
//       controller->setParam("steps",1);
//            controller->setParam("s4avg",2);
//            controller->setParam("s4delay",2);
//            controller->setParam("teacher",0.0);
//       //     controller->setParam("dampS",0.0001);
//       //     controller->setParam("dampA",0.00003);
//           controller->setParam("weighting",1);
//           controller->setParam("noise",0);
//           controller->setParam("dampA",0.01);//multiplied by epsA
//           controller->setParam("dampC",0.00001);

//       global.configs.push_back(controller);

//       // create pointer to one2onewiring
//         One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
//       // DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
// //       c.useId = true;
// //       c.useFirstD = false;
// //       DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );
// =======
     //     controller->setParam("adaptrate",0);
     //     controller->setParam("rootE",3);
     controller->setParam("epsC",0.05);
     controller->setParam("epsSat",0.02);
     controller->setParam("epsA",0.003);
     controller->setParam("steps",1);
     controller->setParam("s4avg",10);
     controller->setParam("s4delay",4);
     controller->setParam("teacher",0.0);
     controller->setParam("dampC",0.00001);
     controller->setParam("dampA",0.003);
     controller->setParam("weighting",1);
     controller->setParam("noise",0);
     controller->setParam("noiseY",0.1);
     controller->setParam("zetaupdate",0);

     controller->setParam("PIDint",.3);
          controller->setParam("intstate",1);

     global.configs.push_back(controller);

     // create pointer to one2onewiring
     //     One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
     DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
     c.useId = true;
     c.useFirstD = false;
     DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise(.2) );
     //>>>>>>> 1.13



     // create pointer to agent
     // initialize pointer with controller, robot and wiring
     // push agent in globel list of agents
     OdeAgent* agent = new OdeAgent(global);
     agent->init(controller, human, wiring);
     //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
     global.agents.push_back(agent);

     //
   }// Several humans end





    //****** SNAKES **********/
    //creation of normal   snakes
   for(int i=0; i<snakes; i++){

// <<<<<<< main.cpp
//       //****************/
//       SchlangeConf conf = Schlange::getDefaultConf();
//       double snakesize = .6;
//       conf.segmMass   = .8;
//       conf.segmLength= .8 * snakesize;// 0.8;
//         conf.segmDia=.15 *snakesize;
//       conf.motorPower= 2 * snakesize;
//       conf.segmNumber = 13+4*i;//-i/2;
//       // conf.jointLimit=conf.jointLimit*3;
//       conf.jointLimit=conf.jointLimit* 1.6;
//       conf.frictionGround=0.03;// +((double)i)/100;
//       conf.frictionJoint=0.001;
//       //PlattfussSchlange* schlange1;
//       SchlangeServo2* schlange1;
//       if (i==0) {
//         schlange1 =

//           new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.1, 0.3, 0.8)),
//                                //  new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(1.0, 1.0, 1.0)),
//                                conf, "S1");
//       } else {
//         schlange1 =
//           new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.1, 0.3, 0.8)),
//                                // new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(0.8, 0.4, .3)),
//                                conf, "S2");
//       }
//       //Positionieren und rotieren
//       schlange1->place(osg::Matrix::rotate(M_PI/2,0, 1, 0)*
//                         osg::Matrix::translate(-.7+0.7*i,-2*i,1+(i+1)*(.2+conf.segmNumber)/2.0/*+2*/));
//                        // osg::Matrix::translate(5-i,2 + i*2,height+2));
//       schlange1->setTexture("Images/whitemetal_farbig_small.rgb");
//       if (i==0) {
//         schlange1->setHeadColor(Color(1.0,0,0));
//       } else {
//         schlange1->setHeadColor(Color(0,1.0,0));
//       }
//       //   Primitive* trunk = schlange1->getMainPrimitive();

//    //fixator = new FixedJoint(trunk, global.environment);
//        //  fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) ,                    Axis(0,0,1), Axis(0,1,0));
// //       fixator->init(odeHandle, osgHandle);


//       //      AbstractController *controller = new InvertMotorNStep();
//       // DerBigControllerConf cconf = DerBigController::getDefaultConf();
//       // DerControllerConf cconf = DerController::getDefaultConf();
//             DerLinInvertMPIConf cconf = DerLinInvertMPI::getDefaultConf();
//       // cconf.useS=true;
//       cconf.cInit=1.05;

//           vector<Layer> layers;
//           //    layers.push_back(Layer(10,0.5,FeedForwardNN::tanh)); // hidden layer
//            //  size of output layer is automatically set
//           layers.push_back(Layer(1,1,FeedForwardNN::linear));
//           MultiLayerFFNN* net = new MultiLayerFFNN(0.0, layers, false);// false means no bypass.
//           cconf.model=net;
// =======
     //****************/
     SchlangeConf conf = Schlange::getDefaultConf();
     double snakesize = .6;
     conf.segmMass   = .8;
     conf.segmLength= .8 * snakesize;// 0.8;
     conf.segmDia=.15 *snakesize;
     conf.motorPower= 2 * snakesize;
     conf.segmNumber = 14-4*i;//-i/2;
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
                      osg::Matrix::translate(20/*1*/ -.7*i,20-.2*i,1+(i+1)*(.2+conf.segmNumber)/2.0/*+2*/));
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
     DerLinInvertMPIConf cconf = DerLinInvertMPI::getDefaultConf();
     // cconf.useS=true;
     cconf.cInit=1.0;

     vector<Layer> layers;
     //    layers.push_back(Layer(10,0.5,FeedForwardNN::tanh)); // hidden layer
     //  size of output layer is automatically set
     layers.push_back(Layer(1,1,FeedForwardNN::linear));
     MultiLayerFFNN* net = new MultiLayerFFNN(0.0, layers, false);// false means no bypass.
     cconf.model=net;
     //>>>>>>> 1.13

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
     AbstractController* controller = new DerLinInvertMPI(cconf);

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
     controller->setParam("epsC",0.1);
     controller->setParam("epsA",0.01);
     controller->setParam("adaptrate",0.0);//0.005);
     controller->setParam("rootE",3);
     controller->setParam("logaE",0);
     controller->setParam("epsSat",0.02);
     controller->setParam("weighting",1);

     // controller->setParam("desens",0.0);
     controller->setParam("s4delay",2.0);
     controller->setParam("s4avg",3.0);

     controller->setParam("factorB",0.0);
     controller->setParam("noiseB",0.0);
     controller->setParam("zetaupdate",0.0);

// <<<<<<< main.cpp
//       controller->setParam("frictionjoint",0.01);
//       controller->setParam("frictionground",0.03);
//       controller->setParam("teacher", 0.002);
//           controller->setParam("dampA",0.0001);
//           controller->setParam("dampC",0.000001);
// =======
     controller->setParam("frictionjoint",0.01);
     controller->setParam("frictionground",0.03);
     controller->setParam("teacher", 0.0);
     controller->setParam("dampA",0.01);
     controller->setParam("dampC",0.00001);
     //>>>>>>> 1.13

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


     DerLinInvertMPIConf cc = DerLinInvertMPI::getDefaultConf();
     // AbstractController* controller = new DerLinInvertMPI(cc);

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
     AbstractController* controller = new DerLinInvertMPI(cc);

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

     controller->setParam("factorB",0.0);
     controller->setParam("noiseB",0.0);

     controller->setParam("frictionjoint",0.01);
     controller->setParam("teacher", 0.0);

   }//creation of flatsnakes End


   /******* S L I D E R - W H E E L I E *********/
   for(int i=0; i < wheelies; i++) {
     SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
     mySliderWheelieConf.segmNumber=12;
     mySliderWheelieConf.motorPower=1.4;
     mySliderWheelieConf.frictionGround=0.8;
     mySliderWheelieConf.sliderLength=.8;
     mySliderWheelieConf.segmLength=0.6;
     OdeRobot* robot = new SliderWheelie(odeHandle, osgHandle,
                                         mySliderWheelieConf, "sliderWheelie1");
     double height = 2;
     robot->place(Pos(23,23, height));

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
          dBodyAddForce(body->getBody(), d.x()*centerforce, d.y()*centerforce,  d.z()*centerforce*.8);
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

