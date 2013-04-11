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
 *   Revision 1.6  2011-10-14 09:36:18  martius
 *   snakes have no frictionGround parameter anymore, since it was not used,
 *    use the substances now
 *
 *   Revision 1.5  2011/06/03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
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
 *   Revision 1.1  2007/10/29 12:43:59  robot3
 *   MAIN WITH DIFFERET OBJECTS AND DERCONTROLLER
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
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>

// used robot
#include "skeleton.h"
//#include "derskeleton.h"

// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivemesh.h>
#include <ode_robots/joint.h>

// used controller
//#include <selforg/invertnchannelcontroller.h>
#include <selforg/dercontroller/derbigcontroller.h>
#include <selforg/dercontroller/dercontroller.h>
#include <selforg/invertmotorbigmodel.h>
#include <selforg/multilayerffnn.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
/************/

#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/nimm2.h>
//#include <ode_robots/derivativewiring.h>

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
  AbstractObstacle* playground;
  double hardness;
  Substance s;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-1.66705, 4.47234, 3.86184),  Pos(-158.908, -10.5863, 0));

  // Set number of robots:


    // int plattfuesse = 0;
    // int flatsnakes  = 0;
    int snakes = 0;
    //int sphericalsIR = 0;
    //int sphericalsXYZ = 0;
    //int hurlings = 0;
    //int cigars = 0;
    int wheelies = 0;
    int humanoids = 1;
    int sphericalsIR = 1;


    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("noiseY",0.03);
    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.001);
    global.odeConfig.setParam("gravity",-0.7);
    //    global.odeConfig.setParam("cameraspeed", 250);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double        height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)
    // s.toMetal(0.9);
     s.toMetal(90);
     //s.toRubber(30);

    int anzgrounds=2;
    for (int i=0; i< anzgrounds; i++){
      //    playground = new Playground(odeHandle, osgHandle, osg::Vec3(30+4*i, .2, .95+0.15*i), 1, i==(anzgrounds-1));
      playground = new Playground(odeHandle, osgHandle, osg::Vec3(50.3+4*i, .9, 1.95+0.15*i), 1, i==(anzgrounds-1));
      OdeHandle myhandle = odeHandle;
      //      myhandle.substance.toFoam(10);
      // playground = new Playground(myhandle, osgHandle, osg::Vec3(/*base length=*/50.5,/*wall = */.1, /*height=*/1));
      playground->setPosition(osg::Vec3(0,0,0.2)); // playground positionieren und generieren
      playground->setSubstance(s);
      // playground->setPosition(osg::Vec3(i,-i,0)); // playground positionieren und generieren
    //global.obstacles.push_back(playground);
    }
    global.obstacles.push_back(playground);


//     //
//     AbstractObstacle* m = new PassiveMesh(odeHandle, osgHandle, "Meshes/skeleton/Chest_center.wrl",0.4,1);
//     m->setPosition(osg::Vec3(1,1,1));
//     global.obstacles.push_back(m);


    ///********* Creation of HUMANOIDS  ****************

    for (int i=0; i< humanoids; i++){ //Several humans

//       ZweiBeinerConf conf = ZweiBeiner::getDefaultConf();

//       ZweiBeiner* human = new ZweiBeiner(odeHandle, osgHandle,conf, "Humanoid");
//       human->place(osg::Matrix::translate(4*i,0,2));
//       global.configs.push_back(human);

//       Primitive* trunk = human->getMainPrimitive();
//       fixator = new FixedJoint(trunk, global.environment);
//       fixator->init(odeHandle, osgHandle);

      SkeletonConf conf = Skeleton::getDefaultConf();

      double powerfactor = 2;//.3

     //  conf.bodyMass   = 0;
//       conf.relLegmass = 0;
//       conf.relFeetmass = 1000;
//       conf.relArmmass = 0.0;

//       conf.hipJointLimit=0.001; //!
       conf.kneeJointLimit=1.9; //!
       conf.hip2JointLimit=.9; //!
       conf.armJointLimit=1.2; //!
//       conf.ankleJointLimit=0.001; //!
       conf.pelvisJointLimit=.5; //!
            conf.hipPower=50 * powerfactor;
            conf.hip2Power=20 * powerfactor;
            conf.pelvisPower=200 * powerfactor;
      conf.kneePower= 40 * powerfactor;
      conf.anklePower= 10 * powerfactor;
      conf.armPower = 20 * powerfactor;
      conf.anklePower = 10 * powerfactor;
      OdeHandle skelHandle=odeHandle;
      // skelHandle.substance.toMetal(1);
      skelHandle.substance.toRubber(80);
      Skeleton* human = new Skeleton(skelHandle, osgHandle,conf, "Humanoid");
      human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)
                   *osg::Matrix::translate(4*i,0,2));
      global.configs.push_back(human);

      Primitive* trunk = human->getMainPrimitive();

       fixator = new FixedJoint(trunk, global.environment);
      //  fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) ,                    Axis(0,0,1), Axis(0,1,0));
      fixator->init(odeHandle, osgHandle);

      // create pointer to controller
      // push controller in global list of configurables
      //  AbstractController *controller = new SineController();
      //  controller->setParam("sinerate",50);
      //  controller->setParam("phaseshift",0);
       //   InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
//          cc.useS=false;
//          AbstractController *controller = new InvertMotorNStep(cc);
//XX            DerBigControllerConf cc = DerBigController::getDefaultConf();
            DerControllerConf cc = DerController::getDefaultConf();
            //     InvertMotorBigModelConf cc = InvertMotorBigModel::getDefaultConf();
 //XX         vector<Layer> layers;
          //          layers.push_back(Layer(6/*20*/,0.5,FeedForwardNN::linear/*tanh*/)); // hidden layer
           //  size of output layer is automatically set
//XX          layers.push_back(Layer(1,1,FeedForwardNN::linear));
 //XX         MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass.
    //XX      cc.model=net;
          //cc.useS=true;
          cc.useS=false;
      //XX           AbstractController* controller = new DerBigController(cc);
           AbstractController* controller = new DerController(cc);
              // AbstractController* controller = new InvertMotorBigModel(cc);
          controller->setParam("adaptrate",0);
          controller->setParam("rootE",3);
          controller->setParam("epsC",0.3);
          controller->setParam("epsA",0.05);
          controller->setParam("steps",1);
          controller->setParam("s4avg",2);
          controller->setParam("s4delay",2);
          controller->setParam("teacher",0);

          controller->setParam("dampS",0.0001);
          controller->setParam("dampA",0.0);
          controller->setParam("dampC",0.0);
          controller->setParam("teaching",false);
          //    controller->setParam("kwta",4);
          //    controller->setParam("inhibition",0.01);

      global.configs.push_back(controller);

      // create pointer to one2onewiring
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

      // create pointer to agent
      // initialize pointer with controller, robot and wiring
      // push agent in globel list of agents
      OdeAgent* agent = new OdeAgent(global);
      agent->init(controller, human, wiring);
      //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
      global.agents.push_back(agent);

      //
    }// Several humans end

 double height = 0.5;

    for(int i=0; i<0; i++){

      PassiveBox* b =
        new  PassiveBox(odeHandle,
                        osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)),
                        osg::Vec3(1.0, 1.0, 1.0), /*mass=*/9);
      b->setPosition(Pos(i*0.3, i*0.5, height));
      b->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(b);
    }
    // Creation of passive spheres
    //****** PASSIVE SPHERES **********/
   for(int i=0; i<0; i++){

      PassiveSphere* s =
        new PassiveSphere(odeHandle,
                          osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)),
                          /*Diameter=*/.4+i*0.1, /*Mass=*/.8);
      s->setPosition(Pos(i, -i, height));
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);
    }

    //******Creation of  SNAKES **********/
    //creation of normal   snakes
    for(int i=0; i<snakes; i++){

      //****************/
      SchlangeConf conf = Schlange::getDefaultConf();
      conf.segmMass   = .2;
      conf.segmLength= 1.9;// 0.8;
        conf.segmDia=.6;
      conf.motorPower=.5;
      conf.segmNumber = 12+2*i;//-i/2;
      // conf.jointLimit=conf.jointLimit*3;
      conf.jointLimit=conf.jointLimit* 1.6;
      conf.frictionJoint=0.02;
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
                        osg::Matrix::translate(-.7+0.7*i,0,(i+1)*(.2+conf.segmNumber)/2.0/*+2*/));
                       // osg::Matrix::translate(5-i,2 + i*2,height+2));
      schlange1->setTexture("Images/whitemetal_farbig_small.rgb");
      if (i==0) {
        schlange1->setHeadColor(Color(1.0,0,0));
      } else {
        schlange1->setHeadColor(Color(0,1.0,0));
      }


      //      AbstractController *controller = new InvertMotorNStep();
      // DerBigControllerConf cconf = DerBigController::getDefaultConf();
      DerControllerConf cconf = DerController::getDefaultConf();
      // cconf.useS=true;
      cconf.cInit=1.2;

        //   vector<Layer> layers;
//            layers.push_back(Layer(10,0.5,FeedForwardNN::tanh)); // hidden layer
//            //  size of output layer is automatically set
//           layers.push_back(Layer(1,1,FeedForwardNN::linear));
//           MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass.
//           cconf.model=net;

      // AbstractController *controller = new DerBigController(cconf);
      AbstractController *controller = new DerController(cconf);
      //  controller->setParam("fantcontrol",200);
      // controller->setParam("fantcontrollen",50);
      //AbstractController *controller = new SineController();

      //  AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); //Only this line for one2Onewiring
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = true;
      c.useFirstD = false;
      DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );

      OdeAgent* agent = new OdeAgent(global);
      agent->init(controller, schlange1, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(schlange1);

      controller->setParam("steps",1);
      controller->setParam("epsC",0.1);
      controller->setParam("epsA",0.1);
      controller->setParam("adaptrate",0.0);//0.005);
      controller->setParam("rootE",3);
      controller->setParam("logaE",0);

      // controller->setParam("desens",0.0);
      controller->setParam("s4delay",1.0);
      controller->setParam("s4avg",1.0);

      controller->setParam("factorB",0.0);
      controller->setParam("noiseB",0.0);

      controller->setParam("frictionjoint",0.01);
      controller->setParam("frictionground",0.01);
      controller->setParam("teacher", 0.0);

    }//creation of snakes End

    // Creation of spherical robots:
    //****** SPHERICALS IR **********/
    for(int i=0; i<sphericalsIR; i++){
      OdeRobot* sphere1;
      //Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      // conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
      conf.diameter=1.4;
      conf.spheremass = 2;
      conf.irAxis1=true;
      conf.irAxis2=true;
      conf.irAxis3=true;
      conf.pendularrange=0.25;
      // conf.irCharacter=0.5;
      sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0.0)),
                                                conf, "Sphere1", 0.5);
//      sphere1 = new ForcedSphere(odeHandle, osgHandle.changeColor(Color(1.0,0.0,0.0))
//                                 , ForcedSphere::getDefaultConf(), "FSphere");

      // sphere1->place ( Pos(-3,1/2,3+2*i));
      sphere1->place ( Pos(5, i*2, height));
      AbstractController* controller = new DerController();
      //AbstractController* controller = new InvertMotorNStep();
      //      controller->setParam("steps", 2);
      controller->setParam("steps", 1);
      controller->setParam("adaptrate", 0.0);
      controller->setParam("nomupdate", 0.0);
      controller->setParam("epsC", 0.1);
      controller->setParam("epsA", 0.1);
      controller->setParam("rootE", 3);
      controller->setParam("factorB", 0.0);
      controller->setParam("teacher", 0.0);

      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = false;
      c.useFirstD = true;
      DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise(0.5) );


      // One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
      OdeAgent* agent = new OdeAgent ( plotoptions );
      agent->init ( controller , sphere1 , wiring );
      //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
    } //End  Creation of spherical robots.



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
      controller->setParam("epsC",.001);
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
    }


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

