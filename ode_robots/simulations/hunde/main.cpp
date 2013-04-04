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
 *   Revision 1.4  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.3  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.2  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.1  2008/01/29 10:01:37  der
 *   first version
 *
 *   Revision 1.9  2007/12/13 17:00:02  martius
 *   walk controller
 *
 *   Revision 1.8  2007/11/21 13:18:10  der
 *   ralfs aenderungen
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
#include <selforg/feedbackwiring.h>
#include <selforg/wiringsequence.h>


// used robot
#include "vierbeiner.h"

// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/joint.h>

// used controller
//#include <selforg/invertnchannelcontroller.h>
#include <selforg/derpseudosensor.h>
#include <selforg/derbigcontroller.h>

#include <selforg/dercontroller.h>
#include <selforg/invertmotorbigmodel.h>
#include <selforg/multilayerffnn.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/universalcontroller.h>
#include "walkcontroller.h"
/************/

#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/nimm2.h>
//#include <ode_robots/derivativewiring.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:


  Joint* fixator;
  AbstractObstacle* playground;
  double hardness;
  Substance s;
  AbstractController *teachcontroller;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));

    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
<<<<<<< main.cpp

    global.odeConfig.setParam("controlinterval",5);
    global.odeConfig.setParam("simstepsize",0.003);
    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("realtimefactor",3);

    //    global.odeConfig.setParam("gravity", 0);
=======
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("noise",0.1);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("gravity", -3);
>>>>>>> 1.9
    //    global.odeConfig.setParam("cameraspeed", 250);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
<<<<<<< main.cpp
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double        height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)

    s.toPlastic(0.9);

    double diam = .8;
    int anzgrounds = 4;
=======
    s.toPlastic(0.9);
    double scale = 20;
    double height = 0;
    int anzgrounds=1;
>>>>>>> 1.9
    for (int i=0; i< anzgrounds; i++){
<<<<<<< main.cpp
      playground = new Playground(odeHandle, osgHandle, osg::Vec3(40+4*i, .2, .15+0.15*i), 1, i==(anzgrounds-1));

=======
      playground = new Playground(odeHandle, osgHandle,
             osg::Vec3((4+4*i)*scale, .2, (.15+0.15*i)*height), 1, i==(anzgrounds-1));
>>>>>>> 1.9
      OdeHandle myhandle = odeHandle;

      myhandle.substance.toFoam(10);
      // playground = new Playground(myhandle, osgHandle, osg::Vec3(/*base length=*/50.5,/*wall = */.1, /*height=*/1));
      playground->setPosition(osg::Vec3(0,0,0.2)); // playground positionieren und generieren
      playground->setSubstance(s);
      // playground->setPosition(osg::Vec3(i,-i,0)); // playground positionieren und generieren
    //global.obstacles.push_back(playground);
    }
    global.obstacles.push_back(playground);


    // add passive spheres as obstacles
    for (int i=0; i< 0/*2*/; i+=2){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.3);
      // s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      s1->setPosition(osg::Vec3(0,0,10+i*5));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }
<<<<<<< main.cpp
    ///////// CREATION OF DOGS --- Begin
=======


    teachcontroller = new WalkController();
    teachcontroller->init(12,12);
>>>>>>> 1.9
    for (int i=0; i< 1/*2*/; i++){ //Several dogs

    VierBeinerConf conf = VierBeiner::getDefaultConf();
        //  conf.hipJointLimit = M_PI/8;
    conf.legNumber = 8;
    conf.hipPower = 6;
    conf.kneePower = 4;


<<<<<<< main.cpp
    VierBeiner* dog = new VierBeiner(odeHandle, osgHandle,conf, "Dog");
=======
    OdeHandle doghandle = odeHandle;
    doghandle.substance.toRubber(10);
    VierBeiner* dog = new VierBeiner(doghandle, osgHandle,conf, "Dog");
>>>>>>> 1.9
    //dog->place(osg::Matrix::translate(0,0,0.15));
    dog->place(osg::Matrix::translate(0,0,.5 + 4*i));
    global.configs.push_back(dog);

    Primitive* trunk = dog->getMainPrimitive();
    fixator = new FixedJoint(trunk, global.environment);
    fixator->init(odeHandle, osgHandle);

    // use Nimm4 vehicle as robot:
    // - create pointer to nimm4 (with odeHandle and osg Handle and possible other settings, see nimm4.h)
    // - place robot
    //OdeRobot* vehiInvertMotorSpacecle = new Nimm4(odeHandle, osgHandle);
    //vehicle->place(Pos(0,2,0));

<<<<<<< main.cpp
    // create pointer ^y to controller
    // push controller in global list of configurables
    // AbstractController *controller = new SineController();
    //    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
=======
    // create pointer to controller

    // AbstractController *controller = new SineController();
    //AbstractController *controller = new WalkController();
    // InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    // cc.useS=false;
    // AbstractController *controller = new InvertMotorNStep(cc);

    // AbstractController *controller = new SineController();
    //   InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
>>>>>>> 1.9
    //    cc.useS=true;
<<<<<<< main.cpp
    //    AbstractController *controller = new InvertMotorNStep(cc);
    DerPseudoSensorConf cc = DerPseudoSensor::getDefaultConf();
    //DerBigControllerConf cc = DerBigController::getDefaultConf();
=======
    //  AbstractController *controller = new InvertMotorNStep(cc);
    // DerBigControllerConf cc = DerBigController::getDefaultConf();

    //    DerControllerConf cc = DerController::getDefaultConf();
>>>>>>> 1.9
    //    InvertMotorBigModelConf cc = InvertMotorBigModel::getDefaultConf();
<<<<<<< main.cpp
    vector<Layer> layers;
    //    layers.push_back(Layer(20,0.5,FeedForwardNN::tanh)); // hidden layer
    // size of output layer is automatically set
    layers.push_back(Layer(1,1,FeedForwardNN::linear));
    MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass.
    cc.model=net;
    cc.useS=false;
    //    cc.someInternalParams=false;

    //  AbstractController* controller = new DerBigController(cc);
     AbstractController* controller = new DerPseudoSensor(cc);
=======
   //  vector<Layer> layers;
//      layers.push_back(Layer(20,0.5,FeedForwardNN::tanh)); // hidden layer
//     // size of output layer is automatically set
//     layers.push_back(Layer(1,1,FeedForwardNN::linear));
//     MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass.
//     cc.model=net;
//     cc.useS=true;
    //cc.useS=false;
    // AbstractController* controller = new DerBigController(cc);

    //     AbstractController* controller = new DerController(cc);
>>>>>>> 1.9
        // AbstractController* controller = new InvertMotorBigModel(cc);
<<<<<<< main.cpp
     // controller->setParam("sinerate",50);
     //controller->setParam("phaseshift",1);
=======

    AbstractController* controller = new UniversalController(UniversalController::getDefaultNetConf());

    controller->setParam("sinerate",50);
    controller->setParam("phaseshift",1);
>>>>>>> 1.9
    controller->setParam("adaptrate",0);
    controller->setParam("rootE",3);
<<<<<<< main.cpp
    controller->setParam("epsC",0.02);
    controller->setParam("epsA",0.02);
=======
    controller->setParam("epsC",0.1);
    controller->setParam("epsA",0.1);
>>>>>>> 1.9
    controller->setParam("steps",1);
    controller->setParam("s4avg",2);
    controller->setParam("s4delay",2);
    controller->setParam("teacher",0);
    controller->setParam("dampS",0.0001);
    controller->setParam("dampA",0.00003);
<<<<<<< main.cpp
    controller->setParam("noiseY",0.03);
=======
    controller->setParam("continuity",0.5);
>>>>>>> 1.9
    //    controller->setParam("kwta",4);
    //    controller->setParam("inhibition",0.01);

    // TEST
    controller->setParam("epsC",0.0);
    controller->setParam("epsA",0.1);
    controller->setParam("steps",1);
    controller->setParam("s4avg",2);
    controller->setParam("s4delay",2);
    controller->setParam("teacher",0.1);
    controller->setParam("continuity",0);


    global.configs.push_back(controller);

    // create pointer to one2onewiring
    //AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    // feedback connection and blind channels
    AbstractWiring* wiring =
      new WiringSequence(new FeedbackWiring(new ColorUniformNoise(0.1),
                                            FeedbackWiring::Motor,0.5),
                         new One2OneWiring(0, false, 2));

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, dog, wiring);
    //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
    global.agents.push_back(agent);

    //
    }// Several dogs end


    ///////// CREATION OF DOGS -- End



  }


  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control && !pause && teachcontroller){
      double sensors[24];
      double motors[24];
      AbstractController* contr = globalData.agents.front()->getController();
      InvertMotorNStep* c = dynamic_cast<InvertMotorNStep*>(contr);
      if(c){
        int len = c->getSensorNumber();
        teachcontroller->step(sensors, len, motors, len);
        c->setMotorTeachingSignal(motors,len);
      }
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

