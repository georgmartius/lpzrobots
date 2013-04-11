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
 *   Revision 1.3  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.2  2007/04/03 11:27:07  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2007/02/23 09:37:50  der
 *   *** empty log message ***
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
#include "vierbeiner.old.h"

// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/joint.h>

// used controller
//#include <selforg/invertnchannelcontroller.h>
#include <selforg/derbigcontroller.h>
#include <selforg/invertmotorbigmodel.h>
#include <selforg/multilayerffnn.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
/************/

#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/octaplayground.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:


  Joint* fixator;
  AbstractObstacle* playground;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));
    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("realtimefactor",3);
    //    global.odeConfig.setParam("gravity", 0);
    //    global.odeConfig.setParam("cameraspeed", 250);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double        height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)
    playground = new Playground(odeHandle, osgHandle, osg::Vec3(20, 0.2, 0.4));
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);
    //     double diam = .8;
//     OctaPlayground* playground3 = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter*/10*diam, .2*diam,/*Height*/ 2), 12,false);
//       playground3->setColor(Color(.0,0.2,1.0,0.1));
//       playground3->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
//      global.obstacles.push_back(playground3);


//      OctaPlayground* playground4 = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter*/11.5 *diam,.1,/*Height*/ 1), 12,true); //false heisst ohne Schatten
//        playground4->setColor(Color(.2,.2,.2,0.1));
//        playground4->setGroundTexture("Images/really_white.rgb");
//        playground4->setGroundColor(Color(255.0f/255.0f,200.0f/255.0f,21.0f/255.0f));
//        playground4->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
//       global.obstacles.push_back(playground4);




    // add passive spheres as obstacles
    // - create pointer to sphere (with odehandle, osghandle and
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere
    // - set a texture for the sphere
    // - add sphere to list of obstacles
    for (int i=0; i< 0/*2*/; i+=2){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.3);
      // s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      s1->setPosition(osg::Vec3(0,0,10+i*5));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }

    VierBeinerOldConf conf = VierBeinerOld::getDefaultConf();
    //    conf.hipJointLimit = M_PI/8;
    conf.legNumber = 4;
    conf.motorPower = 5;
    conf.kneePower = 5;
    VierBeinerOld* dog = new VierBeinerOld(odeHandle, osgHandle,conf, "Dog");
    dog->place(osg::Matrix::translate(0,0,0.15));
    global.configs.push_back(dog);

    Primitive* trunk = dog->getMainPrimitive();
    fixator = new FixedJoint(trunk, global.environment);
    fixator->init(odeHandle, osgHandle);

    // use Nimm4 vehicle as robot:
    // - create pointer to nimm4 (with odeHandle and osg Handle and possible other settings, see nimm4.h)
    // - place robot
    //OdeRobot* vehiInvertMotorSpacecle = new Nimm4(odeHandle, osgHandle);
    //vehicle->place(Pos(0,2,0));

    // create pointer to controller
    // push controller in global list of configurables
    // AbstractController *controller = new SineController();
    //    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    //    cc.useS=true;
    //    AbstractController *controller = new InvertMotorNStep(cc);
    DerBigControllerConf cc = DerBigController::getDefaultConf();
    //    InvertMotorBigModelConf cc = InvertMotorBigModel::getDefaultConf();
    vector<Layer> layers;
    layers.push_back(Layer(20,0.5,FeedForwardNN::tanh)); // hidden layer
    // size of output layer is automatically set
    layers.push_back(Layer(1,1,FeedForwardNN::linear));
    MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, true);
    cc.model=net;
    cc.useS=true;
    AbstractController* controller = new DerBigController(cc);
    //AbstractController* controller = new InvertMotorBigModel(cc);
    controller->setParam("sinerate",50);
    controller->setParam("phaseshift",1);
    controller->setParam("adaptrate",0);
    controller->setParam("rootE",3);
    controller->setParam("epsC",0.01);
    controller->setParam("epsA",0.01);
    controller->setParam("steps",1);
    controller->setParam("s4avg",2);
    controller->setParam("teacher",0);
    //    controller->setParam("kwta",4);
    //    controller->setParam("inhibition",0.01);

    global.configs.push_back(controller);

    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, dog, wiring);
    global.agents.push_back(agent);


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

