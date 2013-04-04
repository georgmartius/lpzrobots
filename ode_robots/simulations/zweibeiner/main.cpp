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
 *   Revision 1.6  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.5  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.4  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
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
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>

// used robot
#include "skeleton.h"

// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivemesh.h>
#include <ode_robots/joint.h>

// used controller
//#include <selforg/invertnchannelcontroller.h>
#include <selforg/derbigcontroller.h>
#include <selforg/dercontroller.h>
#include <selforg/invertmotorbigmodel.h>
#include <selforg/multilayerffnn.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
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

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-1.66705, 4.47234, 3.86184),  Pos(-158.908, -10.5863, 0));

    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.001);
        global.odeConfig.setParam("gravity", -1);
    //    global.odeConfig.setParam("cameraspeed", 250);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double        height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)
    s.toMetal(0.9);

    int anzgrounds=1;
    for (int i=0; i< anzgrounds; i++){
      playground = new Playground(odeHandle, osgHandle, osg::Vec3(100.0+4*i, .2, 1.0+0.15*i), 1, i==(anzgrounds-1));
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




    for (int i=0; i< 1/*2*/; i++){ //Several humans

//       ZweiBeinerConf conf = ZweiBeiner::getDefaultConf();

//       ZweiBeiner* human = new ZweiBeiner(odeHandle, osgHandle,conf, "Humanoid");
//       human->place(osg::Matrix::translate(4*i,0,2));
//       global.configs.push_back(human);

//       Primitive* trunk = human->getMainPrimitive();
//       fixator = new FixedJoint(trunk, global.environment);
//       fixator->init(odeHandle, osgHandle);

      SkeletonConf conf = Skeleton::getDefaultConf();

//       conf.hipJointLimit=0.001; //!
//       conf.kneeJointLimit=0.001; //!
//       conf.hip2JointLimit=0.001; //!
//       conf.armJointLimit=0.001; //!
//       conf.ankleJointLimit=0.001; //!
//       conf.pelvisJointLimit=0.001; //!
      OdeHandle skelHandle=odeHandle;
      skelHandle.substance.toMetal(1);
      Skeleton* human = new Skeleton(skelHandle, osgHandle,conf, "Humanoid");
      human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)
                   *osg::Matrix::translate(4*i,0,2));
      global.configs.push_back(human);

      Primitive* trunk = human->getMainPrimitive();
      fixator = new FixedJoint(trunk, global.environment);
      fixator->init(odeHandle, osgHandle);

      // create pointer to controller
      // push controller in global list of configurables
      //AbstractController *controller = new SineController();
      //controller->setParam("sinerate",50);
      //controller->setParam("phaseshift",0);
      //    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
      //  cc.useS=false;
      //   AbstractController *controller = new InvertMotorNStep(cc);
      //     DerBigControllerConf cc = DerBigController::getDefaultConf();

           DerControllerConf cc = DerController::getDefaultConf();
           AbstractController* controller = new DerController(cc);
  //     //    InvertMotorBigModelConf cc = InvertMotorBigModel::getDefaultConf();
      //     vector<Layer> layers;
      //     // layers.push_back(Layer(20,0.5,FeedForwardNN::tanh)); // hidden layer
      //     // size of output layer is automatically set
      //     layers.push_back(Layer(1,1,FeedForwardNN::linear));
      //     MultiLayerFFNN* net = new MultiLayerFFNN(0.01, layers, false);// false means no bypass.
      //     cc.model=net;
      //     cc.useS=true;
      //     //cc.useS=false;
      //     // AbstractController* controller = new DerBigController(cc);
      //         // AbstractController* controller = new InvertMotorBigModel(cc);
      //     controller->setParam("adaptrate",0);
      //     controller->setParam("rootE",3);
      //     controller->setParam("epsC",0.1);
      //     controller->setParam("epsA",0.2);
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

      // create pointer to agent
      // initialize pointer with controller, robot and wiring
      // push agent in globel list of agents
      OdeAgent* agent = new OdeAgent(global);
      agent->init(controller, human, wiring);
      //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
      global.agents.push_back(agent);

      //
    }// Several humans end



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

