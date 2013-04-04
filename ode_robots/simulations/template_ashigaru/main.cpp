/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
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

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
#include <ode_robots/passivesphere.h>  // passive balls
#include <ode_robots/passivecapsule.h> // passive capsules

// controller
//#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>

#include <selforg/noisegenerator.h> // include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/one2onewiring.h>  // simple wiring

// robots
#include <ode_robots/ashigaru.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;



class ThisSim : public Simulation {
public:
  AbstractController* controller;
  OdeRobot* robot;


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos //(Pos(12.1211, 5.91774, 7.22559),  Pos(110.806, -12.6131, 0));
                     (Pos(1.68218, 1.46902, 0.740094),  Pos(130.455, -13.4935, 0));

    // initialization
    // - set global noise to 0.1
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity", 0); // no gravity

    // simulated robotic hand
    // - create pointer to robot (with odeHandle, osgHandle and configuration)
    // - place robot
    AshigaruConf conf = Ashigaru::getDefaultConf();

    robot = new Ashigaru(odeHandle, osgHandle, conf, "Ashigaru");
    robot->setColor(Color(1.0,0.5,1.0));
    robot->place(Pos(0,0,0.01));
    global.configs.push_back ( robot );

//    // fix hand (in actual position) to simulation environment
//    Primitive* trunk = hand->getMainPrimitive();
//    fixator = new FixedJoint(trunk, global.environment);
//    fixator->init(odeHandle, osgHandle);


//#ifdef SINEController
    //SineController (produces just sine waves)
    // create pointer to controller
    // set some parameters
    controller = new SineController();
    controller->setParam("amplitude", 1);
    controller->setParam("period", 500);
    controller->setParam("phaseshift", 0.5);

//#else
//    // Selforg - Controller
//    // create pointer to controller
//    // set some parameters
//    controller = new InvertMotorSpace(10);
//    controller->setParam("epsA",0.3); // model learning rate
//    controller->setParam("epsC",0.3); // controller learning rate
//    controller->setParam("rootE",3);    // model and contoller learn with square rooted error
//#endif
    // push controller in global list of configurables
    global.configs.push_back ( controller );

    // create pointer to one2onewiring which uses colored-noise
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );

    // create pointer to agent (plotoptions is provided by Simulation (generated from cmdline options)
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent (global );
    agent->init ( controller , robot , wiring );
    global.agents.push_back ( agent );

    // display all parameters of all configurable objects on the console

  }

//  /** is called if a key was pressed.
//      For keycodes see: osgGA::GUIEventAdapter
//      @return true if the key was handled
//  */
//  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData,
//                       int key, bool down) {
//    if (down) { // only when key is pressed, not when released
//      switch ( (char) key ) {
//        case 'x':
//          if(fixator) delete fixator;
//          fixator=0;
//          break;
//        case 'c' :{
//          passive_capsule =  new PassiveCapsule(odeHandle, osgHandle, 1,1,5);
//                passive_capsule->setColor(Color(1.0f,0.2f,0.2f,1.0f));
//                passive_capsule->setTexture("Images/furry_toy.jpg");
//                passive_capsule->setPosition(Pos(0,0,5));
//                globalData.obstacles.push_back(passive_capsule); }
//          break;
//      default:
//        return false;
//      }
//      return true;
//    } else return false;
//  }
//
//  virtual void bindingDescription(osg::ApplicationUsage & au) const {
//    au.addKeyboardMouseBinding("Simulation: x","Delete fixation joint (hand will fall down)");
//    au.addKeyboardMouseBinding("Simulation: c","Create Capsule in hand");
//  }

};

int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}


