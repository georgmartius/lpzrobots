/*
 * thissim.cpp
 *
 *  Created on: Feb 21, 2012
 *      Author: timo
 */

#include "thissim.h"

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>

#include <selforg/one2onewiring.h>

#include "ode_robots/amosII.h"
#include "mycontroller.h"

ThisSim::ThisSim() {
  setGroundTexture("textures/tiles-512x512.png");
}

// starting function (executed once at the beginning of the simulation loop)
void ThisSim::start(const lpzrobots::OdeHandle& odeHandle,
    const lpzrobots::OsgHandle& osgHandle, lpzrobots::GlobalData& global) {

  // set initial camera position
  setCameraHomePos(
      lpzrobots::Pos(-0.0114359, 6.66848, 0.922832),
      lpzrobots::Pos(178.866, -7.43884, 0));

  // set simulation parameters
  global.odeConfig.setParam("controlinterval", 10); //was 10
  global.odeConfig.setParam("simstepsize", 0.01); //was 0.01 (martin)

  // Add amosII robot
  lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf();
  lpzrobots::OdeHandle rodeHandle = odeHandle;
  rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);
  amos = new lpzrobots::AmosII(
      rodeHandle,
      osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
      myAmosIIConf, "AmosII");

  // define the usage of the individual legs
  amos->setLegPosUsage(amos->L0, amos->LEG);
  amos->setLegPosUsage(amos->L1, amos->LEG);
  amos->setLegPosUsage(amos->L2, amos->LEG);
  amos->setLegPosUsage(amos->R0, amos->LEG);
  amos->setLegPosUsage(amos->R1, amos->LEG);
  amos->setLegPosUsage(amos->R2, amos->LEG);

  // put amos a little bit in the air
  amos->place(osg::Matrix::translate(.0, .0, 1));

  // create a simple example controller
  controller = new MyController();

  // create wiring
  One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

  // create agent and init it with controller, robot and wiring
  lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
  agent->init(controller, robot, wiring);

  // create a fixed joint to hold the robot in the air at the beginning
  robotfixator = new lpzrobots::FixedJoint(
      robot->getMainPrimitive(),
      global.environment);
  robotfixator->init(odeHandle, osgHandle,false);

  // inform global variable over everything that happened:
  global.configs.push_back(robot);
  global.agents.push_back(agent);
  global.configs.push_back(controller);

  std::cout << "\n\n"
            << "################################\n"
            << "#   Press x to free amosII!    #\n"
            << "################################\n"
            << "\n\n" << std::endl;
}

// add own key handling stuff here, just insert some case values
bool ThisSim::command(const lpzrobots::OdeHandle&, const lpzrobots::OsgHandle&,
    lpzrobots::GlobalData& globalData, int key, bool down) {
  if (down) { // only when key is pressed, not when released
    switch (char(key)) {
      case 'x':
        std::cout << "dropping robot" << std::endl;
        delete robotfixator;
        break;
      default:
        return false;
        break;
    }
  }
  return false;
}
