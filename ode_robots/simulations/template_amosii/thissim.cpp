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
void ThisSim::start(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle,
    lpzrobots::GlobalData& global) {
#ifdef VERBOSE
  std::cerr << "ThisSim::start called\n";
#endif
  /** set initial camera position */
  setCameraHomePos(lpzrobots::Pos(-0.0114359, 6.66848, 0.922832), lpzrobots::Pos(178.866, -7.43884, 0));

  /** set simulation parameters */
  global.odeConfig.setParam("controlinterval", 10); //was 10
  global.odeConfig.setParam("simstepsize", 0.01); //was 0.01 (martin)

  /** remember where to find the global data */
  global_data = &global;

  /**
   * Add A M O S II robot
   */
  lpzrobots::Substance RobotSubstance(3.0, 0.0, 50.0, 0.8);
  lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosIIConf::getDefaultConf();
  myAmosIIConf.texture = "textures/gray-texture-128x128.jpg";
  myAmosIIConf.useLocalVelSensor = true;
  lpzrobots::OdeHandle rodeHandle = odeHandle;
  rodeHandle.substance = RobotSubstance;

  //myAmosIIConf.coxaPower *= 0.01;

  lpzrobots::AmosII * amos = new lpzrobots::AmosII(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
      myAmosIIConf, "AmosII");

  /*
   amos->setLegPosUsage(amos->L0, amos->WHEEL);
   amos->setLegPosUsage(amos->L1, amos->LEG);
   amos->setLegPosUsage(amos->L2, amos->WHEEL);
   amos->setLegPosUsage(amos->R0, amos->WHEEL);
   amos->setLegPosUsage(amos->R1, amos->UNUSED);
   amos->setLegPosUsage(amos->R2, amos->WHEEL);
   */
  robot = amos;

  robot->place(osg::Matrix::rotate(0, 0, 0, 1) * osg::Matrix::translate(.0, .0, 0.01));

  /** add robot to configurables of simulation */
  global.configs.push_back(robot);

  controller = new MyController();

  // create wiring and agent
  One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.0));
  lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);

  // init agent with controller, robot and wiring
  agent->init(controller, robot, wiring);

  // Possibility to add tracking for robot
  bool track = 0;
  if (track)
    agent->setTrackOptions(TrackRobot(true, false, false, false, ""));

  //add agent to agents
  global.agents.push_back(agent);
  //add controller to configurables
  global.configs.push_back(controller);

  lpzrobots::Primitive* trunk = robot->getMainPrimitive();
  robotfixator = new lpzrobots::FixedJoint(trunk, global.environment);
  robotfixator->init(odeHandle, osgHandle);

  // @todo repair in new lpzrobots
  //lpzrobots::showParams(global.configs);

  myInspectables.push_back(amos);
  myInspectables.push_back(wiring);
  myInspectables.push_back(controller);
  myInspectables.push_back(agent);

  pause = true;

#ifdef VERBOSE
  std::cerr << "ThisSim::start finished\n";
#endif
}

// add own key handling stuff here, just insert some case values
bool ThisSim::command(const lpzrobots::OdeHandle&, const lpzrobots::OsgHandle&, lpzrobots::GlobalData& globalData,
    int key, bool down) {
  if (down) { // only when key is pressed, not when released

    lpzrobots::OdeHandle my_odeHandle = odeHandle;
    my_odeHandle.substance.toRubber(5);

    std::vector<double> f;
    switch ((char) key) {
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

// Called between physical simulation step and drawing in every timestep
void ThisSim::addCallback(lpzrobots::GlobalData& globalData, bool draw, bool pause, bool control) {
  lpzrobots::Simulation::addCallback(globalData, draw, pause, control);
  if (robot->getPosition().x > 1)
    simulation_time_reached = true;
}

bool ThisSim::restart(const lpzrobots::OdeHandle&, const lpzrobots::OsgHandle&, lpzrobots::GlobalData&) {
  std::cerr << "calling robot place" << std::endl;
  robot->place(lpzrobots::Pos(0, 0, 0));
  std::cerr << "returned from robot place" << std::endl;
  return true;
}

