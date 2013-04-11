/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
 *    mai00bvz@studserv.uni-leipzig.de                                     *
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
 *   DESCRIPTION:                                                          *
 *   template for simulations which are parallized. This may be useful     *
 *   for analysis which is using different simulation parameters for       *
 *   several simulations.                                                  *
 *   This simulation is indented to be used with multicore processors,     *
 *   e.g. touchwood (Uni Leipzig) with up to 16 processors.                *
 *                                                                         *
 *   This template is not much useful for clusters, where typically        *
 *   simulations are batch processed and therefore the simulation jobs are *
 *   distributed on the cluster nodes. But there could be a gain of        *
 *   performance if a single node contains more than one cpu and the       *
 *   batch processing algorithm does not distribute more than one job to   *
 *   one cluster node.                                                     *
 *   Remember that the use of parallel threads in one simulation should be *
 *   disabled to avoid thread locks. So the steps of ODE, OSG and          *
 *   controller are executed sequential (guaranteed by TaskedSimulation).  *
 *
 *   PLEASE read and study the other template simulations if you are not   *
 *   familiar with lpzrobots! The comments in this file are limited to     *
 *   the parallelism stuff.                                                *
 *
 *   $Log$
 *   Revision 1.4  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.3  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.2  2009/08/21 09:49:08  robot12
 *   (guettler) support for tasked simulations.
 *   - use the simulation template_taskedSimulations.
 *   - merged (not completely) from lpzrobots_tasked.
 *   - graphics is supported, but only for one simulation of a pool
 *
 *   Revision 1.1.2.1  2009/08/11 16:00:17  guettler
 *   - support for tasked simulations, does not yet work with graphics
 *   - in development state
 *
 *   Revision 1.2  2009/07/02 10:05:59  guettler
 *   added example erasing an agent after one cycle and creating new ones
 *                                                                         *
 *   Revision 1.1  2009/04/23 14:17:34  guettler
 *   new: simulation cycles, first simple implementation, use the
 *   additional method bool restart() for starting new cycles,
 *   template simulation can be found in template_cycledSimulation
 *   (originally taken from template_onerobot)
 *                                                                         *
 ***************************************************************************/
#include <stdio.h>

// include all necessary stuff
#include <ode-dbl/ode.h>
#include <selforg/noisegenerator.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/playground.h>
#include <selforg/invertmotorspace.h>

// include some needed files for parallel task handling
// class TaskedSimulation, holds the SimulationTaskHandle and additional info (like taskId)
#include <ode_robots/taskedsimulation.h>
// class SimulationTask encapsulates one simulation as a single task
#include <ode_robots/simulationtask.h>
// holds all data needed by handling the tasks, additionally there can be put more data.
#include <ode_robots/simulationtaskhandle.h>
// manages the handling of the tasks, including the parallel loop.
#include <ode_robots/simulationtasksupervisor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


// create your own SimulationTaskHandle
struct ThisSimulationTaskHandle : public SimulationTaskHandle
{
  // add needed data here for access from the simulation
  // example: position of robot in the simulation is stored in the list
  std::vector<Position> positionList;
};

/**
 * Just create your own simulation, it's up to you.
 *
 * It's essential that your simulation is deduced from
 * TaskedSimulation instead of Simulation.
 * With this little change you have access to the
 * taskId and the global simTaskHandle.
 */
class ThisSim : public TaskedSimulation {
public:

  OdeRobot* vehicle;
  OdeAgent* agent;

  /**
   * starting function (executed once at the beginning of the simulation loop/first cycle)
   * This function contains the additional parameters simTaskHandle and taskId, with these
   * you have access to your global data.
   */
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, SimulationTaskHandle& sTHandle, int taskId)
  {
    ThisSimulationTaskHandle* simTaskHandle = static_cast<ThisSimulationTaskHandle*> (&sTHandle);

    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    global.odeConfig.noise=0.05;
    // set realtimefactor to maximum
    global.odeConfig.setParam("realtimefactor", 0);

    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(32, 0.2, 0.5));
    playground->setPosition(osg::Vec3(0,0,0.05));
    global.obstacles.push_back(playground);

    Nimm2Conf c = Nimm2::getDefaultConf();
    vehicle = new Nimm2(odeHandle, osgHandle, c, "Nimm2");
    // place the vehicle in accordance with the positions stored in the simTaskHandle (if available)
    if ((int)(simTaskHandle->positionList.size()-1)<=taskId)
      vehicle->place(simTaskHandle->positionList[taskId]);
    else
      vehicle->place(Pos(0,0,0));

    AbstractController *controller = new InvertMotorSpace(10);
    global.configs.push_back(controller);

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);

    // disable print of params, because there are much simulations to run
    //
  }

  /**
   * restart() is called at the second and all following starts of the cylce
   * The end of a cycle is determined by (simulation_time_reached==true)
   * @param the odeHandle
   * @param the osgHandle
   * @param globalData
   * @return if the simulation should be restarted; this is false by default
   */
  virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, SimulationTaskHandle& sTHandle, int taskId)
  {
    //ThisSimulationTaskHandle* simTaskHandle = static_cast<ThisSimulationTaskHandle*> (&sTHandle);

    return false; // don't restart, just quit
    // see template_cycledSimulation for more info about usage
  }

  /** optional additional callback function which is called every simulation step.
      Called between physical simulation step and drawing.
      @param draw indicates that objects are drawn in this timestep
      @param pause always false (only called of simulation is running)
      @param control indicates that robots have been controlled this timestep
   */
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control, SimulationTaskHandle& sTHandle, int taskId)
  {
    //ThisSimulationTaskHandle* simTaskHandle = static_cast<ThisSimulationTaskHandle*> (&sTHandle);
    // for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
    // if simulation_time_reached is set to true, the simulation cycle is finished
    if (globalData.sim_step>=(60000/this->currentCycle))
    {
      simulation_time_reached=true;
    }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down, SimulationTaskHandle& sTHandle, int taskI)
  {
    //ThisSimulationTaskHandle* simTaskHandle = static_cast<ThisSimulationTaskHandle*> (&sTHandle);
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        default:
          return false;
          break;
        }
    }
    return false;
  }

};

/**
 * Defines a method to construct a ThisSim. This method is needed by the
 * SimulationTask, provided through the SimulationTaskSupervisor.
 * If you like to get the singleton instance of SimulationTaskSupervisor, you have
 * to pass as argument an instance of the ThisSimulationBuilder.
 */
class ThisSimCreator : public TaskedSimulationCreator
{
public:
  virtual TaskedSimulation* buildTaskedSimulationInstance()
  {
    return new ThisSim();
  }
};


int main (int argc, char **argv)
{
  // 1. create your own deduced SimulationTaskHandle
  ThisSimulationTaskHandle simTaskHandle;
  // 2. create your ThisSimCreator
  ThisSimCreator simCreator;
  // 2. get instance of a SimulationTaskSupervisor
  SimulationTaskSupervisor* simTaskSupervisor = SimulationTaskSupervisor::getInstance();
  // you can set the number of threads, by default it is the number of available cpu cores
  // (so you don't need to set)
  int index = ThisSim::contains(argv, argc, "-nthreads");
  if(index && argc > index)
      simTaskSupervisor->setNumberThreads(atoi(argv[index]));
  // set simTaskHandle and simCreator
  SimulationTaskSupervisor::setSimTaskHandle(simTaskHandle);
  SimulationTaskSupervisor::setTaskedSimCreator(simCreator);
  // 3. add needed data to your simTaskHandle
  simTaskHandle.positionList.push_back(Position(0,0,0));
  // 4. create one SimulationTask
  simTaskSupervisor->createSimTask();
  // Let's create some more SimulationTasks (and needed data)
  simTaskHandle.positionList.push_back(Position(3,0,0));
  simTaskHandle.positionList.push_back(Position(6,0,0));
  simTaskHandle.positionList.push_back(Position(0,3,0));
  simTaskHandle.positionList.push_back(Position(0,6,0));
  simTaskHandle.positionList.push_back(Position(0,0,6));
  simTaskSupervisor->createSimTasks(9);
  // HINT: Every SimulationTask (and therefore the associated TaskedSimulation)
  // gets a taskId at creation time of the SimulationTask:taskId= SimulationTaskHandle.simTaskList.size(),
  // thus in ascending order. Therewith your simulation can access your stored
  // data in the SimulationTaskHandle and get the proper ones.

  // And now come on, start the tasks
  simTaskSupervisor->setSimTaskNameSuffix("taskpool 0");
  simTaskSupervisor->runSimTasks(&argc,argv);

  // just add another task pool and run this ones
  simTaskSupervisor->createSimTasks(10);
  simTaskSupervisor->setSimTaskNameSuffix("taskpool 1");
  simTaskSupervisor->runSimTasks(&argc,argv);
}

