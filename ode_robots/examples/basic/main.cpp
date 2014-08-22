/***************************************************************************
 *   Copyright (C) 2005-2013 LpzRobots development team                    *
 *   Authors:                                                              *
 *    Simón Smith <artificialsimon at ed dot ac dot uk>                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
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

// Simulation
#include <ode_robots/simulation.h>
// Noise generator
#include <selforg/noisegenerator.h>
// Agent: bind robot, controller and wiring together
#include <ode_robots/odeagent.h>
// The robot
#include "differential.h"
// Speed sensor
#include <ode_robots/speedsensor.h>

// Robot's controller
#include "basiccontroller.h"
// Robot's wiring
#include <selforg/one2onewiring.h>
// Environmnet and obstacles
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>


using namespace lpzrobots;

class ThisSim : public Simulation
{
  public:
    ThisSim() {
      // set Title of simulation
      setTitle("BASIC SIM by Simon");
    }
    ~ThisSim() { }

    /// start() is called at the start and should create all the object (obstacles, agents...).
    virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
      // Initial position and orientation of the camera (use 'p' in graphical window to find out)
      setCameraHomePos(Pos(-14,14, 10),  Pos(-135, -24, 0));
      // Some simulation parameters can be set here
      global.odeConfig.setParam("controlinterval", 1);
      global.odeConfig.setParam("gravity", -9.8);

      /** New robot instance */
      // Get the default configuration of the robot
      DifferentialConf conf = Differential::getDefaultConf();
      // Values can be modified locally
      conf.wheelMass = .5;
      // Instantiating the robot
      auto robot = new Differential(odeHandle, osgHandle, conf, "Differential robot");
      // add a speed sensor to the robot (attached to the "main primitive" (-1)
      //  (specifiy index if needed)
      robot->addSensor(std::make_shared<SpeedSensor>(1), Attachment(-1));

      // Placing the robot in the scene
      robot->place(Pos(.0, .0, .2));

      // Instantiatign the controller
      auto controller = new BasicController("Basic Controller");
      // Create the wiring with color noise
      auto wiring = new One2OneWiring(new ColorUniformNoise(.1));
      // Create Agent
      auto agent = new OdeAgent(global);
      // Agent initialisation
      agent->init(controller, robot, wiring);
      // Adding the agent to the agents list
      global.agents.push_back(agent);
      global.configs.push_back(agent);

      /** Environment and obstacles */
      // New playground
      auto playground = new Playground(odeHandle, osgHandle,osg::Vec3(15., .2, 1.2), 1);
      // Set colours
      playground->setGroundColor(Color(.784, .784, .0));
      playground->setColor(Color(1., .784, .082, .3));
      // Set position
      playground->setPosition(osg::Vec3(.0, .0, .1));
      // Adding playground to obstacles list
      global.obstacles.push_back(playground);

      // Add a new box obstacle (or use 'o' to drop random obstacles)
      //auto box = new PassiveBox(odeHandle, osgHandle, osg::Vec3(1., 1., 1.), 2.);
      //box->setPose(osg::Matrix::translate(-.5, 4., .7));
      //global.obstacles.push_back(box);
    }

    /* Functions not used in this tutorial but typically useful */
    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    }

    virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
      return false;
    }

    virtual void bindingDescription(osg::ApplicationUsage & au) const {
    }
};

int main (int argc, char **argv)
{
  // New simulation
  ThisSim sim;
  // Simulation begins
  return sim.run(argc, argv) ? 0 : 1;
}
