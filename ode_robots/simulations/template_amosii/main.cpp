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
 *   $Log: main.cpp,v $                                                    *
 *                                                                         *
 ***************************************************************************/

// include simulation environment stuff
#include <ode_robots/simulation.h>
// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
// simple wiring
#include <selforg/one2onewiring.h>
// the robot
#include "ode_robots/amosII.h"
// the controller
#include "mycontroller.h"

class ThisSim : public lpzrobots::Simulation {

  public:
    /**
     * starting function (executed once at the beginning of the simulation loop)
     */
    virtual void start(const lpzrobots::OdeHandle& odeHandle,
        const lpzrobots::OsgHandle& osgHandle,
        lpzrobots::GlobalData& global) {
      // set initial camera position
      setCameraHomePos(
          lpzrobots::Pos(-0.0114359, 6.66848, 0.922832),
          lpzrobots::Pos(178.866, -7.43884, 0));

      // set simulation parameters
      global.odeConfig.setParam("controlinterval", 10);
      global.odeConfig.setParam("simstepsize", 0.01);

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
      agent->init(controller, amos, wiring);

      // create a fixed joint to hold the robot in the air at the beginning
      robotfixator = new lpzrobots::FixedJoint(
          amos->getMainPrimitive(),
          global.environment);
      robotfixator->init(odeHandle, osgHandle, false);

      // inform global variable over everything that happened:
      global.configs.push_back(amos);
      global.agents.push_back(agent);
      global.configs.push_back(controller);

      std::cout << "\n\n"
          << "################################\n"
          << "#   Press x to free amosII!    #\n"
          << "################################\n"
          << "\n\n" << std::endl;
    }

    /**
     * add own key handling stuff here, just insert some case values
     */
    virtual bool command(const lpzrobots::OdeHandle&,
        const lpzrobots::OsgHandle&,
        lpzrobots::GlobalData& globalData,
        int key,
        bool down)
        {
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
  protected:
    lpzrobots::Joint* robotfixator;
    AbstractController* controller;
    lpzrobots::AmosII* amos;
};

int main(int argc, char **argv)
    {
  ThisSim sim;
  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}

