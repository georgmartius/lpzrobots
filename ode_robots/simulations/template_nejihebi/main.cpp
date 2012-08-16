/***************************************************************************
 *   Copyright (C) 2012 by1                                                *
 *    Timo Nachstedt <nachstedt@physik3.gwdg.de>                           *
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
// simple wiring
#include <selforg/one2onewiring.h>
// the robot
#include <ode_robots/nejihebi.h>
// we need to increase the stack size
#include <sys/resource.h>

/**
 * Simple Example Controller
 *
 * This is just a simple controller to control the robot
 */
class ExampleController : public AbstractController {

  public:
    ExampleController():AbstractController( "ExampleController",
        "$Id: main.cpp,v 0.1 $") {};

    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0) {
      number_channels = motornumber;
    }

    virtual ~ExampleController() {};

    /**
     * returns the name of the object (with version number)
     */
    virtual paramkey getName() const {
      return "Nejihebi Example Controller";
    }

    /**
     * returns the number of sensors the controller was initialised with or 0
     * if not initialised
     */
    virtual int getSensorNumber() const {
      return number_channels;
    }

    /**
     * returns the mumber of motors the controller was initialised with or 0 if
     * not initialised
     */
    virtual int getMotorNumber() const {
      return number_channels;
    }

    /**
     * performs one step (includes learning).
     * Calulates motor commands from sensor inputs.
     */
    virtual void step(const sensor* x, int number_sensors, motor* y,
        int number_motors) {
      stepNoLearning(x, number_sensors, y, number_motors);
    }

    /**
     * performs one step without learning. Calculates motor commands from
     * sensor inputs.
     */
    virtual void stepNoLearning(const sensor* x, int number_sensors,
        motor* y, int number_motors) {
      // screw speeds
      for (int i=0; i<4; i++) y[i] = 0.2*(-1+2*(i%2));
      // servo goal positions
      y[4]  = 0.0;
      y[5]  = 0.0;
      y[6]  = 0.0;
      y[7]  = 0.0;
      y[8]  = 0.0;
      y[9]  = 0.0;
      // servo torque limits
      y[10] = 1.0;
      y[11] = 1.0;
      y[12] = 1.0;
      y[13] = 1.0;
      y[14] = 1.0;
      y[15] = 1.0;
    }

    /***** STOREABLE ****/
    /** stores the controller values to a given file. Not implemented. */
    virtual bool store(FILE* f) const { return false; }
    /** loads the controller values from a given file. Not implemented. */
    virtual bool restore(FILE* f) {return false; };

  protected:
    unsigned short number_channels;

  public:
};


class ThisSim : public lpzrobots::Simulation {
  public:

    ThisSim(){
      // nothing to do here
    }

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
      global.odeConfig.setParam("simstepsize", 0.05);


      // Add snake-like robot using screw drive mechanism
      snake = new lpzrobots::Nejihebi(
          odeHandle,
          osgHandle.changeColor(lpzrobots::Color(1, 1, 1)));

      // put snake a little bit in the air
      snake->place(osg::Matrix::translate(.0, .0, 2.0));

      controller = new ExampleController();

      // create wiring
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

      // create agent and init it with controller, robot and wiring
      lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
      agent->init(controller, snake, wiring);

      // inform global variables over everything that happened:
      global.configs.push_back(snake);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
    }

  private:
    lpzrobots::Joint* robotfixator;
    AbstractController* controller;
    lpzrobots::Nejihebi* snake;
};

int main(int argc, char **argv) {

  // We have to increase the stack size as there are really a lot of collisions
  // to handle for this robot
  const rlim_t kStackSize = 32 * 1024 * 1024;   // min stack size = 32 MB
  struct rlimit rl;
  int result = getrlimit(RLIMIT_STACK, &rl);
  if (result == 0) {
      if (rl.rlim_cur < kStackSize) {
          rl.rlim_cur = kStackSize;
          result = setrlimit(RLIMIT_STACK, &rl);
          if (result != 0)
            std::cerr << "setrlimit returned result = " << result << "\n";
      }
  } else {
    std::cerr << "getrlimit returned result = " << result << "\n";
  }

  ThisSim sim;
  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}

