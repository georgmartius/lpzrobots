// Simulation
#include <ode_robots/simulation.h>
// Noise generator
#include <selforg/noisegenerator.h>
// Agent: bind robot, controller and wiring together
#include <ode_robots/odeagent.h>
// The robot
#include "differential.h"
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
    BasicController* controller;
    Differential* robot;

    /// start() is called at the start and should create all the object (obstacles, agents...).
    virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
      // Initial position and orientation of the camera
      setCameraHomePos(Pos(8., 8., 8.), Pos(.0, .0, .0));
      // Some simulation parameters can be set here
      global.odeConfig.setParam("controlinterval", 1);
      global.odeConfig.setParam("gravity", -9.8);
      global.odeConfig.setParam("realtimefactor", 1);

      /** New robot instance */
      // Get the default configuration of the robot
      DifferentialConf conf = Differential::getDefaultConf();
      // Values can be modified locally
      conf.wheelMass = .5;
      // Instantiating the robot
      robot = new Differential(odeHandle, osgHandle, conf, "Differential robot");
      // Placing the robot in the scene
      ((OdeRobot*)robot)->place(Pos(.0, .0, .2));
      // Instantiatign the controller
      controller = new BasicController("Basic Controller", "$ID$");
      // Create the wiring with color noise
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(.1));
      // Create Agent
      OdeAgent* agent = new OdeAgent(global);
      // Agent initialisation
      agent->init(controller, robot, wiring);
      // Adding the agent to the agents list
      global.agents.push_back(agent);
      //global.configs.push_back(controller);
      global.configs.push_back(robot);

      /** Environment and obstacles */
      // New playground
      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(15., .2, 1.2), 1);
      // Set colours
      playground->setGroundColor(Color(.784, .784, .0));
      playground->setColor(Color(1., .784, .082, .1));
      // Set position
      playground->setPosition(osg::Vec3(.0, .0, .1));
      // Adding playground to obstacles list
      global.obstacles.push_back(playground);
     
      // Add a new box obstacle
      //PassiveBox* box = new PassiveBox(odeHandle, osgHandle, osg::Vec3(1., 1., 1.), 2.);
      //box->setPose(osg::Matrix::translate(-.5, 4., .7));
      //global.obstacles.push_back(box);    
    }

    /* Functions not used in this tutorial but needed by interface */
    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    }

    virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
      return false;
    }

    virtual void bindingDescription(osg::ApplicationUsage & au) const {
    }
    
    ThisSim() {
    }

    ~ThisSim() {
    }

};

int main (int argc, char **argv)
{
  // New simulation
  ThisSim sim;
  // Caption set
  sim.setCaption("BASIC SIM");
  // Simulation begins
  return sim.run(argc, argv) ? 0 : 1;
}




