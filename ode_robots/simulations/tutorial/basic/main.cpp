#include <selforg/noisegenerator.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>
#include "basiccontroller.h"
#include "differential.h"

// ODE obstacles
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>

using namespace lpzrobots;

class ThisSim : public Simulation
{
  public:
    BasicController* controller;
    Differential* robot;

    ThisSim() {
    }

    ~ThisSim() {
    }

    /// start() is called at the start and should create all the object (obstacles, agents...).
    virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
      //setCameraHomePos(Pos(5.0, 5.0, 5.0), Pos(.0, .0, .0));
      setCameraHomePos(Pos(2.0, 2.0, 2.0), Pos(.0, .0, .0));
      //global.odeConfig.setParam("controlinterval",1);
      global.odeConfig.setParam("gravity", -9.8);
      //global.odeConfig.setParam("realtimefactor",0);

      // Get the default configuration of the robot
      DifferentialConf conf = Differential::getDefaultConf();
      // Values can be modified locally
      conf.wheelMass = .5;

      // Instantiating the robot
      robot = new Differential(odeHandle, osgHandle, conf, "Differential robot");
      // Placing the robot in the scene
      //((OdeRobot*)robot)->place(Pos(.0, .0, .0));
      ((OdeRobot*)robot)->place(Pos(.0, .0, .2));

      // Instantiatign the controller
      controller = new BasicController("Basic Controller", "$ID$");

      // Create the wiring
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(.1));

      // Create Agent
      OdeAgent* agent = new OdeAgent(global);
      agent->init(controller, robot, wiring);
      global.agents.push_back(agent);
      //global.configs.push_back(controller);
      global.configs.push_back(robot);
     //global.configs.push_back(this);1
     //
     //
     //
      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(15, 0.2, 1.2 ), 1);
      playground->setGroundColor(Color(200/255.0,200/255.0,0/255.0));
      //playground->setGroundTexture("Images/really_white.rgb");    
      //playground->setGroundTexture("");    
      playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.1));
      playground->setPosition(osg::Vec3(0,0,0.1));
      //playground->setTexture("");
      global.obstacles.push_back(playground);
      //     // inner playground
      //playground = new Playground(odeHandle, osgHandle,osg::Vec3(10, 0.2, 1.2), 1, false);
      //playground->setColor(Color(255/255.0,200/255.0,0/255.0, 0.1));
      //playground->setPosition(osg::Vec3(0,0,0.1));
      //playground->setTexture("");
      global.obstacles.push_back(playground);
      
      
      
      //PassiveBox* box = new PassiveBox(odeHandle, osgHandle,
      //osg::Vec3(2.0, 2.0, 2.0), 2.0);
      //box->setPose(osg::Matrix::translate(-5.0, 3.0, 1.0));
      //global.obstacles.push_back(box);    

    }


    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    }


    virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
      //if (!down) return false;
      return false;
    }

    virtual void bindingDescription(osg::ApplicationUsage & au) const {
    }

};




int main (int argc, char **argv)
{
  ThisSim sim;
  sim.setCaption("BASIC SIM");
  return sim.run(argc, argv) ? 0 : 1;
}




