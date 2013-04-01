#include <selforg/noisegenerator.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>
#include "basiccontroller.h"
#include "differential.h"

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
        setCameraHomePos(Pos(5.0, 5.0, 5.0), Pos(.0, .0, .0));
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
        ((OdeRobot*)robot)->place(Pos(.0, .0, 10.0));
 
        // Instantiatign the controller
        controller = new BasicController("Basic Controller", "$ID$");

       // Create the wirin
        AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(.1));

        // Create Agent
        OdeAgent* agent = new OdeAgent(global);
        agent->init(controller, robot, wiring);
        global.agents.push_back(agent);
        //global.configs.push_back(controller);
        //global.configs.push_back(robot);
        //global.configs.push_back(this);

    }


    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    }


    virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
      return true;
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




