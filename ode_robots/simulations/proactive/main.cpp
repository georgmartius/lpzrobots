#include <stdio.h>

#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include <selforg/onelayerffnn.h>
#include <selforg/proactive2.h>
#include <selforg/invertmotornstep.h>
// #include <selforg/sinecontroller.h>

#include "odeagent.h"
#include "simulation.h"
#include "nimm2.h"
#include "playground.h"

using namespace lpzrobots;

Playground* playground;
Pos plpos;

class ProactiveSim : public Simulation {
public:

  //Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));

    // initialization
    global.odeConfig.noise=0.05;
  
    playground = new Playground(odeHandle, osgHandle, osg::Vec3(8, 0.2, 0.5));
    playground->setPosition(plpos); // playground positionieren und generieren
    global.obstacles.push_back(playground);


    Nimm2Conf nimm2Conf = Nimm2::getDefaultConf();
    nimm2Conf.irFront=true;
    nimm2Conf.irRange=2;
    nimm2Conf.singleMotor=true;
    nimm2Conf.force = 5;
    //  nimm2Conf.force=nimm2Conf.force*3;  
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, nimm2Conf);
    vehicle->place(Pos(0,0,0.2));

  
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.buffersize=50;
    // cc.useS=true;
    cc.cInit=1.1;
    cc.someInternalParams=false;
    AbstractController *controller = new ProActive2(1,30, cc);  
    // AbstractController *controller = new SineController();  
    // AbstractController *controller = new InvertMotorNStep(cc);
    //   controller->setParam("steps",2);
    controller->setParam("epsH",100);
    // controller->setParam("nomupdate",0.001);
    controller->setParam("adaptrate",0);
    controller->setParam("epsC",0.05);
    controller->setParam("epsA",0.05);
    controller->setParam("eps",0.1); // eps for delta H net 
    controller->setParam("s4avg",1); 
    //  global.odeConfig.setParam("realtimefactor",3);
  
    DerivativeWiringConf wconf = DerivativeWiring::getDefaultConf();
    //    wconf.useFirstD = true;
    //    wconf.useSecondD = false;
    //wconf.useSecondD = true;
    wconf.derivativeScale=1;
    wconf.eps = 0.4;
    AbstractWiring* wiring = new DerivativeWiring(wconf, new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(plotoptions);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);

    global.configs.push_back(controller);
    showParams(global.configs);
  }

  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;    
    bool handled = false;
    switch ( key )
      {
      case 'i' : 
	plpos = plpos + Pos(0.2,0,0);
	playground->setPosition(plpos);
	handled = true; 
	break;
      case 'k' : 
	plpos = plpos - Pos(0.2,0,0);
	playground->setPosition(plpos);
	handled = true; 
	break;	
      case 'j' : 
	plpos = plpos + Pos(0,0.2,0);
	playground->setPosition(plpos);
	handled = true; 
	break;
      case 'l' : 
	plpos = plpos - Pos(0,0.2,0);
	playground->setPosition(plpos);
	handled = true; 
	break;
      }
  
    fflush(stdout);
    return handled;
  }


  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Playground: i","shift forward");
    au.addKeyboardMouseBinding("Playground: k","shift backward");
    au.addKeyboardMouseBinding("Playground: j","shift left");
    au.addKeyboardMouseBinding("Playground: l","shift right");
    au.addKeyboardMouseBinding("Simulation: s","store");
    au.addKeyboardMouseBinding("Simulation: l","load");
  }

};
  
  
int main (int argc, char **argv)
{  
  ProactiveSim sim;
  return sim.run(argc, argv) ? 0 : 1; 
}
 
