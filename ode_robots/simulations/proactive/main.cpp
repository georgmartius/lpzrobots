#include <stdio.h>

#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include <selforg/onelayerffnn.h>
#include <selforg/proactive.h>
#include <selforg/invertmotornstep.h>

#include "odeagent.h"
#include "simulation.h"
#include "nimm2.h"
#include "playground.h"

using namespace lpzrobots;

list<PlotOption> plotoptions;

class ProactiveSim : public Simulation {
public:

  //Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) {

    //   //Anfangskameraposition und Punkt auf den die Kamera blickt
    //   float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
    //   float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
    //   dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
    //   dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

    // initialization
    global.odeConfig.noise=0.1;
  
    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(6, 0.2, 0.5));
    playground->setPosition(Pos(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);


    Nimm2Conf nimm2Conf = Nimm2::getDefaultConf();
    nimm2Conf.irFront=true;
    nimm2Conf.singleMotor=true;
    //  nimm2Conf.force=nimm2Conf.force*3;  
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, nimm2Conf);
    vehicle->place(Pos(0,0,0));

  
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.buffersize=50;
    //cc.useS=true;
    cc.cInit=1;
    AbstractController *controller = new ProActive(1, 20, cc);  
    // AbstractController *controller = new InvertMotorNStep(cc);
    //   controller->setParam("steps",2);
    //  controller->setParam("epsH",0.001);
    controller->setParam("nomupdate",0.001);
    //controller->setParam("adaptrate",0);
    //  controller->setParam("epsC",0.5);
    //  controller->setParam("epsA",0.1);
    //  global.odeConfig.setParam("realtimefactor",3);
  
    DerivativeWiringConf wconf = DerivativeWiring::getDefaultConf();
    //wconf.useSecondD = true;
    wconf.eps = 0.1;
    AbstractWiring* wiring = new DerivativeWiring(wconf, new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(plotoptions);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);

    global.configs.push_back(controller);
    showParams(global.configs);
  }

  virtual void end(GlobalData& global){
    for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++){
      delete (*i);
    }
    global.obstacles.clear();
    for(OdeAgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++){
      delete (*i)->getRobot();
      delete (*i)->getController();
      delete (*i);
    }
    global.agents.clear();
  }

};
  
void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile", progname);
}  
  
int main (int argc, char **argv)
{  
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv, argc, "-l")) plotoptions.push_back(PlotOption(GuiLogger_File));
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  ProactiveSim sim;
  sim.run(argc, argv);
  return 0;
}
 
