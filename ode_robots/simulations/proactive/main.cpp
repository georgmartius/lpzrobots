#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

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

list<PlotOption> plotoptions;

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.noise=0.1;
  int chessTexture = dsRegisterTexture("chess.ppm");
  printf("Chess: %i", chessTexture);
  
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(6, 0.2, 0.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);


  Nimm2Conf nimm2Conf = Nimm2::getDefaultConf();
  nimm2Conf.irFront=true;
  nimm2Conf.singleMotor=true;
  //  nimm2Conf.force=nimm2Conf.force*3;  
  Nimm2* vehicle = new Nimm2(odeHandle, nimm2Conf);
  vehicle->setTextures(DS_WOOD, chessTexture); 
  vehicle->place(Position(0,0,0));

  
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

void end(GlobalData& global){
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


// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){
  changeParams(global.configs);
}

void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile", progname);
}

int main (int argc, char **argv)
{  
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv, argc, "-l")) plotoptions.push_back(PlotOption(GuiLogger_File));
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
