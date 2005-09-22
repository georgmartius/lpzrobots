#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include "noisegenerator.h"
#include "agent.h"
#include "one2onewiring.h"
#include "playground.h"

#include "invertnchannelcontroller.h"
#include "noisegenerator.h"

#include "schlange.h"

ConfigList configs;
PlotMode plotMode = NoPlot;

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
    
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(7.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);
    
  //****************
  SchlangenConf conf = Schlange::getStandartConf();  
  Schlange* schlange1 = new Schlange ( 1 , odeHandle, conf);
  Color col(0,0.5,0.8);
  schlange1->place(Position(0,0,0),&col);
  AbstractController *controller = new InvertNChannelController(10);  
  
  One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());
  Agent* agent = new Agent(plotMode);
  agent->init(controller, schlange1, wiring);
  global.agents.push_back(agent);
  configs.push_back(controller);
  
  /*
  Schlange* schlange2 = new Schlange ( 2 , world , space , contactgroup,  
				       0 , 0 , 0.25 , 4, 0.5 , 0.2 , 0 , 0.1 , 2 , 10 , angle );
  Position p2 = {0,2,0};
  Color col2 = {0.5,0,0.5};
  schlange2->place(p2,&col2);
  AbstractController *controller2 = new InvertNChannelController(10);  
  
  One2OneWiring* wiring2 = new One2OneWiring();
  Agent* agent2 = new Agent(new ColorUniformNoise(),NoPlot);
  agent2->init(controller2, schlange2, wiring2);
  global.agents.push_back(agent2);
  configs.push_back(controller2);
  */
    
  showParams(configs);
}

void end(GlobalData& global){
  for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++){
    delete (*i);
  }
  global.obstacles.clear();
  for(AgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++){
    delete (*i)->getRobot();
    delete (*i)->getController();
    delete (*i);
  }
  global.agents.clear();
}


// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){
  changeParams(configs);
}

void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile", progname);
  exit(0);
}

int main (int argc, char **argv)
{  
  if(contains(argv, argc, "-g")) plotMode = GuiLogger;
  if(contains(argv, argc, "-l")) plotMode = GuiLogger_File;
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}

 
