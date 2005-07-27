#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"
#include "simulation.h"
#include "agent.h"
#include "one2onewiring.h"
#include "derivativewiring.h"

#include "nimm2.h"
#include "playground.h"

#include "invertmotorspace.h"
#include "invertmotornstep.h"
#include "invertmotornstepwiths.h"
#include "sinecontroller.h"

ConfigList configs;
PlotMode plotMode = NoPlot;

// Funktion die die Steuerung des Roboters uebernimmt
bool StepRobot()
{
  for(AgentList::iterator i=agents.begin(); i != agents.end(); i++){
    (*i)->step(simulationConfig.noise);
  }
  return true;
}

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start() 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //  dWorldSetGravity ( world , 0 , 0 ,-9.81 );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  simulationConfig.noise=0.1;

  Playground* playground = new Playground(world, space);
  playground->setGeometry(20.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren 
  obstacles.push_back(playground);

  AbstractRobot* vehicle = new Nimm2(world, space, contactgroup,1.5,1.5,6);

  vehicle->place(Position(0,0,0));
  AbstractController *controller = new InvertMotorNStep(10);   
  // AbstractController *controller = new InvertMotorNStep(10);  
  // AbstractController *controller = new InvertMotorSpace(10);  
  //  controller->setParam("factorB",0);
  //  controller->setParam("eps",0.5);
  // AbstractController *controller = new SineController();  
  //  controller->setParam("phaseShift",1);
  DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
  c.useId=true;
  c.useFirstD=false;
  c.derivativeScale=10;
  c.eps=0.1;
  // c.blindMotorSets=1;
  AbstractWiring* wiring = new DerivativeWiring(c, new ColorUniformNoise(0.1));
  //AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  Agent* agent = new Agent(plotMode);
  agent->init(controller, vehicle, wiring);
  agents.push_back(agent);
  
  configs.push_back(&simulationConfig);
  configs.push_back(controller);
  showParams(configs);

}

void end(){
  for(ObstacleList::iterator i=obstacles.begin(); i != obstacles.end(); i++){
    delete (*i);
  }
  obstacles.clear();
  for(AgentList::iterator i=agents.begin(); i != agents.end(); i++){
    delete (*i)->getRobot();
    delete (*i)->getController();
    delete (*i);
  }
  agents.clear();
}


// this function is called if the user pressed Ctrl-C
void config(){
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
 
