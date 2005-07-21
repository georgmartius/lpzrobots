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

#include "hurlingsnake.h"

#include "invertnchannelcontroller.h"
#include "invertmotorspace.h"

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

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {-0.34f,-11.59f,6.33f};
  float KameraViewXYZ[3] = {90.0f,-38.50f,0.00f};
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  simulationConfig.noise=0.3;
  simulationConfig.setParam("controlinterval",1);

  
   Playground* playground = new Playground(world, space);
   playground->setGeometry(20.0, 0.2, 2.5);
   playground->setPosition(0,0,0); // playground positionieren und generieren
   obstacles.push_back(playground);

//   Nimm2* vehicle = new Nimm2(world, space, contactgroup);
//   Position p = {3,3,0};
//   vehicle->place(p);
//   AbstractController *controller = new InvertNChannelController(10);  
  
//   One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
//   Agent* agent = new Agent(NoPlot/*plotMode*/);
//   agent->init(controller, vehicle, wiring);
//   agents.push_back(agent);
//   configs.push_back(controller);


  configs.push_back(&simulationConfig);




  HurlingSnake* hs;
  AbstractController *controller; 
  AbstractWiring* wiring;
  Agent* agent;

  for (int i=0; i<3; i++){
    hs = new HurlingSnake(world, space, contactgroup);
    if (i==0) hs->place(mkPosition(i-1,-7,0.3), &mkColor(2,2,0));
    if (i==1) hs->place(mkPosition(i-1,-7,0.3), &mkColor(0,2,0));
    if (i==2) hs->place(mkPosition(i-1,-7,0.3), &mkColor(0,2,2));
    controller = new InvertMotorSpace(10);  
    //    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
    c.blindMotorSets=0;
    c.useId = true;
    c.useFirstD = true;
    c.derivativeScale = 50;
    wiring = new DerivativeWiring(c, new ColorUniformNoise(0.1));
    if (i==0) agent = new Agent(plotMode);
    else  agent = new Agent(NoPlot);
    agent->init(controller, hs, wiring);
    agents.push_back(agent);
    
    hs->setParam("factorForce",5);
    hs->setParam("frictionGround",0.3);
    controller->setParam("epsA", 0.15);
    controller->setParam("epsC", 0.04);
    
    configs.push_back(controller);
    configs.push_back(hs);
    

  }
  
//   ///////////////////////
//   hs = new HurlingSnake(world, space, contactgroup);
//   Position p4 = {1,1,0.3};
//   hs->place(p4);
//   //AbstractController *controller2 = new InvertNChannelController(10);  
//   controller2 = new InvertMotorSpace(10);  
  
//   wiring2 = new One2OneWiring(new ColorUniformNoise(0.1));
//   agent2 = new Agent(plotMode);
//   agent2->init(controller2, hs, wiring2);
//   agents.push_back(agent2);

//   configs.push_back(controller2);
//   configs.push_back(hs);
//   controller2->setParam("rho", 0.1);



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
    delete (*i)->getWiring();
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
 
