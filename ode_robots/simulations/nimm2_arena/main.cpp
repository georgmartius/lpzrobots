#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"
#include "simulation.h"
#include "agent.h"
#include "one2onewiring.h"
#include "nimm2.h"
#include "playground.h"
#include "octaplayground.h"

#include "invertnchannelcontroller.h"
#include "invertmotornstep.h"

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
  float KameraXYZ[3]= {0.25f,-6.0f,3.79f};
  float KameraViewXYZ[3] = {90.00f,-43.50f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  simulationConfig.noise=0.1;
  configs.push_back(&simulationConfig);
  
  Playground* playground = new Playground(world, space, /*factorxy=1*/ -4);
  playground->setGeometry(8.0, 0.09, 0.25);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  obstacles.push_back(playground);
  

//   OctaPlayground* octaplayground = new OctaPlayground(world, space, 330);
//   octaplayground->setGeometry(6.5, 0.09, 0.25);
//   octaplayground->setPosition(0,0,0); // playground positionieren und generieren
//   obstacles.push_back(octaplayground);

  Nimm2* nimm2;
  AbstractController* controller;
  AbstractWiring* wiring;
  Agent* agent;

  int chessTexture = dsRegisterTexture("chess.ppm");

    for (int j=-1; j<2; j++){ 
      for (int i=-4; i<5; i++){
      //      nimm2 = new Nimm2(world, space, contactgroup);
      nimm2 = new Nimm2(world, space, contactgroup,
			/*size=1.0*/  1.0, 
			/*force=0.1*/ 0.5,
			/*speed=10*/ 20,
			/*drawWheels=true*/ true,
			/*bumper = false*/ true,
			/*cigarMode = false*/ true);
      nimm2->setTextures(DS_WOOD, chessTexture); 
      if ((i==0) && (j==0)) {nimm2->place(Position(j*0.26,i*0.26,0), &Color(2,2,0));}
      else {
	nimm2->place(Position(j*2.5,i*1.26,0));
      }
      //controller = new InvertMotorNStep(10);  
      controller = new InvertNChannelController(10);  

      configs.push_back(controller);
      
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      if ((i==0) && (j==0)) {agent = new Agent(plotMode);}
      else {
	agent = new Agent(NoPlot);
      }
      agent->init(controller, nimm2, wiring);
      agents.push_back(agent);

      controller->setParam("factorB",0);
    }
  }

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
 
