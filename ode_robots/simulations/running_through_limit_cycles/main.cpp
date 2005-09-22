
/*
  gelaufen mit:
  invertnchannelcontroller.h   CVS-1.9
  invertnchannelcontroller.hpp CVS-1.10
  fixedsnake2elements.h        CVS-1.1
  fixedsnake2elements.cpp      CVS-1.1
*/


#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"
#include "simulation.h"
#include "one2onewiring.h"
#include "agent.h"
#include "nimm2.h"
#include "fixedsnake2elements.h"
#include "playground.h"

#include "invertnchannelcontroller.h"

ConfigList configs;

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
  playground->setGeometry(20.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);
  
  /*
  Nimm2* vehicle = new Nimm2(odeHandle);
  Position p = {5,0,0};
  vehicle->place(p);
  AbstractController *controller = new InvertNChannelController(10);  
  configs.push_back(controller);

  One2OneAgent* agent = new One2OneAgent(NoPlot); 
  agent->init(controller, vehicle);
  global.agents.push_back(agent);
  */

  FixedSnake2Elements* testjoints = new FixedSnake2Elements(odeHandle);
  testjoints->place(Position(0,0,0));
  AbstractController *controller2 = new InvertNChannelController(10);  
  configs.push_back(controller2);
  
  One2OneWiring* wiring2 = new One2OneWiring(new ColorUniformNoise());
  Agent* agent2 = new Agent(/*NoPlot*/GuiLogger);
  agent2->init(controller2, testjoints, wiring2);
  global.agents.push_back(agent2);
  
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


int main (int argc, char **argv)
{  
  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
