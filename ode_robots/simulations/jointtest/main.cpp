#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"
#include "simulation.h"
#include "agent.h"
#include "one2onewiring.h"
#include "nimm2.h"
#include "jointtest.h"
#include "playground.h"

#include "invertnchannelcontroller.h"

ConfigList configs;

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
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  simulationConfig.noise=0.1;
  configs.push_back(&simulationConfig);
  
  Playground* playground = new Playground(&world, &space);
  playground->setGeometry(20.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  obstacles.push_back(playground);
  
  /*
  Nimm2* vehicle = new Nimm2(&world, &space, &contactgroup);
  Position p = {5,0,0};
  vehicle->place(p);
  AbstractController *controller = new InvertNChannelController(10);  
  configs.push_back(controller);

  One2OneWiring* wiring = new One2OneWiring(); 
  Agent* agent = new Agent(new WhiteUniformNoise(), NoPlot); 
  agent->init(controller, vehicle, wiring);
  agents.push_back(agent);
  */

  JointTest* testjoints = new JointTest(&world, &space, &contactgroup);
  Position p2 = {0,0,0};
  testjoints->place(p2);
  AbstractController *controller2 = new InvertNChannelController(10);  
  configs.push_back(controller2);
  
  One2OneWiring* wiring2 = new One2OneWiring(new WhiteUniformNoise());
  Agent* agent2 = new Agent(/*NoPlot*/GuiLogger);
  agent2->init(controller2, testjoints, wiring2);
  agents.push_back(agent2);
  
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


int main (int argc, char **argv)
{  
  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
