#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include "one2oneagent.h"
#include "vehicle_2wheels.h"
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
  dsPrint ( "\n\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "--------------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  simulationConfig.noise=0.1;
  
  Playground* playground = new Playground(&world, &space);
  playground->setGeometry(7.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  obstacles.push_back(playground);

  Vehicle* vehicle = new Vehicle(&world, &space);
  Position p = {0,0,0};
  vehicle->place(p);
  AbstractController *controller = getController(2,10);  
  
  One2OneAgent* agent = new One2OneAgent(true);
  agent->init(controller, vehicle);
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


void config(){
  changeParams(configs);
}


int main (int argc, char **argv)
{  
  simulation_init(&start, &end, &config);
  simulation_start(argc, argv);
  simulation_close();  
  return 0;
}
 
