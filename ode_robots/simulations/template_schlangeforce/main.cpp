#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"

#include "simulation.h"
#include "one2oneagent.h"
#include "playground.h"
#include "sphere.h"

#include "invertnchannelcontroller.h"
#include "invertmotorspace.h"
#include "sinecontroller.h"
#include "sinecontrollerWP.h"

#include "schlangeforce.h"
#include "schlange.h"
#include "nimm2.h"

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

  dWorldSetGravity ( world , 0 , 0 , 0 );
  

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  //float KameraXYZ[3]= {0.276f,7.12f,1.78f};
  //float KameraViewXYZ[3] = {-88.0f,-5.5f,0.0000f};
  float KameraXYZ[3]= {2.4f,7.2f,10.47f};
  float KameraViewXYZ[3] = {-88.0f,-5.5f,0.0000f};

  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  configs.push_back(&simulationConfig);


  Playground* playground = new Playground(&world, &space);
  playground->setGeometry(7.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  obstacles.push_back(playground);

  Sphere* sphere[1];
  for (int i=0; i<1; i++){
    sphere[i]= new Sphere(&world, &space);
    sphere[i]->setPosition(0,0,3+i); //positionieren und generieren
    obstacles.push_back(sphere[i]);
  }

  //****************
  SchlangeForce* schlange1 = new SchlangeForce ( 1 , &world , &space , &contactgroup , 0 , 0 , 0.25 , /*4*/8 , 0.8/*0.5*/ , 0.2 , 0 , 0.4/*0.04*/ , 2 , 10 , anglerate);
  Position p = {0,0,10};
  Color col = {0,0.5,0.8};
  schlange1->place(p,&col); 
  //AbstractController *controller = new InvertNChannelController(100/*,true*/);  
  AbstractController *controller = new InvertMotorSpace(100/*,true*/);  
  //AbstractController *controller = new SineController();  
  //AbstractController *controller = new SineControllerWP();  
  
  One2OneAgent* agent = new One2OneAgent(new ColorUniformNoise(0.1), plotMode);
  agent->init(controller, schlange1);
  agents.push_back(agent);
  configs.push_back(controller);
  configs.push_back(schlange1);

 
  simulationConfig.setParam("noise",0.1);
  simulationConfig.setParam("simstepsize",0.005);
  simulationConfig.setParam("drawinterval",1);
  simulationConfig.setParam("controlinterval",1);

   controller->setParam("epsC",0.01);
//   // controller->setParam("desens",0.0);
//   controller->setParam("s4delay",1.0);
//   controller->setParam("s4avg",1.0);
   controller->setParam("epsA",0.01);
//   controller->setParam("factorB",0.0);
//   controller->setParam("zetaupdate",0.1);

  schlange1->setParam("gamma",/*0.0000*/ 0.0);
  schlange1->setParam("frictionGround",0.1);
  schlange1->setParam("factorForce", /*0.0005*/3);
  schlange1->setParam("factorSensors", /*20.0 */5);
  


  /*
  SchlangeForce* schlange3 = new SchlangeForce ( 2 , &world , &space , &contactgroup , 0 , 0 , 0.25 , 4 , 0.5 , 0.2 , 0 , 0.4 , 2 , 10 , anglerate);
  Position p3 = {1,1,0};
  Color col3 = {1,1,0};
  schlange1->place(p3,&col3);
  AbstractController *controller3 = new InvertNChannelController(10,true);  
  
  One2OneAgent* agent3 = new One2OneAgent(NoPlot/*GuiLogger* /);
  agent3->init(controller3, schlange3);
  agents.push_back(agent3);
  configs.push_back(controller3);
  configs.push_back(schlange3);
  */

  /*
  Nimm2* vehicle = new Nimm2 ( &world , &space , &contactgroup);
  Position p2 = {0,0,3};
  vehicle->place(p2);
  AbstractController *controller2 = new InvertNChannelController(10,true);  
  
  One2OneAgent* agent2 = new One2OneAgent(NoPlot/*GuiLogger* /);
  agent2->init(controller2, vehicle);
  agents.push_back(agent2);
  //  configs.push_back(controller2);
  */
  
  
  
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
 
