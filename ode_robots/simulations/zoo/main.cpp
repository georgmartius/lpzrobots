#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include "agent.h"
#include "noisegenerator.h"

#include "invertnchannelcontroller.h"
#include "invertmotorspace.h"
#include "invertmotornstep.h"
#include "one2onewiring.h"
#include "derivativewiring.h"

#include "playground.h"
#include "sphere.h"

#include "hurlingsnake.h"
#include "schlangeforce.h"
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

  simulationConfig.setParam("gravity",-9.81); // do not use 'simulationConfig.gravity=0.0;', 
                                            // because world is already initialized and 
                                            // dWorldSetGravity will not be called when 
                                            // you only set the value  

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  //float KameraXYZ[3]= {0.276f,7.12f,1.78f};
  //float KameraViewXYZ[3] = {-88.0f,-5.5f,0.0000f};
  float KameraXYZ[3]= {2.4f,7.2f,5.47f};
  float KameraViewXYZ[3] = {-88.0f,-20.5f,-0.0000f};

  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  simulationConfig.setParam("noise",0.1);
  simulationConfig.setParam("drawinterval",1);
  simulationConfig.setParam("controlinterval",1);
  // initialization
  configs.push_back(&simulationConfig);

  Playground* playground = new Playground(world, space);
  playground->setGeometry(15.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  obstacles.push_back(playground);

  Sphere* sphere;
  for (int i=-1; i<1; i++){
    sphere = new Sphere(world, space);
    sphere->setPosition(i*0.5,i*0.5,1.0); //positionieren und generieren
    obstacles.push_back(sphere);
  }

  Agent* agent;
  AbstractWiring* wiring;
  AbstractRobot* robot;
  AbstractController *controller;
  DerivativeWiringConf deriveconf;

  SchlangeForce* snake;
  //****************/
  snake = new SchlangeForce ( 1 , world , space , contactgroup , 
			      0      , 0   , 0.25 , // start position
			      4/*4*/ , 0.8/*0.5*/, // #arms, len,
			      0.2    , 0   , 0.4/*0.04*/ ,  // diameter, distance, mass
			      2      , 10  , anglerate // maxforce, velofactor, sensoroutput
			      );
  {
    Position p = {-5,-5,0};
    Color col = {0,0.5,0.8};
    snake->place(p,&col); 
  }
  controller = new InvertMotorNStep(10);  
  wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  agent = new Agent( NoPlot );
  agent->init(controller, snake, wiring);
  agents.push_back(agent);
  configs.push_back(controller);
  configs.push_back(snake);   
  snake->setParam("gamma",/*0.0000*/ 0.0);
  snake->setParam("frictionGround",0.1);
  snake->setParam("factorForce", /*0.0005*/3);
  snake->setParam("factorSensors", /*20.0 */5);
  
  showParams(configs);

  //****************/
  snake = new SchlangeForce ( 1 , world , space , contactgroup , 
			      0      , 0   , 0.25 , // start position
			      8/*4*/ , 0.8/*0.5*/, // #arms, len,
			      0.2    , 0   , 0.4/*0.04*/ ,  // diameter, distance, mass
			      2      , 10  , anglerate // maxforce, velofactor, sensoroutput
			      );
  {
    Position p = {0,0,0};
    Color col = {0,0.5,0.8};
    snake->place(p,&col); 
  }
  controller = new InvertMotorNStep(10);  
  wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  agent = new Agent( NoPlot );
  agent->init(controller, snake, wiring);
  agents.push_back(agent);
  configs.push_back(controller);
  configs.push_back(snake);   
  snake->setParam("gamma",/*0.0000*/ 0.0);
  snake->setParam("frictionGround",0.1);
  snake->setParam("factorForce", /*0.0005*/3);
  snake->setParam("factorSensors", /*20.0 */5);
  
  showParams(configs);

  //******* N I M M  2 *********/
  for(int r=0; r < 3; r++) {
    robot = new Nimm2(world, space, contactgroup,1.6);
    Position p = {(r-1)*5,5,0};
    robot->place(p);
    controller = new InvertMotorNStep(10);   
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent = new Agent( NoPlot );
    agent->init(controller, robot, wiring);
    agents.push_back(agent);        
  }

  //****** H U R L I N G **********/
  for(int r=0; r < 2; r++) {
    robot = new HurlingSnake(world, space, contactgroup);
    Color c;    
    if (r==0) c=mkColor(2,2,0);
    if (r==1) c=mkColor(0,2,0);
    robot->place(mkPosition(r*5,-7,0.3), &c);
    controller = new InvertMotorNStep(10);   
    // deriveconf = DerivativeWiring::getDefaultConf();
//     deriveconf.blindMotorSets=0;
//     deriveconf.useId = true;
//     deriveconf.useFirstD = true;
//     deriveconf.derivativeScale = 50;
//     wiring = new DerivativeWiring(deriveconf, new ColorUniformNoise(0.1));
    wiring = new One2OneWiring(new ColorUniformNoise(0.05));
    agent = new Agent( NoPlot );
    agent->init(controller, robot, wiring);
    configs.push_back(controller);
    agents.push_back(agent);     
  }


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
 
