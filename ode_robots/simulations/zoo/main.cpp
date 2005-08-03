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
#include "nimm4.h"

// for video capturing use:
// realtimefactor=0.5
// drawinterval=5

ConfigList configs;
PlotMode plotMode = NoPlot;

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
  int chessTexture = dsRegisterTexture("chess.ppm");
  printf("Chess: %i", chessTexture);
  int dust = dsRegisterTexture("dusty.ppm");
  printf("Chess: %i", chessTexture);


  //Anfangskameraposition und Punkt auf den die Kamera blickt
  //float KameraXYZ[3]= {0.276f,7.12f,1.78f};
  //float KameraViewXYZ[3] = {-88.0f,-5.5f,0.0000f};
  float KameraXYZ[3]= {-0.0f, 10.4f, 6.8f};
  float KameraViewXYZ[3] = {-90.0f,-40.0f,0.0000f};

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
  for (int i=-3; i<3; i++){
    sphere = new Sphere(world, space);
    sphere->setColor(184 / 255.0, 233 / 255.0, 237 / 255.0);
    sphere->setTexture(dust);
    sphere->setPosition(i*0.5-2,i*0.5,1.0); //positionieren und generieren

    obstacles.push_back(sphere);
  }

  ODEHandle odeHandle(world, space, contactgroup);

  Agent* agent;
  AbstractWiring* wiring;
  AbstractRobot* robot;
  AbstractController *controller;

  SchlangeForce* snake;
  SchlangenConf snakeConf = SchlangeForce::getDefaultConf();
  //******* S C H L A N G E  (Short) *********/
  snakeConf.armAnzahl=4;
  snakeConf.maxWinkel=M_PI/3;
  snakeConf.frictionGround=0.1;
  snakeConf.factorForce=3;
  snakeConf.factorSensors=5;
  snake = new SchlangeForce ( 1 , odeHandle, snakeConf );
  {
    Color col(0,0.5,0.8);
    snake->place(Position(-5,-5,0),&col); 
  }
  controller = new InvertMotorNStep(10);  
  wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  agent = new Agent( plotMode );
  agent->init(controller, snake, wiring);
  agents.push_back(agent);
  configs.push_back(controller);
  configs.push_back(snake);   
  snake->setParam("gamma",/*0.0000*/ 0.0);
  
  showParams(configs);

  //******* S C H L A N G E  (Long)  *********/
  snakeConf = SchlangeForce::getDefaultConf();
  snakeConf.armAnzahl   = 8;
  snakeConf.maxWinkel   = M_PI/3;
  snakeConf.frictionGround=0.1;
  snakeConf.factorForce=3.5;
  snakeConf.factorSensors=5;
  snake = new SchlangeForce ( 1 , odeHandle, snakeConf );
  {
    Color col(0,0.5,0.8);
    snake->place(Position(0,0,0),&col); 
  }
  controller = new InvertMotorNStep(10);  
  wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  agent = new Agent( NoPlot );
  agent->init(controller, snake, wiring);
  agents.push_back(agent);
  configs.push_back(controller);
  configs.push_back(snake);   
  snake->setParam("gamma",/*0.0000*/ 0.0);
  
  showParams(configs);

  //******* N I M M  2 *********/
  for(int r=0; r < 3; r++) {
    robot = new Nimm2(world, space, contactgroup,1.6);
    Position p((r-1)*5,5,0);
    robot->place(p);
    ((Nimm2*)robot)->setTextures(DS_WOOD, chessTexture);
    controller = new InvertMotorSpace(10);   
    controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent = new Agent( NoPlot );
    agent->init(controller, robot, wiring);
    agents.push_back(agent);        
  }

  //******* N I M M  4 *********/
  for(int r=0; r < 1; r++) {
    robot = new Nimm4(world, space, contactgroup);
    Position p((r-1)*5,-3,0);
    robot->place(p);
    ((Nimm4*)robot)->setTextures(DS_WOOD, chessTexture);
    controller = new InvertMotorSpace(20);
    controller->setParam("s4avg",10); 
    controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent = new Agent( NoPlot );
    agent->init(controller, robot, wiring);
    agents.push_back(agent);        
  }

  //****** H U R L I N G **********/
  for(int r=0; r < 2; r++) {
    HurlingSnake* snake;
    snake = new HurlingSnake(world, space, contactgroup);
    Color c;    
    if (r==0) c=Color(0.8, 0.8, 0);
    if (r==1) c=Color(0,   0.8, 0);
    snake->place(Position(r*5,-6,0.3), &c);
    snake->setParam("factorForce",8);

    controller = new InvertMotorNStep(10);   
    // deriveconf = DerivativeWiring::getDefaultConf();
//     deriveconf.blindMotorSets=0;
//     deriveconf.useId = true;
//     deriveconf.useFirstD = true;
//     deriveconf.derivativeScale = 50;
//     wiring = new DerivativeWiring(deriveconf, new ColorUniformNoise(0.1));
    wiring = new One2OneWiring(new ColorUniformNoise(0.05));
    agent = new Agent( NoPlot );
    agent->init(controller, snake, wiring);
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
  printf("Usage: %s [-g] [-l] [-r seed]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile\n\t-r seed\trandom number seed ", progname);
  exit(0);
}

int main (int argc, char **argv)
{  
  if(contains(argv, argc, "-g")) plotMode = GuiLogger;
  if(contains(argv, argc, "-l")) plotMode = GuiLogger_File;
  if(contains(argv, argc, "-h")) printUsage(argv[0]);
  int seedIndex = contains(argv, argc, "-r");
  long seed;
  // initialize random number generator
  if(seedIndex && argc > seedIndex) {
    seed=atoi(argv[seedIndex]);
  }else{
    time(0);
  }
  printf("Use random number seed: %i\n", seed);
  srand(seed);    

  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
