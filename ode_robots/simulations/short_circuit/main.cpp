#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include "noisegenerator.h"
#include "agent.h"
#include "one2onewiring.h"
#include "selectiveone2onewiring.h"
#include "derivativewiring.h"

#include "nimm2.h"
#include "shortcircuit.h"
#include "playground.h"

#include "sinecontroller.h"
#include "invertmotornstep.h"
#include "invertmotorspace.h"
#include "invertnchannelcontroller.h"

ConfigList configs;
PlotMode plotMode=NoPlot;
int channels;
int t=0;
double omega = 0.05;

SineWhiteNoise* sineNoise;

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird 
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Noise frequency modification with  < > , . r n\n" );
  dsPrint ( "Press Ctrl-C (on the console) for an basic commandline interface.\n\n" );


  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.setParam("noise",0.03);
  //  global.odeConfig.setParam("realtimefactor",0);
  //  global.odeConfig.setParam("drawinterval", 500);
  
  AbstractRobot* robot = new ShortCircuit(odeHandle, channels, channels);  
  //  AbstractRobot* robot = new Nimm2(odeHandle);  
  AbstractController *controller = new InvertMotorNStep(10);  
  //AbstractController *controller = new InvertMotorSpace(10,1.2);  
  //  controller->setParam("adaptrate",0.0);
  controller->setParam("epsA",0.01);
  controller->setParam("epsC",0.05);
  controller->setParam("factorB",0);
  //  AbstractController *controller = new InvertNChannelController(10);  
  //AbstractController *controller = new SineController();
  
  Agent* agent = new Agent(plotMode, Robot);
  
  //sineNoise = new SineWhiteNoise(omega,2,M_PI/2);
  //One2OneWiring* wiring = new One2OneWiring(sineNoise, true);
  One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise(), true);
 
  //AbstractWiring* wiring = new SelectiveOne2OneWiring(sineNoise, &select_firsthalf);
  // DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
//   c.useId=true;
//   c.useFirstD=true;
//   c.derivativeScale=20;
//   c.blindMotorSets=0;
//   AbstractWiring* wiring = new DerivativeWiring(c, new ColorUniformNoise(0.1));
  agent->init(controller, robot, wiring);
  global.agents.push_back(agent);
  
    configs.push_back(controller);
  showParams(configs);
}

// void addcallback (bool, bool){
//   t++;
//   sineNoise->setOmega(0.1*cos(t/100000.0));
// }

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

void command(GlobalData& global, int key){
  switch (key){
  case '>': omega+=0.05;
    break;
  case '<': omega-=0.05;
    break;
  case '.': omega+=0.005;
    break;
  case ',': omega-=0.005;
    break;
  case 'r': omega=0.05;
    break;
  case 'n': omega=0;
    break;
  }
  fprintf(stderr, "Omega: %g\n", omega);
  sineNoise->setOmega(omega);
}

void printUsage(const char* progname){
  printf("Usage: %s numchannels [-g] [-l]\n\tnumchannels\tnumber of channels\n\
\t-g\t\tuse guilogger\n\t-l\t\tuse guilogger with logfile", progname);
}

int main (int argc, char **argv)
{  
  if(argc<=1) printUsage(argv[0]);  
  channels = max(1,atoi(argv[1]));
  if(contains(argv, argc, "-g")) plotMode = GuiLogger;
  if(contains(argv, argc, "-l")) plotMode = GuiLogger_File;
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config, &command);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
