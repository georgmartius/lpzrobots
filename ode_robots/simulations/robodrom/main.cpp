#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"
#include "simulation.h"
#include "agent.h"
#include "one2onewiring.h"
#include "forcedsphere.h"
#include "sphererobotarms.h"
#include "playground.h"
#include "terrainground.h"

#include "invertmotorspace.h"
#include "invertmotornstep.h"
#include "sinecontroller.h"


ConfigList configs;
PlotMode plotMode = NoPlot;

SphererobotArms* sphere ;

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
  float KameraXYZ[3]= {12.4f,-12.0f,13.5f};
  float KameraViewXYZ[3] = {139.0000f,-25.0000f,0.0000f};
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden
  
  dsSetGroundTexture(dsRegisterTexture("greenground.ppm"));

  // initialization
  simulationConfig.noise=0.05;
  double height=5;
  Playground* playground = new Playground(world, space);
  playground->setGeometry(20.0, 0.2, 0.3+height);
  playground->setColor(34/255.0, 97/255.0, 32/255.0);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  obstacles.push_back(playground);
  
  Terrainground *terrainground = new Terrainground(world, space, 20.0, height, "terrains/dip128_flat.ppm");
  int tex = dsRegisterTexture("terrains/dip128_flat_texture.ppm", true);
  //  Terrainground *terrainground = new Terrainground(world, space, 20.0, height, "terrains/3potential.ppm");
  //  int tex = dsRegisterTexture("terrains/3potential_texture.ppm", true);
  terrainground->setTextureID(tex);

  terrainground->setPosition(-10,-10,0.3);
  obstacles.push_back(terrainground);
  
  configs.push_back(&simulationConfig);
  Color col;
  for(int i=0; i<1; i++){
    SphererobotArmsConf conf = SphererobotArms::getStandartConf();  
    conf.diameter=2;
    conf.spheremass=0.01;
    conf.pendularrange=0.35; 
    //SphererobotArms* sphere = new SphererobotArms ( ODEHandle(world , space , contactgroup), conf);
    sphere = new SphererobotArms ( ODEHandle(world , space , contactgroup), conf, 0.4);
    sphere->setTexture(DS_WOOD);  
    if(i==0){
      col.r=0;
      col.g=0.5;
      col.b=1;
    }
    if(i==1){
      col.r=1;
      col.g=0.4;
      col.b=0;
    }

    sphere->place ( Position ( (i*4)-2 , (i*4)-2 , 6 ) , &col );
    //AbstractController *controller = new InvertNChannelController(10);  
    AbstractController *controller = new InvertMotorNStep(50);
    controller->setParam("factorB", 0.0);
    controller->setParam("steps", i+1);
    
    AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
    Agent* agent = new Agent ( i==0 ? plotMode : NoPlot );
    agent->init ( controller , sphere , wiring );
    agents.push_back ( agent );
    configs.push_back ( controller );
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

//Funktion die eingegebene Befehle/kommandos verarbeitet
void command (int cmd) 
{
  //dsPrint ( "Eingabe erfolgt %d (`%c')\n" , cmd , cmd );
  switch ( (char) cmd )
    {
    case 'a' : dBodyAddForce ( sphere->object[SphererobotArms::Base].body , 50 ,0 , 0 ); break;
    case 's' : dBodyAddForce ( sphere->object[SphererobotArms::Base].body , 0 , 50 , 0); break;
    case 'q' : dBodyAddTorque( sphere->object[SphererobotArms::Base].body , 0 , 0 , 10 ); break;
    case 'w' : dBodyAddTorque( sphere->object[SphererobotArms::Base].body , 0 , 0 , -10 ); break;
    }
}

// this function is called if the user pressed Ctrl-C
void config(){
  changeParams(configs);
}

void printUsage(const char* progname){
  printf("Custom: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile\n", progname);
}



int main (int argc, char **argv)
{  
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
 
