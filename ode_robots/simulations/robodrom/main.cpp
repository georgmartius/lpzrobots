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

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );
  
  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {12.4f,-13.0f,10.5f};
  float KameraViewXYZ[3] = {132.0000f,-25.0000f,0.0000f};
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden
  
  dsSetGroundTexture(dsRegisterTexture("greenground.ppm"));

  // initialization
  global.odeConfig.noise=0.05;
  double height=2.2;
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(20.0, 0.2, 0.5+height);
  playground->setColor(34/255.0, 97/255.0, 32/255.0);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);
  
  //Terrainground *terrainground = new Terrainground(odeHandle, 20.0, height, "terrains/dip128_flat.ppm");
  //  int tex = dsRegisterTexture("terrains/dip128_flat_texture.ppm", true);
  Terrainground *terrainground = new Terrainground(odeHandle, 20.0, height, "terrains/3potential.ppm");
  int tex = dsRegisterTexture("terrains/3potential_texture.ppm", true);
  terrainground->setTextureID(tex);

  terrainground->setPosition(-10,-10,0.5);
  global.obstacles.push_back(terrainground);
  
    Color col;
  for(int i=0; i<3; i++){
    SphererobotArmsConf conf = SphererobotArms::getStandartConf();  
    conf.diameter=1.5;
    conf.spheremass=0.01;
    conf.pendularrange=0.35; 
    //SphererobotArms* sphere = new SphererobotArms ( odeHandle, conf);
    sphere = new SphererobotArms ( odeHandle, conf, 0.4);
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
    if(i==2){
      col.r=0;
      col.g=1;
      col.b=0.4;
    }
    if(i==1){
      sphere->place ( Position ( 2 , -2 , 6 ) , &col );
    } else 
      sphere->place ( Position ( (i*4)-2 , (i*4)-2 , 6 ) , &col );
    
    //AbstractController *controller = new InvertNChannelController(10);  
    AbstractController *controller = new InvertMotorNStep(50);
    controller->setParam("factorB", 0.0);
    controller->setParam("steps", i+1);
    
    AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
    Agent* agent = new Agent ( i==0 ? plotMode : NoPlot );
    agent->init ( controller , sphere , wiring );
    global.agents.push_back ( agent );
    configs.push_back ( controller );
  }


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

//Funktion die eingegebene Befehle/kommandos verarbeitet
void command (const OdeHandle&, GlobalData& globalData, int cmd) 
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
void config(GlobalData& global){
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
 
// dip128_flat
//     height=5
//     conf.diameter=2;
//     conf.spheremass=0.01;
//     conf.pendularrange=0.33; 

// random seed 1126621396
// random seed 1126688945

//3potential
//     height=2
//     conf.diameter=1.5;
//     conf.spheremass=0.01;
//     conf.pendularrange=0.35; 

// 1126702048
