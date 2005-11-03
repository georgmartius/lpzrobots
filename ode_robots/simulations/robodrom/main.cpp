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


PlotMode plotMode = NoPlot;

SphererobotArms* sphere ;
const double height = 6.5;

void addRobot(const OdeHandle& odeHandle, GlobalData& global, int i){
  Color col;
  
  SphererobotArmsConf conf = SphererobotArms::getStandartConf();  
  conf.diameter=1.0;
  conf.pendularrange= 0.35; // 0.15;
  //SphererobotArms* sphere = new SphererobotArms ( odeHandle, conf);
  sphere = new SphererobotArms ( odeHandle, conf, 0.4);
  sphere->setTexture(DS_WOOD);  
  if(i==0){
    col.r=0;
    col.g=0.5;
    col.b=1;
    sphere->place ( Position ( 9.5 , 0 , height+1 ) , &col );
  }else
    if(i==1){
      col.r=1;
      col.g=0.4;
      col.b=0;
      sphere->place ( Position ( 2 , -2 , height+1 ) , &col );
    } else {
      col.r=0;
      col.g=1;
      col.b=0.4;
      sphere->place ( Position ( double(rand())/RAND_MAX*10 , 0 , height+1 ) , &col );
    }
  
  
  //AbstractController *controller = new InvertNChannelController(10);  
  AbstractController *controller = new InvertMotorNStep(30);
  //    controller->setParam("factorB", 0.1);
  //    controller->setParam("steps", 2);
  //    controller->setParam("nomupdate", 0.005);
  
  AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
  Agent* agent = new Agent ( i==0 ? plotMode : NoPlot );
  agent->init ( controller , sphere , wiring );
  global.agents.push_back ( agent );
  global.configs.push_back ( controller );  
}

void removeRobot(GlobalData& global){
  if(!global.agents.empty()){
    AgentList::iterator i =  global.agents.end()-1;
    delete (*i)->getRobot();
    delete (*i)->getController(); 
    delete (*i);
    global.agents.erase(i);    
  }
}


//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );
  
  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {12.4f,-13.0f,15.5f};
  float KameraViewXYZ[3] = {132.0000f,-25.0000f,0.0000f};
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden
  
  dsSetGroundTexture(dsRegisterTexture("greenground.ppm"));

  // initialization
  global.odeConfig.setParam("noise",0.05);
  global.odeConfig.setParam("controlinterval",1);
  global.odeConfig.setParam("realtimefactor",0);
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(20.0, 0.2, 0.5+height);
  playground->setColor(34/255.0, 97/255.0, 32/255.0);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);
  
  //Terrainground *terrainground = new Terrainground(odeHandle, 20.0, height, "terrains/dip128_flat.ppm");
  //int tex = dsRegisterTexture("terrains/dip128_flat_texture.ppm", true);
  //Terrainground *terrainground = new Terrainground(odeHandle, 20.0, height, "terrains/3potential.ppm");
  //  int tex = dsRegisterTexture("terrains/3potential_texture.ppm", true);
  //terrainground->setTextureID(tex);
  Terrainground *terrainground = new Terrainground(odeHandle, 20.0, height, "terrains/potLMH_256.ppm", 
   						   Terrainground::LowMidHigh);
  //int tex = dsRegisterTexture("terrains/potTex_1024.ppm", true);
  // Terrainground *terrainground = new Terrainground(odeHandle, 20.0, height, "terrains/macrospheresLMH_256.ppm", 
//   						   Terrainground::LowMidHigh);
//   int tex = dsRegisterTexture("terrains/macrospheresTex_1024.ppm", true);
  //terrainground->setTextureID(tex);
  terrainground->setColor(1,1,1);
  
  terrainground->setPosition(-10,-10,0.5);
  global.obstacles.push_back(terrainground);
  
  addRobot(odeHandle, global, 0);
  addRobot(odeHandle, global, 1);


  showParams(global.configs);
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
void command (const OdeHandle& odeHandle, GlobalData& globalData, int cmd) 
{
  //dsPrint ( "Eingabe erfolgt %d (`%c')\n" , cmd , cmd );
  switch ( (char) cmd )
    {
    case 'a' : dBodyAddForce ( sphere->object[SphererobotArms::Base].body , 50 ,0 , 0 ); break;
    case 's' : dBodyAddForce ( sphere->object[SphererobotArms::Base].body , 0 , 50 , 0); break;
    case 'q' : dBodyAddTorque( sphere->object[SphererobotArms::Base].body , 0 , 0 , 10 ); break;
    case 'w' : dBodyAddTorque( sphere->object[SphererobotArms::Base].body , 0 , 0 , -10 ); break;
    case 'n' : addRobot(odeHandle, globalData, 3); break;
    case 'r' : removeRobot(globalData);            break;
    }
}

// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){
  changeParams(global.configs);
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
