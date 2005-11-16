/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   $Log$
 *   Revision 1.15.4.2  2005-11-16 11:27:38  martius
 *   invertmotornstep has configuration
 *
 *   Revision 1.15.4.1  2005/11/15 12:29:56  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.15  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include <selforg/noisegenerator.h>
#include "simulation.h"
#include "odeagent.h"
#include <selforg/one2onewiring.h>
#include "forcedsphere.h"
#include "sphererobotarms.h"
#include "playground.h"
#include "terrainground.h"

#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>


list<PlotOption> plotoptions;

SphererobotArms* sphere ;
//const double height = 6.5;
const double height = 3;

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
  AbstractController *controller = new InvertMotorNStep();
  //    controller->setParam("factorB", 0.1);
  controller->setParam("steps", 2);
  //    controller->setParam("nomupdate", 0.005);
  
  AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
  OdeAgent* agent = new OdeAgent ( i==0 ? plotoptions : list<PlotOption>() );
  agent->init ( controller , sphere , wiring );
  global.agents.push_back ( agent );
  global.configs.push_back ( controller );  
}

void removeRobot(GlobalData& global){
  if(!global.agents.empty()){
    OdeAgentList::iterator i =  global.agents.end()-1;
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
  Terrainground *terrainground = new Terrainground(odeHandle, 20.0, height, 
						   "terrains/macrospheresLMH_256.ppm",
   						   Terrainground::LowMidHigh);
    int tex = dsRegisterTexture("terrains/macrospheresSum_256.ppm", true);
  //terrainground->setTextureID(tex);
//  Terrainground *terrainground = new Terrainground(odeHandle, 20.0, height, "terrains/potLMH_256.ppm", 
//   						   Terrainground::LowMidHigh);
  //int tex = dsRegisterTexture("terrains/potTex_1024.ppm", true);
  // Terrainground *terrainground = new Terrainground(odeHandle, 20.0, height, "terrains/macrospheresLMH_256.ppm", 
//   						   Terrainground::LowMidHigh);
//   int tex = dsRegisterTexture("terrains/macrospheresTex_1024.ppm", true);
  terrainground->setTextureID(tex);
  //terrainground->setColor(1,1,1);
  
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
   
   for(OdeAgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++){
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
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv, argc, "-l")) plotoptions.push_back(PlotOption(GuiLogger_File));
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
