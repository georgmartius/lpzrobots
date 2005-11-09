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
 *   Revision 1.5  2005-11-09 14:08:48  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2005/11/09 13:41:10  fhesse
 *   GPL added
 *                                                                 *
 *                                                                         * 
***************************************************************************/
#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"
#include "simulation.h"
#include "agent.h"
#include "one2onewiring.h"
#include "nimm2.h"
#include "playground.h"
#include "octaplayground.h"

#include "invertnchannelcontroller.h"
#include "invertmotornstep.h"

ConfigList configs;
PlotMode plotMode = NoPlot;

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {0.25f,-6.0f,3.79f};
  float KameraViewXYZ[3] = {90.00f,-43.50f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.noise=0.1;
    
  Playground* playground = new Playground(odeHandle, /*factorxy=1*/ -4);
  playground->setGeometry(8.0, 0.09, 0.25);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);
  

//   OctaPlayground* octaplayground = new OctaPlayground(odeHandle, 330);
//   octaplayground->setGeometry(6.5, 0.09, 0.25);
//   octaplayground->setPosition(0,0,0); // playground positionieren und generieren
//   global.obstacles.push_back(octaplayground);

  Nimm2* nimm2;
  AbstractController* controller;
  AbstractWiring* wiring;
  Agent* agent;

  int chessTexture = dsRegisterTexture("chess.ppm");

  for (int j=-1; j<2; j++){ 
    for (int i=-4; i<5; i++){
      //      nimm2 = new Nimm2(odeHandle);
      Nimm2Conf conf = Nimm2::getDefaultConf();
      conf.speed=20;
      conf.force=0.5;
      conf.bumper=true;
      conf.cigarMode=true;
      nimm2 = new Nimm2(odeHandle, conf);
      nimm2->setTextures(DS_WOOD, chessTexture); 
      Color c(2,2,0);
      if ((i==0) && (j==0)) {nimm2->place(Position(j*0.26,i*0.26,0), &c);}
      else {
	nimm2->place(Position(j*2.5,i*1.26,0));
      }
      //controller = new InvertMotorNStep(10);  
      controller = new InvertNChannelController(10);  

      configs.push_back(controller);
      
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      if ((i==0) && (j==0)) {agent = new Agent(plotMode);}
      else {
	agent = new Agent(NoPlot);
      }
      agent->init(controller, nimm2, wiring);
      global.agents.push_back(agent);

      controller->setParam("factorB",0);
    }
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


// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){
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
 
