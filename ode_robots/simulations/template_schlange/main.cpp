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
 *   Revision 1.16.4.1  2005-11-15 12:30:09  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.16  2005/11/09 14:34:14  fhesse
 *   snake 2 working
 *
 *   Revision 1.15  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include <selforg/noisegenerator.h>
#include "odeagent.h"
#include <selforg/one2onewiring.h>
#include "playground.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/noisegenerator.h>

#include "schlange.h"

ConfigList configs;
list<PlotOption> plotoptions;

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.noise=0.1;
    
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(7.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);
    
  // snake1
  SchlangenConf conf = Schlange::getStandartConf();  
  Schlange* schlange1 = new Schlange ( 1 , odeHandle, conf);
  Color col(0,0.5,0.8);
  schlange1->place(Position(0,0,0),&col);
  AbstractController *controller = new InvertNChannelController(10);  
  
  One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());
  OdeAgent* agent = new OdeAgent(plotoptions);
  agent->init(controller, schlange1, wiring);
  global.agents.push_back(agent);
  configs.push_back(controller);
  
  //snake2
  Schlange* schlange2 = new Schlange ( 2 , odeHandle, conf);
  Position p2 (0,2,0);
  Color col2 (0.5,0,0.5);
  schlange2->place(p2,&col2);
  AbstractController *controller2 = new InvertNChannelController(10);  
  
  One2OneWiring* wiring2 = new One2OneWiring(new ColorUniformNoise());
  OdeAgent* agent2 = new OdeAgent(NoPlot);
  agent2->init(controller2, schlange2, wiring2);
  global.agents.push_back(agent2);
  configs.push_back(controller2);
 
    
  showParams(configs);
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
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv, argc, "-l")) plotoptions.push_back(PlotOption(GuiLogger_File));
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}

 
