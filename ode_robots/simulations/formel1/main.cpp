/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.12  2005-11-22 15:49:35  robot3
 *   testing
 *
 *   Revision 1.11  2005/11/15 14:50:22  martius
 *   quark
 *
 *   Revision 1.10  2005/11/15 15:37:49  robot3
 *   test
 *
 *   Revision 1.9  2005/11/15 13:37:49  martius
 *   *** empty log message ***
 *
 *   Revision 1.8  2005/11/15 14:23:44  robot3
 *   raceground testet
 *
 *   Revision 1.7  2005/09/22 13:17:12  martius
 *   OdeHandle and GlobalData finished
 *   doInternalStuff included
 *
 *   Revision 1.6  2005/09/22 12:24:38  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.5  2005/08/23 11:40:16  robot1
 *   advancedTV mode tested
 *
 *   Revision 1.4  2005/08/22 12:42:17  robot1
 *   advancedTV mode tested
 *
 *   Revision 1.3  2005/08/12 11:56:08  robot1
 *   camera module deactivated (now in simulation.cpp integrated)
 *
 *   Revision 1.2  2005/08/09 11:06:30  robot1
 *   camera module included
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"
#include "simulation.h"
#include "agent.h"
#include "one2onewiring.h"
#include "formel1.h"
#include "raceground.h"
#include "camera.h"
#include "simplecontroller.h"
#include "stl_adds.h"


PlotMode plotMode = NoPlot;
SimpleController *controller;

float camPoint[3] = {-4.0f,0.0f,1.5f};
float camAngle[3] = {0.0f,0.0f,0.0f};
 // the robot to follow
 Formel1* vehicle;
 // the position of robot
 double robotPoint[3]= {0.0f,0.0f,0.0f};

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  dsSetViewpoint (camPoint, camAngle);
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.noise=0.1;
  global.odeConfig.setParam("realtimefactor",1.5);
  global.odeConfig.setParam("drawinterval",2);


  int chessTexture = dsRegisterTexture("chess.ppm");
  printf("Chess: %i\n", chessTexture);
  // Playground* playground = new Playground(odeHandle);
  RaceGround* Strecke = new RaceGround(odeHandle);
  list<string> segmentList;
  segmentList+=string("straightline");
  segmentList+=string("degree 180.0 2.5");
  segmentList+=string("straightline");
  segmentList+=string("degree -90.0 5.0");
  segmentList+=string("degree 90.0 5.0");
  segmentList+=string("straightline");
  segmentList+=string("degree 90.0 5.0");
  segmentList+=string("degree 90.0 5.0");
  segmentList+=string("degree -28.1 21.2");
  segmentList+=string("degree 28.1 21.2");
 //  segmentList+=string("degree 90.0f 10");
  //segmentList+=string("straightline");
  //segmentList+=string("degree 90 10");
  //segmentList+=string("straightline");
  //segmentList+=string("degree 90 10");
  cout << "now adding segments:\n";
 Strecke->addSegments(segmentList);
  cout << "finished adding segments! \n";
  Strecke->setPosition(0.0f,0.0f,0.0f);
  global.obstacles.push_back(Strecke);

  vehicle = new Formel1(odeHandle);
 // vehicle->setTextures(DS_WOOD, chessTexture); 
  vehicle->place(Position(robotPoint[0],robotPoint[1],robotPoint[2]));
  
  // Controller
//AbstractController *controller = new InvertNChannelController(10);  
  controller = new SimpleController();
  
  // Wiring for Agent
  One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.001));
  
  // Agent for connecting Controller, Robot and Wiring
  Agent* agent = new Agent(plotMode);
  agent->init(controller, vehicle, wiring);
  global.agents.push_back(agent);

  // Simulation-Configuration
  global.configs.push_back(controller);
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


// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){

  changeParams(global.configs);
}

// gets key from keyboard input, provided by ode
void command(const OdeHandle& odeHandle, GlobalData& global, int key){
    double val;
//     double velocityStep=0.05;
    double maxShift=0.5;
    double maxVelocity=2.0f;
    double shiftStep=0.167;
    switch (key){
    case 'w': // forward
	controller->setParam("velocity",maxVelocity);
	break; 
    case 's': // backward
	controller->setParam("velocity",-maxVelocity);
	break; 
    case 'a': // left
	val = controller->getParam("leftRightShift");
	if (val>(shiftStep-maxShift)) 
		val-=shiftStep;
	else // so it cant get anymore then -1
		val=-maxShift;
	controller->setParam("leftRightShift", val);
	break; 
    case 'd': // right
	    val = controller->getParam("leftRightShift");
	    if (val<(maxShift-shiftStep)) 
		    val+=shiftStep;
	    else // so it cant get anymore then 1
		    val=maxShift;
	    controller->setParam("leftRightShift", val);
	    break;
    }
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
  simulation_init(&start, &end, &config,&command);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
