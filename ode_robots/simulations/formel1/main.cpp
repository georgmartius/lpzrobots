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
 *   Revision 1.2  2005-08-09 11:06:30  robot1
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
#include "playground.h"
#include "camera.h"
#include "simplecontroller.h"


ConfigList configs;
PlotMode plotMode = NoPlot;
SimpleController *controller;

float camPoint[3] = {5.0f,0.0f,1.0f};
float camAngle[3] = {180.0f,0.0f,0.0f};
 // the robot to follow
 Formel1* vehicle;
 // the position of robot
 double robotPoint[3]= {0.0f,0.0f,0.0f};


 /*
 
 // camera points, first is position, second the point to view 
//  double KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float camPos[3]= {5.0f,-0.0f,1.100f};
//  double KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};
  float camView[3] = {180.0f,0.0f,0.0f};
 // the position of robot
 double robotPos[3]= {0.0f,0.0f,0.0f};
 // the view of robot
 double RobotView[3] = {180.0f,0.0f,0.0f};
// the new positions and view of the camera and the robot
 float newCamPos[3];
 float newCamView[3];
 double newRobotPos[3];
 double newRobotView[3];
 

 
 // follows the robot while the camera holds its position,
 // only the angle will be adjusted
void followRobotWhileStanding() {
}

 // follows the robot while the camera moves its position,
 // only the position will be adjusted, the angle stays constant
void followRobotWhileMoving() {
	// getting first the position of robot
	Position pos=vehicle->getPosition();
	newRobotPos[0]=pos.x;
	newRobotPos[1]=pos.y;
	newRobotPos[2]=pos.z;
	// now getting the current angle of the camera	
	dsGetViewpoint(newCamPos,newCamView);
	
	// now adjusting the original position of the camera
	// new values must be stored as old too
	for (int i=0;i<=2;i++) {
		camPos[i]=newCamPos[i]+newRobotPos[i]-robotPos[i];
		camView[i]=newCamView[i]; // no change
		robotPos[i]=newRobotPos[i];
// 		robotView[i]=newRobotView[i]; // not used yet
	}
	dsSetViewpoint(camPos,camView);
}
 */
 
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
  dsSetViewpoint (camPoint, camAngle);
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  simulationConfig.noise=0.1;
  int chessTexture = dsRegisterTexture("chess.ppm");
  printf("Chess: %i", chessTexture);
  Playground* playground = new Playground(world, space);
  playground->setGeometry(100.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  obstacles.push_back(playground);

  vehicle = new Formel1(world, space, contactgroup);
 // vehicle->setTextures(DS_WOOD, chessTexture); 
  vehicle->place(Position(robotPoint[0],robotPoint[1],robotPoint[2]));
  
  // Controller
//AbstractController *controller = new InvertNChannelController(10);  
  controller = new SimpleController();
  
  // Wiring for Agent
  One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  
  // Agent for connecting Controller, Robot and Wiring
  Agent* agent = new Agent(plotMode);
  agent->init(controller, vehicle, wiring);
  agents.push_back(agent);

  // Simulation-Configuration
  configs.push_back(&simulationConfig);
  configs.push_back(controller);
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


// this function is called if the user pressed Ctrl-C
void config(){

  changeParams(configs);
}

// gets key from keyboard input, provided by ode
void command(int key){
    double val;
//     double velocityStep=0.05;
    double maxShift=0.5;
    double maxVelocity=5;
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

 void addCallback (bool draw, bool pause) {
	 if (draw) moveCamera(Following,*vehicle);
 }


int main (int argc, char **argv)
{  
  if(contains(argv, argc, "-g")) plotMode = GuiLogger;
  if(contains(argv, argc, "-l")) plotMode = GuiLogger_File;
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config,&command,0,&addCallback);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
