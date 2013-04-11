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
 *   Revision 1.20  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.19  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.18  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.17  2006/07/14 12:23:46  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.7.4.2  2006/06/25 17:01:54  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.7.4.1  2005/11/15 12:29:39  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
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
#include <ode-dbl/ode.h>

#include <selforg/noisegenerator.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>
#include "formel1.h"
#include <ode_robots/playground.h>
#include <ode_robots/camera.h>
#include "simplecontroller.h"


ConfigList configs;
list<PlotOption> plotoptions;
SimpleController *controller;

float camPoint[3] = {5.0f,0.0f,1.5f};
float camAngle[3] = {180.0f,0.0f,0.0f};
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
  int chessTexture = dsRegisterTexture("chess.ppm");
  printf("Chess: %i\n", chessTexture);
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(100.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);

  vehicle = new Formel1(odeHandle);
 // vehicle->setTextures(DS_WOOD, chessTexture);
  vehicle->place(Position(robotPoint[0],robotPoint[1],robotPoint[2]));

  // Controller
//AbstractController *controller = new InvertNChannelController(10);
  controller = new SimpleController();

  // Wiring for OdeAgent
  One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

  // OdeAgent for connecting Controller, Robot and Wiring
  OdeAgent* agent = new OdeAgent(global);
  agent->init(controller, vehicle, wiring);
  global.agents.push_back(agent);

  // Simulation-Configuration
    configs.push_back(controller);
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

// gets key from keyboard input, provided by ode
void command(const OdeHandle& odeHandle, GlobalData& global, int key){
    double val;
//     double velocityStep=0.05;
    double maxShift=0.5;
    double maxVelocity=2.5f;
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


int main (int argc, char **argv)
{
  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config,&command);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}

