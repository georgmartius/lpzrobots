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
 *   Revision 1.11  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.10  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.9  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.8  2006/07/14 12:23:52  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.7.4.2  2006/06/25 17:01:56  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.7.4.1  2005/11/15 12:29:57  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.7  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/

/*
  gelaufen mit:
  invertnchannelcontroller.h   CVS-1.9
  invertnchannelcontroller.hpp CVS-1.10
  fixedsnake2elements.h        CVS-1.1
  fixedsnake2elements.cpp      CVS-1.1
*/


#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode-dbl/ode.h>

#include <selforg/noisegenerator.h>
#include <ode_robots/simulation.h>
#include <selforg/one2onewiring.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/playground.h>

#include <selforg/invertnchannelcontroller.h>

ConfigList configs;

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
  playground->setGeometry(20.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);
  
  /*
  Nimm2* vehicle = new Nimm2(odeHandle);
  Position p = {5,0,0};
  vehicle->place(p);
  AbstractController *controller = new InvertNChannelController(10);  
  configs.push_back(controller);

  One2OneOdeAgent* agent = new One2OneOdeAgent(global,NoPlot); 
  agent->init(controller, vehicle);
  global.agents.push_back(agent);
  */

  FixedSnake2Elements* testjoints = new FixedSnake2Elements(odeHandle);
  testjoints->place(Position(0,0,0));
  AbstractController *controller2 = new InvertNChannelController(10);  
  configs.push_back(controller2);
  
  One2OneWiring* wiring2 = new One2OneWiring(new ColorUniformNoise());
  OdeAgent* agent2 = new OdeAgent(/*NoPlot*/GuiLogger);
  agent2->init(controller2, testjoints, wiring2);
  global.agents.push_back(agent2);
  
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


int main (int argc, char **argv)
{  
  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
