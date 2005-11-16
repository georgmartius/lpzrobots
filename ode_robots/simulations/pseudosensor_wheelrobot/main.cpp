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
 *   Revision 1.13.4.2  2005-11-16 11:27:38  martius
 *   invertmotornstep has configuration
 *
 *   Revision 1.13.4.1  2005/11/15 12:29:54  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.13  2005/11/09 14:08:48  martius
 *   *** empty log message ***
 *
 *   Revision 1.12  2005/11/09 13:43:52  fhesse
 *   GPL added
 *                                                                 *
 *                                                                         * 
***************************************************************************/
#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include <selforg/noisegenerator.h>
#include "simulation.h"
#include "odeagent.h"
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include "nimm4.h"
#include "playground.h"

#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>

list<PlotOption> plotoptions;

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //  dWorldSetGravity ( world , 0 , 0 ,-9.81 );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {5.8f,-6.1f,4.0f};
  float KameraViewXYZ[3] = {125.5000f,-22.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.noise=0.1;

  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(20.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren 
  global.obstacles.push_back(playground);

  OdeRobot* vehicle = new Nimm4(odeHandle,1,3,15);

  vehicle->place(Position(0,0,0));
  AbstractController *controller = new InvertMotorNStep();   
  // AbstractController *controller = new InvertMotorNStep(10);  
  // AbstractController *controller = new InvertMotorSpace(10);  
  //  controller->setParam("factorB",0);
  //  controller->setParam("eps",0.5);
  //AbstractController *controller = new SineController();  
  //controller->setParam("phaseShift",2);
  DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
  controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
  c.useId=true;
  c.useFirstD=false;
  c.derivativeScale=10;
  c.eps=0.1;
  // c.blindMotorSets=1;
  AbstractWiring* wiring = new DerivativeWiring(c, new ColorUniformNoise(0.1));
  //AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  OdeAgent* agent = new OdeAgent(plotoptions);
  agent->init(controller, vehicle, wiring);
  global.agents.push_back(agent);
  
  global.configs.push_back(controller);
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


// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){
  changeParams(global.configs);
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
 
