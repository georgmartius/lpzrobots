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
 *   Revision 1.15.4.1  2005-11-15 12:30:04  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.15  2005/11/14 12:50:21  martius
 *   *** empty log message ***
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
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
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>

#include "nimm2.h"
#include "shortcircuit.h"
#include "playground.h"

#include <selforg/sinecontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertnchannelcontroller.h>

int channels;
int t=0;
double omega = 0.05;

list<PlotOption> plotoptions;

SineWhiteNoise* sineNoise;

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird 
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Noise frequency modification with  < > , . r n\n" );
  dsPrint ( "Press Ctrl-C (on the console) for an basic commandline interface.\n\n" );


  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.setParam("noise",0.1);
  global.odeConfig.setParam("realtimefactor",0);
  global.odeConfig.setParam("drawinterval", 500);
  
  OdeRobot* robot = new ShortCircuit(odeHandle, channels, channels);  
  //  OdeRobot* robot = new Nimm2(odeHandle);  
  AbstractController *controller = new InvertMotorNStep(10);  
  //AbstractController *controller = new InvertMotorSpace(10,1.2);  
  //controller->setParam("nomupdate",0.001);
  controller->setParam("epsA",0.01);
  controller->setParam("epsC",0.01);
  controller->setParam("factorB",0);
  controller->setParam("steps",2);
  //  AbstractController *controller = new InvertNChannelController(10);  
  //AbstractController *controller = new SineController();
  
  OdeAgent* agent = new OdeAgent(plotoptions);
  
  //sineNoise = new SineWhiteNoise(omega,2,M_PI/2);
  //One2OneWiring* wiring = new One2OneWiring(sineNoise, true);
  One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise(), true);
 
  //AbstractWiring* wiring = new SelectiveOne2OneWiring(sineNoise, &select_firsthalf);
  // DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
//   c.useId=true;
//   c.useFirstD=true;
//   c.derivativeScale=20;
//   c.blindMotorSets=0;
//   AbstractWiring* wiring = new DerivativeWiring(c, new ColorUniformNoise(0.1));
  agent->init(controller, robot, wiring);
  global.agents.push_back(agent);
  
  global.configs.push_back(controller);
  showParams(global.configs);
}

// void addcallback (bool, bool){
//   t++;
//   sineNoise->setOmega(0.1*cos(t/100000.0));
// }

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

void command(const OdeHandle& odeHandle, GlobalData& global, int key){
  switch (key){
  case '>': omega+=0.05;
    break;
  case '<': omega-=0.05;
    break;
  case '.': omega+=0.005;
    break;
  case ',': omega-=0.005;
    break;
  case 'r': omega=0.05;
    break;
  case 'n': omega=0;
    break;
  }
  fprintf(stderr, "Omega: %g\n", omega);
  sineNoise->setOmega(omega);
}

void printUsage(const char* progname){
  printf("Usage: %s numchannels [-g] [-l]\n\tnumchannels\tnumber of channels\n\
\t-g\t\tuse guilogger\n\t-l\t\tuse guilogger with logfile", progname);
}

int main (int argc, char **argv)
{  
  if(argc <= 1){
    printUsage(argv[0]);  
    return -1;
  }
  channels = max(1,atoi(argv[1]));
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger, Robot));
  if(contains(argv, argc, "-l")) plotoptions.push_back(PlotOption(GuiLogger_File, Robot));
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config, &command);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
