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
 *   Revision 1.15.4.4  2006-01-10 21:46:02  martius
 *   moved to osg
 *
 *   Revision 1.15.4.3  2005/12/06 17:38:19  martius
 *   *** empty log message ***
 *
 *   Revision 1.15.4.2  2005/11/16 11:27:38  martius
 *   invertmotornstep has configuration
 *
 *   Revision 1.15.4.1  2005/11/15 12:30:04  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.15  2005/11/14 12:50:21  martius
 *   *** empty log message ***
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include "simulation.h"

#include "odeagent.h"
#include "octaplayground.h"

#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>

#include <selforg/sinecontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertnchannelcontroller.h>

#include "shortcircuit.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

int channels;
int t=0;
double omega = 0.05;

list<PlotOption> plotoptions;

SineWhiteNoise* sineNoise;

class ThisSim : public Simulation {

public:
  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));

    // initialization
    global.odeConfig.setParam("noise",0.1);
    global.odeConfig.setParam("realtimefactor",0);
    global.odeConfig.setParam("drawinterval", 500);


    OdeRobot* robot = new ShortCircuit(odeHandle, osgHandle, channels, channels);  
    //  OdeRobot* robot = new Nimm2(odeHandle);  
    AbstractController *controller = new InvertMotorNStep();  
    //AbstractController *controller = new InvertMotorSpace(10,1.2);  
    //controller->setParam("nomupdate",0.001);
    controller->setParam("adaptrate",0.001);
    controller->setParam("epsA",0.01);
    controller->setParam("epsC",0.01);
    controller->setParam("factorB",0.1);
    //  controller->setParam("steps",2);
    //  AbstractController *controller = new InvertNChannelController(10);  
    //AbstractController *controller = new SineController();
    
    OdeAgent* agent = new OdeAgent(plotoptions);
    
    sineNoise = new SineWhiteNoise(omega,2,M_PI/2);
    One2OneWiring* wiring = new One2OneWiring(sineNoise, true);
    
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
  
};

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

  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
 
