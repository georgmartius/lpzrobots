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
 *   Revision 1.14.4.3  2005-12-06 10:13:25  martius
 *   openscenegraph integration started
 *
 *   Revision 1.14.4.2  2005/11/24 16:19:12  fhesse
 *   include corrected
 *
 *   Revision 1.14.4.1  2005/11/15 12:30:07  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>

// include ode library
#include <ode/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include "simulation.h"
// include compatibility simulation environmet stuff
#include "compatsim.h"

// include agent (class for holding a robot, a controller and a wiring)
#include "odeagent.h"

// used wiring
#include <selforg/one2onewiring.h>

// used robot
//#include "nimm4.h"

// used arena
#include "playground.h"

// used controller
#include <selforg/invertnchannelcontroller.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

// plotoptions is a list of possible online output, 
// if the list is empty no online gnuplot windows and no logging to file occurs.
// The list is modified with commandline options, see main() at the bottom of this file
list<PlotOption> plotoptions;


// starting function (executed once at the beginning of the simulation loop)
void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
{
  ///Todo: setviewpoint

  // initialization
  // - set noise to 0.1
  // - register file chess.ppm as a texture called chessTexture (used for the wheels)
  global.odeConfig.noise=0.1;
  //  int chessTexture = dsRegisterTexture("chess.ppm");

  // use Playground as boundary:
  // - create pointer to playground (odeHandle contains things like world and space the 
  //   playground should be created in; odeHandle is generated in simulation.cpp)
  // - setting geometry for each wall of playground: 
  //   setGeometry(double length, double width, double	height)
  // - setting initial position of the playground: setPosition(double x, double y, double z)
  // - push playground in the global list of obstacles(globla list comes from simulation.cpp)
  Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(10,0.2,0.5));
  playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
  global.obstacles.push_back(playground);

  // use Nimm4 vehicle as robot:
  // - create pointer to nimm4 (with ode Information, further parameters can be set, 
  //   here are the defaults used)
  // - set textures for body and wheels
  // - place robot
//   Nimm4* vehicle = new Nimm4(odeHandle);
//   vehicle->setTextures(DS_WOOD, chessTexture); 
//   vehicle->place(Position(0,0,0));

//   // create pointer to controller
//   // push controller in global list of configurables
//   AbstractController *controller = new InvertNChannelController(10);  
//   global.configs.push_back(controller);
  
//   // create pointer to one2onewiring
//   One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

//   // create pointer to agent
//   // initialize pointer with controller, robot and wiring
//   // push agent in globel list of agents
//   OdeAgent* agent = new OdeAgent(plotoptions);
//   agent->init(controller, vehicle, wiring);
//   global.agents.push_back(agent);
  
  showParams(global.configs);
}

// executed once at the end of the simulation loop
void end(GlobalData& global){
  // clear obstacles list
  for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++){
    delete (*i);
  }
  global.obstacles.clear();
  
  // clear agents list
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

// print command line options
void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile\n", progname);
}

int main (int argc, char **argv)
{ 
  // start with online windows (default: start without plotting and logging)
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger));
  
  // start with online windows and logging to file
  if(contains(argv, argc, "-l")) plotoptions.push_back(PlotOption(GuiLogger_File));
  
  // display help
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  CompatSim sim(&start, &end, &config);
  if(sim.run(argc, argv))
    return 0;
  else 
    return 1;

}
 
