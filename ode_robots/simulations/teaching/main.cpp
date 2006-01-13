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
 *   Revision 1.1.2.1  2006-01-13 12:24:06  martius
 *   env for external teaching input to the controller
 *
 *
 ***************************************************************************/

#include "simulation.h"

#include "odeagent.h"
#include "playground.h"
#include "passivesphere.h"

#include <selforg/invertmotornstep.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include "nimm2.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

// plotoptions is a list of possible online output, 
// if the list is empty no online gnuplot windows and no logging to file occurs.
// The list is modified with commandline options, see main() at the bottom of this file
list<PlotOption> plotoptions;

InvertMotorNStep*controller;
motor teaching[2];

class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.1;
    global.odeConfig.setParam("gravity", 0);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(10, 0.2, 1), 12);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    for(int i=0; i<3; i++){
      PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5);
      s->setPosition(osg::Vec3(5,0,i*3)); 
      global.obstacles.push_back(s);    
    }
    
    Nimm2Conf c = Nimm2::getDefaultConf();    
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, c);
    //OdeRobot* vehicle = new Nimm4(odeHandle, osgHandle);
    vehicle->place(Pos(0,0,0.6));

    // create pointer to controller
    // push controller in global list of configurables
    //  AbstractController *controller = new InvertNChannelController(10);      
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    controller = new InvertMotorNStep(cc);  
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(plotoptions);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(controller);
      
    showParams(global.configs);
  }

  //Funktion die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;    
    bool handled = false;
    switch ( key )
      {
      case 'y' : 
	teaching[0] = min(0.95, teaching[0]+0.1);
	teaching[1] = min(0.95, teaching[1]+0.1);
	controller->setMotorTeachingSignal(teaching, 2);
	printf("Teaching Signal: %f, %f\n", teaching[0], teaching[1]);
	handled = true; 
	break;
      case 'h' : 
	teaching[0] = max(-0.95, teaching[0]-0.1);
	teaching[1] = max(-0.95, teaching[1]-0.1);
	controller->setMotorTeachingSignal(teaching, 2);
	printf("Teaching Signal: %f, %f\n", teaching[0], teaching[1]);
	handled = true; 
	break;
      case 't' : 	
	controller->setTeachingMode(!controller->getTeachingMode());
	printf("Teaching Mode: %i\n", controller->getTeachingMode());
	teaching[0]=0.5;
	teaching[1]=0.5;
	handled = true; break;	
      }
    fflush(stdout);
    return handled;
  }

};

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

  ThisSim sim;
  return sim.run(argc, argv) ? 0 :  1;
}

 
 
