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
 *   Revision 1.6.4.3  2005-12-21 17:40:11  martius
 *   moved to OSG
 *
 *   Revision 1.6.4.2  2005/11/16 11:27:38  martius
 *   invertmotornstep has configuration
 *
 *   Revision 1.6.4.1  2005/11/15 12:29:44  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.6  2005/11/14 12:49:49  martius
 *   new paramters and global config
 *
 *   Revision 1.5  2005/11/09 13:39:41  fhesse
 *   GPL added
 *                                                                 *
 *                                                                         * 
 ***************************************************************************/
#include <stdio.h>

#include <selforg/noisegenerator.h>
#include "simulation.h"
#include "odeagent.h"
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include "nimm2.h"
#include "playground.h"

#include "hurlingsnake.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>

using namespace lpzrobots;

list<PlotOption> plotoptions;

class ThisSim : public Simulation {
public:

  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){

    setCameraHomePos(Pos(30.745, 0.348354, 16.9263),  Pos(91.5938, -30.9927, 0));

    // initialization
    global.odeConfig.noise=0.05;
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("gravity",0);

  
    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(20.0, 0.2, 1.0));
    playground->setPosition(Pos(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    //   Nimm2* vehicle = new Nimm2(odeHandle);
    //   Position p = {3,3,0};
    //   vehicle->place(p);
    //   AbstractController *controller = new InvertNChannelController(10);  
  
    //   One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    //   OdeAgent* agent = new OdeAgent(NoPlot/*plotoptions*/);
    //   agent->init(controller, vehicle, wiring);
    //   global.agents.push_back(agent);
    //   configs.push_back(controller);


    HurlingSnake* hs;
    AbstractController *controller; 
    AbstractWiring* wiring;
    OdeAgent* agent;

    for (int i=0; i<3; i++){
      hs = new HurlingSnake(odeHandle, osgHandle);
      Color col;
      if (i==0) col=Color(2,2,0);
      if (i==1) col=Color(0,2,0);
      if (i==2) col=Color(0,2,2);
      ((OdeRobot* )hs)->place(Pos(-5,(i-1)*5,0.0));

      //controller = new InvertMotorSpace(10);  
      controller = new InvertMotorNStep();  
      controller->setParam("steps", 2);
      // controller->setParam("epsA", 0.15);
      // controller->setParam("epsC", 0.04);
      controller->setParam("adaptrate", 0.001);
      controller->setParam("nomupdate", 0.001);

      // controller = new SineController();
      //    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.blindMotorSets=0;
      c.useId = true;
      //    c.useFirstD = true;
      c.derivativeScale = 20;
      wiring = new DerivativeWiring(c, new ColorUniformNoise(0.05));
      if (i==0) agent = new OdeAgent(plotoptions);
      else  agent = new OdeAgent(NoPlot);
      agent->init(controller, hs, wiring);
      global.agents.push_back(agent);
    

    
      global.configs.push_back(controller);
      global.configs.push_back(hs);
    

    }
  
    //   ///////////////////////
    //   hs = new HurlingSnake(odeHandle);
    //   Position p4 = {1,1,0.3};
    //   hs->place(p4);
    //   //AbstractController *controller2 = new InvertNChannelController(10);  
    //   controller2 = new InvertMotorSpace(10);  
  
    //   wiring2 = new One2OneWiring(new ColorUniformNoise(0.1));
    //   agent2 = new OdeAgent(plotoptions);
    //   agent2->init(controller2, hs, wiring2);
    //   global.agents.push_back(agent2);

    //   configs.push_back(controller2);
    //   configs.push_back(hs);
    //   controller2->setParam("rho", 0.1);



    showParams(global.configs);
  }

  /// end() is called at the end and should tidy up
  virtual void end(GlobalData& global){
    for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++){
      delete (*i);
    }
    global.obstacles.clear();
    for(OdeAgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++){
      delete (*i)->getRobot();
      delete (*i)->getController();
      delete (*i)->getWiring();
      delete (*i);
    }
    global.agents.clear();
  }

};

void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile", progname);
}

int main (int argc, char **argv)
{  
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv, argc, "-l")) plotoptions.push_back(PlotOption(GuiLogger_File));
  if(contains(argv, argc, "-h")) printUsage(argv[0]);
  
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
