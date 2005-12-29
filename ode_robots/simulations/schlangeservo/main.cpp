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
 *   Revision 1.2.4.2  2005-12-29 16:44:54  martius
 *   moved to osg
 *
 *   Revision 1.2.4.1  2005/11/15 12:29:59  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.2  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/


#include <selforg/noisegenerator.h>

#include "simulation.h"
#include "odeagent.h"
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include "playground.h"
#include "passivesphere.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>

#include "schlangeservo.h"

list<PlotOption> plotoptions;

using namespace lpzrobots;

class ThisSim : public Simulation {
public:
	

  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    setCameraHomePos(Pos(-10.7854, -7.41751, 5.92078),  Pos(-50.6311, -5.00218, 0));

    //****************/
    SchlangeConf conf = Schlange::getDefaultConf();
    conf.motorPower=0.3;
    conf.jointLimit=conf.jointLimit*3;
    SchlangeServo* schlange1 = 
      new SchlangeServo ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
			  conf, "S1");
    ((OdeRobot*)schlange1)->place(Pos(0,0,3)); 

    //AbstractController *controller = new InvertNChannelController(100/*,true*/);  
    //  AbstractController *controller = new InvertMotorSpace(100/*,true*/);  
    //AbstractController *controller = new InvertMotorNStep();  
    AbstractController *controller = new SineController();  
  
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    //   DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
    //   c.useId=true;
    //   c.useFirstD=true;
    //   // c.useSecondD=true;
    //   c.derivativeScale=10;
    //   AbstractWiring* wiring = new DerivativeWiring(c, new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(plotoptions);
    agent->init(controller, schlange1, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(controller);
    global.configs.push_back(schlange1);
  
 
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("gravity", 0.0); 


    //  controller->setParam("epsC",0.001);
    //   // controller->setParam("desens",0.0);
    //   controller->setParam("s4delay",1.0);
    //   controller->setParam("s4avg",1.0);
    //   controller->setParam("epsA",0.01);
    //   controller->setParam("factorB",0.0);
    //   controller->setParam("zetaupdate",0.1);

    showParams(global.configs);
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


 
  
