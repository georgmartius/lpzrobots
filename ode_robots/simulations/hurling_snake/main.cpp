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
 *   Revision 1.12  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.11  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.10  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.9  2006/09/21 22:10:42  martius
 *   make opt fixed
 *
 *   Revision 1.8  2006/07/14 15:13:15  fhesse
 *   minor changes
 *
 *   Revision 1.6.4.8  2006/06/25 17:01:54  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.6.4.7  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.6.4.6  2006/03/31 16:11:19  fhesse
 *   tracing via trackrobot
 *
 *   Revision 1.6.4.5  2006/03/28 14:18:38  fhesse
 *   head traced
 *
 *   Revision 1.6.4.4  2005/12/29 16:50:22  martius
 *   end is obsolete
 *
 *   Revision 1.6.4.3  2005/12/21 17:40:11  martius
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
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/playground.h>

#include <ode_robots/hurlingsnake.h>

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>

using namespace lpzrobots;

class ThisSim : public Simulation {
public:

  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){

    setCameraHomePos(Pos(30.745, 0.348354, 16.9263),  Pos(91.5938, -30.9927, 0));

    // initialization
    global.odeConfig.noise=0.05;
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("gravity",-9);


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

    for (int i=0; i<1; i++){
      hs = new HurlingSnake(odeHandle, osgHandle, "Hurling");
      Color col;
      if (i==0) col=Color(2,2,0);
      if (i==1) col=Color(0,2,0);
      if (i==2) col=Color(0,2,2);
      ((OdeRobot* )hs)->place(Pos(-5,(i-1)*5,0.0));

      hs->setParam("factorForce",4);
      hs->setParam("frictionGround",0.05);

      //controller = new InvertMotorSpace(10);
      controller = new InvertNChannelController(20);
      //      controller = new InvertMotorNStep();
      //      controller->setParam("steps", 2);
      controller->setParam("factorA", 0.4);
      controller->setParam("epsA", 0.15);
      controller->setParam("eps", 0.05);
      controller->setParam("adaptrate", 0.000);
      controller->setParam("nomupdate", 0.000);

      // controller = new SineController();
      //    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.blindMotors=0;
      c.useId = true;
      //    c.useFirstD = true;
      c.derivativeScale = 20;
      wiring = new DerivativeWiring(c, new ColorUniformNoise(0.05));
      if (i==0) agent = new OdeAgent(global);
      else  agent = new OdeAgent(global,NoPlot);
      agent->init(controller, hs, wiring);
      // enable tracing of head element
      //agent->setTrackOptions(TrackRobot(false, false, false, true, "0", 1));
      // setting trace length and trace thickness
      // agent->init_tracing(1000, 0.008);

      // track position (plot position in file)
      agent->setTrackOptions(TrackRobot(true, false, false, false, "0"));

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
    //   agent2 = new OdeAgent(global);
    //   agent2->init(controller2, hs, wiring2);
    //   global.agents.push_back(agent2);

    //   configs.push_back(controller2);
    //   configs.push_back(hs);
    //   controller2->setParam("rho", 0.1);




  }

  // add own key handling stuff here, just insert some case values
virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        default:
          return false;
          break;
        }
    }
    return false;
  }

};

int main (int argc, char **argv)
{
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}

