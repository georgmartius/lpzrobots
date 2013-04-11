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
 *   Revision 1.5  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.4  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.3  2006/07/14 12:23:43  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.2.4.5  2006/06/25 17:01:54  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.2.4.4  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.2.4.3  2006/04/18 14:12:20  fhesse
 *   minor changes
 *
 *   Revision 1.2.4.2  2006/01/03 10:01:05  fhesse
 *   moved to osg
 *
 *   Revision 1.2.4.1  2005/11/15 12:29:35  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.2  2005/11/09 13:36:41  fhesse
 *   GPL added
 *
 *   Revision 1.7  2005/11/09 13:28:24  fhesse
 *   GPL added
 *                                                                *
 ***************************************************************************/
#include <stdio.h>

#include <selforg/noisegenerator.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>
#include <ode_robots/playground.h>

#include <ode_robots/arm2segm.h>

#include <selforg/invertnchannelcontroller.h>
#include <selforg/sinecontroller.h>

using namespace lpzrobots;

class ThisSim : public Simulation {

public:


  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(8.85341, -11.2614, 3.32813),  Pos(33.5111, -7.0144, 0));

    // initialization
    global.odeConfig.noise=0.1;

//     Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(7.0, 0.2, 1.5));
//     playground->setPosition(Pos(0,0,0)); // playground positionieren und generieren
//     global.obstacles.push_back(playground);

    Arm2SegmConf arm_conf=Arm2Segm::getDefaultConf();
    Arm2Segm* vehicle = new Arm2Segm(odeHandle, osgHandle, arm_conf, "arm");
    ((OdeRobot* )vehicle)->place(Position(0,0,0));
    //AbstractController *controller = new InvertNChannelController(10);
    AbstractController *controller = new SineController();
    global.configs.push_back(vehicle);

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);

    global.configs.push_back(controller);

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
