/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.4  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.3  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.2  2006/07/14 12:23:45  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.6  2006/05/29 20:28:43  robot5
 *   Annular placement of segments.
 *
 *   Revision 1.1.2.5  2006/05/22 14:19:11  robot5
 *   Added variable conf.firstJoint to represent the first slider type in the alternating sequence.
 *   Code cleaning.
 *
 *   Revision 1.1.2.4  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.1.2.3  2006/05/09 04:24:34  robot5
 *   *** empty log message ***
 *
 *   Revision 1.1.2.2  2006/04/25 09:04:37  robot3
 *   caterpillar is now represented by a box
 *
 *   Revision 1.1.2.1  2006/04/25 08:05:56  robot3
 *   created an own simulation for the caterpillar for testing purposes
 *
 ***************************************************************************/
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/invertmotornstep.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include <ode_robots/caterpillar.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)

    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("gravity:",1); // normally at -9.81
    // initialization

    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(25, 0.2, 1.5));
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    for(int i=0; i<5; i++){
      PassiveSphere* s =
        new PassiveSphere(odeHandle,
                          osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.2);
      s->setPosition(Pos(i*0.5-2, i*0.5, 1.0));
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);
    }

    OdeAgent* agent;
    AbstractWiring* wiring;
//    OdeRobot* robot;
    AbstractController *controller;

    CaterPillar* myCaterPillar;
    CaterPillarConf myCaterPillarConf = DefaultCaterPillar::getDefaultConf();
    //******* R A U P E  *********/
    myCaterPillarConf.segmNumber=6;
    myCaterPillarConf.jointLimit=M_PI/4;
    myCaterPillarConf.motorPower=0.8;
    myCaterPillarConf.frictionGround=0.04;
    myCaterPillar = new CaterPillar ( odeHandle, osgHandle.changeColor(Color(0.0f,1.0f,0.0f)), myCaterPillarConf, "Raupe1");
    ((OdeRobot*) myCaterPillar)->place(Pos(-5,-5,0.2));

    InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
    invertnconf.cInit=2.0;
    controller = new InvertMotorNStep(invertnconf);
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent = new OdeAgent( global, plotoptions );
    agent->init(controller, myCaterPillar, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(controller);
    global.configs.push_back(myCaterPillar);
    myCaterPillar->setParam("gamma",/*gb");
      global.obstacles.push_back(s)0.0000*/ 0.0);

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
