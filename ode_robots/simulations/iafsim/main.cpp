/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *   Revision 1.8  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.7  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.6  2008/05/05 13:42:13  guettler
 *   new iafcontroller tested
 *
 *   Revision 1.5  2008/05/05 06:26:47  guettler
 *   tristateiafcontroller moved to ode_robots/simulations/iafsim
 *   for test purposes
 *
 *   Revision 1.4  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.3  2008/04/30 14:57:57  guettler
 *   improvements testet
 *
 *   Revision 1.2  2008/04/29 15:30:42  guettler
 *   yippie
 *
 *   Revision 1.1  2008/04/28 10:30:39  guettler
 *   some experiments
 *
 *                                                                         *
 ***************************************************************************/
#include <stdio.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include ode library
//#include <ode-dbl/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
//#include <selforg/derivativewiring.h>

// used robot
#include <ode_robots/nimm2.h>
//#include <ode_robots/nimm4.h>


// used arena
#include <ode_robots/playground.h>
// used passive spheres
//#include <ode_robots/passivesphere.h>

// used controller
#include "tristateiafcontroller.h"
#include <selforg/abstractiafcontroller.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // first: position(x,y,z) second: view(alpha,beta,gamma)
    // gamma=0;
    // alpha == horizontal angle
    // beta == vertical angle
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set noise to 0.1
    global.odeConfig.noise=0.05;
    global.odeConfig.setParam("realtimefactor",1);
    //  global.odeConfig.setParam("gravity", 0);

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double        height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)

    // odeHandle and osgHandle are global references
    // vec3 == length, width, height
    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(32, 0.2, 0.5));
    playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
    // register playground in obstacles list
    global.obstacles.push_back(playground);

    // add passive spheres as obstacles
    // - create pointer to sphere (with odehandle, osghandle and
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere
    // - set a texture for the sphere
    // - add sphere to list of obstacles
//     for (int i=0; i < 0/*2*/; i++){
//       PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
//       s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
//       s1->setTexture("Images/dusty.rgb");
//       global.obstacles.push_back(s1);
//     }

    // use Nimm2 vehicle as robot:
    // - get default configuration for nimm2
    // - activate bumpers, cigar mode and infrared front sensors of the nimm2 robot
    // - create pointer to nimm2 (with odeHandle, osg Handle and configuration)
    // - place robot
    Nimm2Conf c = Nimm2::getDefaultConf();
    c.force   = 4;
/*    c.bumper  = true;
    c.cigarMode  = true;*/
    // c.irFront = true;
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, c, "Nimm2");
    vehicle->place(Pos(0,0,0));

    // use Nimm4 vehicle as robot:
    // - create pointer to nimm4 (with odeHandle and osg Handle and possible other settings, see nimm4.h)
    // - place robot
    //OdeRobot* vehicle = new Nimm4(odeHandle, osgHandle, "Nimm4");
    //vehicle->place(Pos(0,1,0));


    // create pointer to controller
    // push controller in global list of configurables
   //  AbstractController *controller = new InvertNChannelController(10);
//     AbstractController *controller = new SineController();
//     TristateIAFControllerConf iafc = TristateIAFController::getDefaultConf();
    AbstractIAFControllerConf iafc = AbstractIAFController::getDefaultConf();
    *iafc.numberIAFNeuronsPerInput=50;
    *iafc.numberIAFNeuronsPerOutput=50;
    *iafc.wIInitScale= 1.2;
    *iafc.wOInitScale= 1.2;
    AbstractController *controller = new AbstractIAFController(iafc);
    global.configs.push_back(controller);

    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise());

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);


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
  return sim.run(argc, argv) ? 0 : 1;

}

