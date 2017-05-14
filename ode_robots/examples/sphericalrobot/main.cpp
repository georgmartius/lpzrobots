/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h> // arena
#include <ode_robots/passivesphere.h>  // passive balls

// controller
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>

#include <selforg/noisegenerator.h> // include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/one2onewiring.h>  // simple wiring

// robots
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/axisorientationsensor.h>

#include <ode_robots/axisorientationsensor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

bool track=false; // whether to track the robot (set by cmdline parameter)

class ThisSim : public Simulation {
public:
  AbstractController* controller;
  OdeRobot* robot;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set global noise sensor to 0.05
    global.odeConfig.setParam("noise",0.05);
    //  global.odeConfig.setParam("gravity", 0); // no gravity

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting initial position of the playground: setPosition(osg::Vec3(double x, double y, double z))
    // - push playground to the global list of obstacles (global list comes from simulation.cpp)
    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(10, 0.2, 1), 12);
    playground->setPosition(osg::Vec3(0,0,0)); // place and create playground
    global.obstacles.push_back(playground);

    // add passive spheres as obstacles
    // - create pointer to sphere (with odehandle, osghandle and
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere
    // - add sphere to list of obstacles
    for(int i=0; i<8; i++){
      PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5);
      s->setPosition(osg::Vec3(5,0,i*3));
      global.obstacles.push_back(s);
    }


    // Spherical Robot with axis (gyro) sensors:
    // - get default configuration for robot
    // - create pointer to spherical robot (with odeHandle, osgHandle and configuration)
    // - place robot (unfortunatelly there is a type cast necessary, which is not quite understandable)
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
    // this could now be done in a generic fashion by adding sensors to robots directly
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    conf.diameter=1.0;
    conf.pendularrange= 0.35;
    robot = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)),
                                       conf, "Spherical", 0.2);
    (robot)->place ( Pos( 0 , 0 , 0.1 ));

    // Selforg - Controller (Note there are several: use Sos or Sox now)
    // create pointer to controller
    // set some parameters
    // push controller in global list of configurables
    controller = new InvertMotorSpace(10);
    controller->setParam("epsA",0.05); // model learning rate
    controller->setParam("epsC",0.2); // controller learning rate
    controller->setParam("rootE",3);    // model and contoller learn with square rooted error
    global.configs.push_back ( controller );

    // SineController (produces just sine waves)
    // controller = new SineController();
    // controller->setParam("sinerate", 40);
    // controller->setParam("phaseshift", 0.0);

    // create pointer to one2onewiring which uses colored-noise
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent ( global );
    agent->init ( controller , robot , wiring );
    if(track){
      // the following line will enables a position tracking of the robot, which is written into a file
      // you can customize what is logged with the TrackRobotConf
      TrackRobotConf tc = TrackRobot::getDefaultConf();
      tc.scene = "zaxis";
      tc.displayTrace = true;
      agent->setTrackOptions(TrackRobot(tc));
    }
    global.agents.push_back ( agent );
  }

  /** is called if a key was pressed.
      For keycodes see: osgGA::GUIEventAdapter
      @return true if the key was handled
  */
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData,
                       int key, bool down) {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key ) {
      case 'T' : robot->getMainPrimitive()->applyTorque(0 , 0 , 3 ); break;
      case 't' : robot->getMainPrimitive()->applyTorque(0 , 0 , -3 ); break;
      default:
        return false;
      }
      return true;
    } else return false;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Simulation: T","Spin robot counter-clockwise");
    au.addKeyboardMouseBinding("Simulation: t","Spin robot clockwise");
  }

  virtual void usage() const {
    printf("\t-track\tenable tracking of the position of the robot\n");
  }
};

int main (int argc, char **argv)
{
  ThisSim sim;
  track = sim.contains(argv, argc, "-track"); // check whether cmd-line contains -track
  return sim.run(argc, argv) ? 0 : 1;
}
