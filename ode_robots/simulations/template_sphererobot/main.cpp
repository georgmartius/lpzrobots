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
 *   Revision 1.18  2007-01-26 12:07:08  martius
 *   orientationsensor added
 *
 *   Revision 1.17  2006/08/04 15:07:47  martius
 *   documentation
 *
 *   Revision 1.16  2006/07/14 12:23:54  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.15.4.5  2006/05/15 13:11:30  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.15.4.4  2006/02/20 10:50:20  martius
 *   pause, random, windowsize, Ctrl-keys
 *
 *   Revision 1.15.4.3  2006/01/12 15:17:39  martius
 *   *** empty log message ***
 *
 *   Revision 1.15.4.2  2006/01/10 20:33:50  martius
 *   moved to osg
 *
 *   Revision 1.15.4.1  2005/11/15 12:30:17  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.15  2005/11/09 14:54:46  fhesse
 *   nchannelcontroller used
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/

// include simulation environment stuff
#include "simulation.h"

// include agent (class for holding a robot, a controller and a wiring)
#include "odeagent.h"
#include "octaplayground.h" // arena
#include "passivesphere.h"  // passive balls

// controller
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>

#include <selforg/noisegenerator.h> // include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/one2onewiring.h>  // simple wiring

// robots
#include "sphererobot3masses.h"
#include "axisorientationsensor.h"

#include "axisorientationsensor.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  AbstractController* controller;
  Sphererobot3Masses* sphere1;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set global noise to 0.1
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity", 0); // no gravity

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the 
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting initial position of the playground: setPosition(osg::Vec3(double x, double y, double z))
    // - push playground to the global list of obstacles (global list comes from simulation.cpp)
    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(10, 0.2, 1), 12);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
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
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    conf.diameter=1.0;
    conf.pendularrange= 0.35;
    sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)), 
				       conf, "Sphere1", 0.2);     
    ((OdeRobot*)sphere1)->place ( Pos( 0 , 0 , 0.1 ));

    // Selforg - Controller
    // create pointer to controller
    // set some parameters
    // push controller in global list of configurables
    controller = new InvertMotorSpace(10);  
    controller->setParam("epsA",0.05); // model learning rate
    controller->setParam("epsC",0.2); // controller learning rate
    controller->setParam("rootE",3);    // model and contoller learn with square rooted error
    global.configs.push_back ( controller );

    //  SineController (produces just sine waves)
    // controller = new SineController();  
    // controller->setParam("sinerate", 40);  
    // controller->setParam("phaseshift", 0.0);
    
    // create pointer to one2onewiring which uses colored-noise 
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );

    // create pointer to agent (plotoptions is provided by Simulation (generated from cmdline options)
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents    
    OdeAgent* agent = new OdeAgent ( plotoptions );
    agent->init ( controller , sphere1 , wiring );
    // the following line will enable a position tracking of the robot, which is written into a file
    // agent->setTrackOptions(TrackRobot(true, false, false, "Sphere_zaxis", 50)); 
    global.agents.push_back ( agent );
      
    // display all parameters of all configurable objects on the console
    showParams(global.configs);
  }

  /** is called if a key was pressed. 
      For keycodes see: osgGA::GUIEventAdapter
      @return true if the key was handled
  */
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, 
		       int key, bool down) { 
    if (down) { // only when key is pressed, not when released
      switch ( (char) key ) {
      case 'X' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
      case 'x' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
      case 'T' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , 3 ); break;
      case 't' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , -3 ); break;
//       case 'S' : controller->setParam("sineRate", controller->getParam("sineRate")*1.2); 
// 	printf("sineRate : %g\n", controller->getParam("sineRate"));
//       break;
//       case 's' : controller->setParam("sineRate", controller->getParam("sineRate")/1.2); 
// 	printf("sineRate : %g\n", controller->getParam("sineRate"));
// 	break;      
      default:
	return false;	
      }
      return true;
    } else return false;
  }  

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Simulation: X","Push robot to right (positive x)");
    au.addKeyboardMouseBinding("Simulation: x","Push robot to left (negative x)");
    au.addKeyboardMouseBinding("Simulation: T","Spin robot counter-clockwise");
    au.addKeyboardMouseBinding("Simulation: t","Spin robot clockwise");
    //    au.addKeyboardMouseBinding("Controller: S","Increase sine frequency");
    //    au.addKeyboardMouseBinding("Controller: s","Decrease sine frequency");
  }

};

int main (int argc, char **argv)
{ 
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
 
 
