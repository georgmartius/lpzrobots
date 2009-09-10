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
 *   Revision 1.1  2009-09-10 10:25:01  fhesse
 *   initial and preleminary version
 *
 *   Revision 1.20  2008/11/14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.19  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.18  2007/01/26 12:07:08  martius
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
#include <ode_robots/hand.h>

//#include <ode_robots/sphererobot3masses.h>
//#include <ode_robots/axisorientationsensor.h>
//
//#include <ode_robots/axisorientationsensor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  AbstractController* controller;
  OdeRobot* hand;
//  Sphererobot3Masses* sphere1;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set global noise to 0.1
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity", 0); // no gravity

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

    HandConf conf = Hand::getDefaultConf();
    conf.velocity = 0.02;
    conf.irRange = 1.0; //0.7;
    conf.set_typ_of_motor = Without_servo_motor;//With_servo_motor;
    conf.show_contacts = true;
    conf.ir_sensor_used =false;//true;
    conf.irs_at_fingertip =true;
    conf.irs_at_fingertop =false;
    conf.irs_at_fingercenter =false;
    conf.irs_at_fingerbottom =false;
    conf.servo_motor_Power = 1.2;
    conf.fix_palm_joint=true;
    conf.one_finger_as_one_motor=true;
    conf.draw_joints=false;//true;
    conf.showFingernails=false;
    conf.fingerJointBendAngle=M_PI*2/5;
    conf.initWithOpenHand=true;
    hand = new Hand(odeHandle, osgHandle,conf,"Hand");
    hand->setColor(Color(1.0,0.5,1.0));
    hand->place(Pos(0,0,6));
    global.configs.push_back ( hand );

    // Spherical Robot with axis (gyro) sensors:
    // - get default configuration for robot
    // - create pointer to spherical robot (with odeHandle, osgHandle and configuration)
    // - place robot (unfortunatelly there is a type cast necessary, which is not quite understandable)
//    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
//    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
//    // regular behaviour
//    conf.motorsensor=false;
//    conf.diameter=1.0;
//    conf.pendularrange= 0.2;
//     conf.diameter=1.0;
//     conf.pendularrange= 0.35;
//    sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)),
//				       conf, "Sphere1", 0.2);
//    ((OdeRobot*)sphere1)->place ( Pos( 0 , 0 , 0.1 ));
//    global.configs.push_back ( sphere1 );

    // Selforg - Controller
    // create pointer to controller
    // set some parameters
    // push controller in global list of configurables
//    controller = new InvertMotorSpace(10);
//    controller->setParam("epsA",0.3); // model learning rate
//    controller->setParam("epsC",0.3); // controller learning rate
//    controller->setParam("rootE",3);    // model and contoller learn with square rooted error
//    global.configs.push_back ( controller );

    //SineController (produces just sine waves)
    controller = new SineController();  
    controller->setParam("sinerate", 40);  
    controller->setParam("phaseshift", 0.0);
    global.configs.push_back ( controller );
    
    // create pointer to one2onewiring which uses colored-noise 
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );

    // create pointer to agent (plotoptions is provided by Simulation (generated from cmdline options)
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents    
    OdeAgent* agent = new OdeAgent ( plotoptions );
    agent->init ( controller , hand , wiring );
    // the following line will enable a position tracking of the robot, which is written into a file
    //agent->setTrackOptions(TrackRobot(true, true, true, false, "Sphere_zaxis", 20));
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
//      case 'X' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
//      case 'x' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
//      case 'T' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , 3 ); break;
//      case 't' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , -3 ); break;
////       case 'S' : controller->setParam("sineRate", controller->getParam("sineRate")*1.2);
//// 	printf("sineRate : %g\n", controller->getParam("sineRate"));
////       break;
////       case 's' : controller->setParam("sineRate", controller->getParam("sineRate")/1.2);
//// 	printf("sineRate : %g\n", controller->getParam("sineRate"));
//// 	break;
      default:
	return false;	
      }
      return true;
    } else return false;
  }  

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
//    au.addKeyboardMouseBinding("Simulation: X","Push robot to right (positive x)");
//    au.addKeyboardMouseBinding("Simulation: x","Push robot to left (negative x)");
//    au.addKeyboardMouseBinding("Simulation: T","Spin robot counter-clockwise");
//    au.addKeyboardMouseBinding("Simulation: t","Spin robot clockwise");
//    //    au.addKeyboardMouseBinding("Controller: S","Increase sine frequency");
//    //    au.addKeyboardMouseBinding("Controller: s","Decrease sine frequency");
  }

};

int main (int argc, char **argv)
{ 
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
 
 
