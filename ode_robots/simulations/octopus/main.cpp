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
 *   Revision 1.1.2.6  2006-05-15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.1.2.5  2006/05/10 13:22:29  robot8
 *   -splitting of the component system to SimpleComponent and RobotComponent
 *   -add ing the possibility to read and write sensors and motors form TwoAxis Joints as connecting joints between components
 *   -octopus adopted to new splitted system
 *
 *   Revision 1.1.2.4  2006/05/10 09:36:10  robot8
 *   okctopus system working correctly
 *
 *   Revision 1.1.2.3  2006/05/09 13:07:47  robot8
 *   new component system, a bit less complex
 *   easy to use, because of only one component class
 *   handling like a normal robot
 *   testet, OK
 *   not handling multi-joints between the components
 *
 *   Revision 1.1.2.2  2006/05/09 11:20:56  robot8
 *   robot with n arm, simulating an octopus
 *   functional, but not finished, snakes have to be changed, so they could rotate their position
 *
 *   Revision 1.1.2.1  2006/05/03 13:11:20  robot8
 *   test robot for the new component system
 *   only with a lot of spheres at the moment
 *
 *   Revision 1.1.2.4  2006/05/02 12:24:32  robot8
 *   new component system, a bit less complex
 *   easy to use, because of only one component class
 *   handling like a normal robot
 *   testet, seams functional
 *   template with two spheres working
 *
 *   Revision 1.1.2.3  2006/04/27 11:44:58  robot8
 *   new component system, a bit less complex
 *   easy to use, because of only one component class
 *   handling like a normal robot
 *   untested
 *
 *   Revision 1.1.2.1  2006/04/25 13:51:01  robot8
 *   new component system, a bit less complex
 *   easy to use, because of only one component class
 *   handling like a normal robot
 *   not functional now
 *
 ***************************************************************************/

#include "simulation.h"

#include "odeagent.h"
#include "octaplayground.h"
#include "passivesphere.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include "sphererobot.h"
#include "schlangeservo.h"
#include "sphererobot3masses.h"

#include "component.h"
#include "simplecomponent.h"
#include "robotcomponent.h"

#define MAX_NUMBER_OF_ARMS 8
#define HEAD_ARM_DISTANCE 1.5


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


AbstractController* controller;





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
    //  global.odeConfig.setParam("gravity", 0);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the 
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground: 
    //   setGeometry(double length, double width, double	height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)
    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(10, 0.2, 1), 12);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    //****************

    Primitive* sphere;
    vector <OdeRobot*> arms;

    vector <Component*> components;
    
//sphere1 
    sphere = new Sphere ( 0.2 );   
    sphere->init ( odeHandle , 1 , osgHandle , Primitive::Body | Primitive::Geom | Primitive::Draw);

    ComponentConf cConf = Component::getDefaultConf ();
    cConf.max_force = 1;

    components.push_back ( new SimpleComponent ( odeHandle , osgHandle , cConf ) );

    ((SimpleComponent*) components.back ())->setSimplePrimitive ( sphere );

    components.back ()->place ( Pos( 0 , 0 , 0.2 ));
   
//arms
    DerivativeWiringConf c;
    DerivativeWiring* wiring;
    OdeAgent* agent;

    HingeJoint* j1;
    Axis axis;

    for ( int n = 0; n < MAX_NUMBER_OF_ARMS; n++ )
    {
	SchlangeConf sc = Schlange::getDefaultConf ();
	sc.segmNumber = 3;
	sc.segmLength = 0.4;
	sc.segmMass = 0.1;
	sc.motorPower = 0.5;
	sc.frictionJoint=0.01;

	arms.push_back ( new SchlangeServo ( odeHandle , osgHandle , sc ,  "octopusarm" ) );

	((OdeRobot*)arms[n])->place ( osg::Matrix::rotate ( ((4*M_PI/MAX_NUMBER_OF_ARMS)+(-2*M_PI/MAX_NUMBER_OF_ARMS)*(n)), osg::Vec3 (0,0,1)) * 
				      osg::Matrix::translate ( sin ( (double) n*M_PI*2/MAX_NUMBER_OF_ARMS )*HEAD_ARM_DISTANCE,
							       cos ( (double) n*M_PI*2/MAX_NUMBER_OF_ARMS )*HEAD_ARM_DISTANCE,
								0 ));

	InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
	cc.cInit=2;

	AbstractController *controller = new InvertMotorNStep ( cc );  
	controller->setParam("adaptrate",0.005);
	controller->setParam("epsC",0.001);
	controller->setParam("epsA",0.001);
	controller->setParam("rootE",1);
	controller->setParam("s4avg",10);
	controller->setParam("steps",2);

	c = DerivativeWiring::getDefaultConf ();
	wiring = new DerivativeWiring ( c , new ColorUniformNoise(0.1) );

	agent = new OdeAgent ( plotoptions );
	agent->init(controller , arms[n] , wiring );
	global.agents.push_back(agent);
	global.configs.push_back(controller);


	//Components
	components.push_back ( new RobotComponent ( odeHandle , osgHandle , cConf ) );
	((RobotComponent*) components.back ())->setRobot ( global.agents.back()->getRobot () );
	
	//creating joint
	vector <Position> positionlist;
	((Schlange*) ((RobotComponent*) components.back ())->getRobot ())->getSegmentsPosition ( positionlist );

	//schlange has no member getPosition, only the abilitie to give its whole object-list of Primitives, and here the first Primitive is used, to get the Position
//	components.front ()->getRobot ()->getPosition () - positionlist.front ();//

	axis = Axis ( ( components.front ()->getPosition () - positionlist.front ()).toArray() );
	//axis = Axis ( ( components.front ()->getRobot ()->getPosition () - components.back ()->getRobot()->getPosition ()).toArray() );
	
	
	j1 = new HingeJoint ( components.front ()->getMainPrimitive () , components.back ()->getMainPrimitive () , components.front ()->getPositionbetweenComponents ( components.back () ) , axis );
	j1->init ( odeHandle , osgHandle , true , 1 );
//	j1->setParam(dParamFMax,0.0001);            // set maximal force
	components.front ()->addSubcomponent ( components.back () , j1 );

	positionlist.clear ();
    }


//adding the controller for the component-connections
//    controller = new InvertMotorSpace ( 10 );  
    controller = new InvertMotorNStep ();  
    controller->setParam("adaptrate", 0.005);
    controller->setParam("epsC", 0.005);
    controller->setParam("epsA", 0.001);
    controller->setParam("rootE", 0);
    controller->setParam("steps", 2);
    controller->setParam("s4avg", 5);
    controller->setParam("factorB",0);

    wiring = new DerivativeWiring ( c , new ColorUniformNoise() );   

    agent = new OdeAgent ( plotoptions );
    agent->init ( controller , components.front () , wiring );
    global.agents.push_back ( agent );
    global.configs.push_back ( controller );


    showParams(global.configs);
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
