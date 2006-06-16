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
 *   Revision 1.1.2.6  2006-06-16 12:06:37  robot8
 *   - correcting some mistakes in component_template
 *
 *   Revision 1.1.2.5  2006/05/15 13:11:30  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
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

#include "simulation.h"

#include "odeagent.h"
#include "octaplayground.h"
#include "passivesphere.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include "sphererobot.h"
#include "sphererobot3masses.h"

#include "robotcomponent.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


AbstractController* controller;

Sphererobot3Masses* sphere;



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

    
//sphere1 
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    conf.irAxis1 = true;
    conf.irAxis2 = false;
    conf.irAxis3 = true;
    conf.drawIRs = true;

    sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0,0)), 
				       conf, "Sphere1", 0.2);   
    ((OdeRobot*)sphere)->place ( Pos( -1 , 0 , 0.1 ));

    controller = new InvertMotorSpace ( 10 );  
    controller->setParam("adaptrate", 0.000);
    controller->setParam("epsC", 0.005);
    controller->setParam("epsA", 0.001);
    controller->setParam("rootE", 0);
    controller->setParam("steps", 2);
    controller->setParam("s4avg", 5);
    controller->setParam("factorB",0);

    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );   

    OdeAgent* agent = new OdeAgent ( plotoptions );
    agent->init ( controller , sphere , wiring );
    // agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
    global.agents.push_back ( agent );
    global.configs.push_back ( controller );



//sphere2
    conf = Sphererobot3Masses::getDefaultConf();  
    conf.irAxis1 = true;
    conf.irAxis2 = false;
    conf.irAxis3 = true;
    conf.drawIRs = true;

    sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(0,1.0,0)), 
				       conf, "Sphere2", 0.2);   
    ((OdeRobot*)sphere)->place ( Pos( 1 , 0 , 0.1 ));

    controller = new InvertMotorSpace ( 10 );  
    controller->setParam("adaptrate", 0.000);
    controller->setParam("epsC", 0.005);
    controller->setParam("epsA", 0.001);
    controller->setParam("rootE", 0);
    controller->setParam("steps", 2);
    controller->setParam("s4avg", 5);
    controller->setParam("factorB",0);

    wiring = new One2OneWiring ( new ColorUniformNoise() );   

    agent = new OdeAgent ( plotoptions );
    agent->init ( controller , sphere , wiring );
    // agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
    global.agents.push_back ( agent );
    global.configs.push_back ( controller );
   
   
    //creating compoents for the two spheres
    RobotComponent* C1 = new RobotComponent ( odeHandle , osgHandle , Component::getDefaultConf () );
    RobotComponent* C2 = new RobotComponent ( odeHandle , osgHandle , Component::getDefaultConf () );

    //setting the spheres as robots for the components
    C1->setRobot ( global.agents[0]->getRobot () );
    C2->setRobot ( global.agents[1]->getRobot () );
    //externaly creating the connecting joint
    Axis axis = Axis ( ( C1->getRobot ()->getPosition () - C2->getRobot ()->getPosition ()).toArray() );


    HingeJoint* j1 = new HingeJoint ( C1->getMainPrimitive () , C2->getMainPrimitive () , C1->getPositionbetweenComponents ( C2 ) , axis );
    j1->init ( odeHandle , osgHandle , true , 1 );

    //connecting both components, and creating the new physical form of the robot
    C1->addSubcomponent ( C2 , j1 );

    //adding the controller for the component-connections
    controller = new InvertMotorSpace ( 10 );  
    controller->setParam("adaptrate", 0.000);
    controller->setParam("epsC", 0.005);
    controller->setParam("epsA", 0.001);
    controller->setParam("rootE", 0);
    controller->setParam("steps", 2);
    controller->setParam("s4avg", 5);
    controller->setParam("factorB",0);

    wiring = new One2OneWiring ( new ColorUniformNoise() );   

    agent = new OdeAgent ( plotoptions );
    agent->init ( controller , C1 , wiring );
    global.agents.push_back ( agent );
    global.configs.push_back ( controller );

    


    showParams(global.configs);
  }

//   //Funktion die eingegebene Befehle/kommandos verarbeitet
//   void command (const OdeHandle&, GlobalData& globalData, int key)
//   {
//     //dsPrint ( "Eingabe erfolgt %d (`%c')\n" , cmd , cmd );
//     switch ( (char) key )
//       {
//       case 'y' : dBodyAddForce ( sphere1->object[Sphererobot::Base]->getBody() , 10 ,0 , 0 ); break;
//       case 'Y' : dBodyAddForce ( sphere1->object[Sphererobot::Base]->getBody() , -10 , 0 , 0 ); break;
//       case 'x' : dBodyAddTorque ( sphere1->object[Sphererobot::Base]->getBody() , 0 , 0 , 3 ); break;
//       case 'X' : dBodyAddTorque ( sphere1->object[Sphererobot::Base]->getBody() , 0 , 0 , -3 ); break;
//       case 'S' : controller->setParam("sineRate", controller->getParam("sineRate")*1.2); 
// 	printf("sineRate : %g\n", controller->getParam("sineRate"));
//       break;
//       case 's' : controller->setParam("sineRate", controller->getParam("sineRate")/1.2); 
// 	printf("sineRate : %g\n", controller->getParam("sineRate"));
// 	break;
//       }
//   }
  
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
 
 
