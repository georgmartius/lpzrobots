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
 *   Revision 1.16  2006-07-14 12:23:54  martius
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

#include "simulation.h"

#include "odeagent.h"
#include "octaplayground.h"
#include "passivesphere.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include "sphererobot.h"
#include "sphererobot3masses.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


AbstractController *controller;
Sphererobot3Masses* sphere1;

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

    for(int i=0; i<5; i++){
      PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5);
      s->setPosition(osg::Vec3(5,0,i*3)); 
      global.obstacles.push_back(s);    
    }

    
    //****************
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)), 
				       conf, "Sphere1", 0.2); 
    //    SphererobotConf conf = Sphererobot::getDefaultConf();  
    //    sphere1 = new Sphererobot ( odeHandle, osgHandle, conf, "Sphere1");
    
    ((OdeRobot*)sphere1)->place ( Pos( 0 , 0 , 0.1 ));
    //controller = new InvertNChannelController(10);  
    controller = new SineController();  
    controller->setParam("sinerate", 40);  
    controller->setParam("phaseshift", 0.0);
    
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
    OdeAgent* agent = new OdeAgent ( plotoptions );
    agent->init ( controller , sphere1 , wiring );
    // agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
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
 
 
