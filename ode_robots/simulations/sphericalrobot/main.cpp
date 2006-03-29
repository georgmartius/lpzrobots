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
 *   Revision 1.6.4.4  2006-03-29 15:10:22  martius
 *   *** empty log message ***
 *
 *   Revision 1.6.4.3  2006/02/17 16:47:55  martius
 *   moved to new system
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

#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include "forcedsphere.h"
#include "sphererobot3masses.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

// plotoptions is a list of possible online output, 
// if the list is empty no online gnuplot windows and no logging to file occurs.
// The list is modified with commandline options, see main() at the bottom of this file
list<PlotOption> plotoptions;

AbstractController *controller;

class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    global.odeConfig.setParam("noise",0.03);
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",1);

//   // Outer Ring
//   AbstractObstacle* ring1 = new OctaPlayground(odeHandle, 20);
//   ring1->setGeometry(6, 0.1, 2); 
//   ring1->setPosition(0,0,0); // playground positionieren und generieren
//   global.obstacles.push_back(ring1);
//   // Inner Ring
//   AbstractObstacle* ring2 = new OctaPlayground(odeHandle, 24);
//   ring2->setGeometry(11.5, 0.1, 2);
//   ring2->setPosition(0,0,0); // playground positionieren und generieren
//   global.obstacles.push_back(ring2);

//     for(int i=0; i<5; i++){
//       PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5);
//       s->setPosition(osg::Vec3(5,0,i*3)); 
//       global.obstacles.push_back(s);    
//     }

    
    //****************
     OdeRobot* sphere1;
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    conf.axisZsensor=true;
    conf.axisXYZsensor=false;
    conf.irAxis1=false;
    conf.irAxis2=false;
    conf.irAxis3=false;
    conf.spheremass   = 1;
    sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)), 
				       conf, "Sphere1", 0.2); 
//    sphere1 = new ForcedSphere(odeHandle, osgHandle, "FSphere");
    
    sphere1->place ( Pos( 0 , 0 , 0 ));
    controller = new InvertMotorNStep();
    controller->setParam("steps", 2);    
    controller->setParam("adaptrate", 0.001);    
    controller->setParam("nomupdate", 0.005);    
    controller->setParam("epsC", 0.01);    
    controller->setParam("epsA", 0.005);    
    controller->setParam("rootE", 3);    
    controller->setParam("logaE", 2);    
//     controller = new SineController();  
//     controller->setParam("sinerate", 40);  
//     controller->setParam("phaseshift", 0.0);
    
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
    OdeAgent* agent = new OdeAgent ( plotoptions );
    agent->init ( controller , sphere1 , wiring );
    //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
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
  

};

// print command line options
void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile\n", progname);
}

int main (int argc, char **argv)
{ 
  // start with online windows (default: start without plotting and logging)
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger));
  
  // start with online windows and logging to file
  if(contains(argv, argc, "-f")) plotoptions.push_back(PlotOption(GuiLogger_File, Controller, 5));
  
  // display help
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}
 
