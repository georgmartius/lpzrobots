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
 *   Revision 1.8  2006-11-29 09:16:09  martius
 *   modell stuff
 *
 *   Revision 1.7  2006/07/14 12:23:52  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.6.4.6  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.6.4.5  2006/05/15 12:29:43  robot3
 *   handling of starting the guilogger moved to simulation.cpp
 *   (is in internal simulation routine now)
 *
 *   Revision 1.6.4.4  2006/03/29 15:10:22  martius
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
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include "forcedsphere.h"
#include "sphererobot3masses.h"
#include "barrel2masses.h"
#include "axisorientationsensor.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  AbstractController *controller;
  OdeRobot* sphere1;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(-0.497163, 11.6358, 3.67419),  Pos(-179.213, -11.6718, 0));
    // initialization
    global.odeConfig.setParam("noise",0.03);
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",4);

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
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    conf.pendularrange  = 0.3; 
    conf.motorsensor=false;
    conf.axisZsensor=true;
    conf.axisXYZsensor=false;
    conf.irAxis1=false;
    conf.irAxis2=false;
    conf.irAxis3=false;
    conf.spheremass   = 1;
    //    sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)), 
    //				       conf, "Sphere1", 0.2); 
    sphere1 = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)), 
				  conf, "Barrel1", 0.2); 
    //// FORCEDSPHERE
    // ForcedSphereConf fsc = ForcedSphere::getDefaultConf();
    // fsc.drivenDimensions=ForcedSphere::X;
    // fsc.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    // sphere1 = new ForcedSphere(odeHandle, osgHandle, fsc, "FSphere");
    // 
    sphere1->place ( osg::Matrix::rotate(M_PI/2, 1,0,0));

    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.cInit=0.5;
    controller = new InvertMotorNStep(cc);    
    //controller = new SineController();
    controller->setParam("steps", 2);    
    //    controller->setParam("adaptrate", 0.001);    
    controller->setParam("adaptrate", 0.0);    
    controller->setParam("nomupdate", 0.005);    
    controller->setParam("epsC", 0.03);    
    controller->setParam("epsA", 0.05);    
    // controller->setParam("epsC", 0.001);    
    // controller->setParam("epsA", 0.001);    
    //    controller->setParam("rootE", 1);    
    //    controller->setParam("logaE", 2);    
    controller->setParam("rootE", 3);    
    controller->setParam("logaE", 0);    
//     controller = new SineController();  
    controller->setParam("sinerate", 15);  
    controller->setParam("phaseshift", 0.45);
    
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
    OdeAgent* agent = new OdeAgent ( plotoptions );
    agent->init ( controller , sphere1 , wiring );
    //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
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
	case 'y' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , 10 ,0 , 0 ); break;
	case 'Y' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , -10 , 0 , 0 ); break;
	case 'x' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , 3 ); break;
	case 'X' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , -3 ); break;
	case 'S' : controller->setParam("sineRate", controller->getParam("sineRate")*1.2); 
	  printf("sineRate : %g\n", controller->getParam("sineRate"));
	  break;
	case 's' : controller->setParam("sineRate", controller->getParam("sineRate")/1.2); 
	  printf("sineRate : %g\n", controller->getParam("sineRate"));
	  break;
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
 
