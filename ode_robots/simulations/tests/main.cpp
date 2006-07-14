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
 *   Revision 1.2  2006-07-14 12:23:54  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.2  2006/06/25 17:01:57  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.1.2.1  2006/06/16 21:35:50  martius
 *   test environment
 *
 *   Revision 1.14.4.15  2006/05/18 11:55:56  robot3
 *   made playground smaller (for shadowing issues)
 *
 *   Revision 1.14.4.14  2006/05/15 13:11:30  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.14.4.13  2006/04/25 09:06:16  robot3
 *   *** empty log message ***
 *
 *   Revision 1.14.4.12  2006/03/31 12:14:49  fhesse
 *   orange color for nimm robot
 *
 *   Revision 1.14.4.11  2006/03/31 11:27:53  fhesse
 *   documentation updated
 *   one sphere removed (todo: fix prob;em with sphere placing)
 *
 *   Revision 1.14.4.10  2006/01/12 15:17:46  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.9  2005/12/29 15:55:33  martius
 *   end is obsolete
 *
 *   Revision 1.14.4.8  2005/12/29 15:47:12  martius
 *   changed to real Sim class
 *
 *   Revision 1.14.4.7  2005/12/14 15:37:25  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.6  2005/12/13 18:12:09  martius
 *   switched to nimm2
 *
 *   Revision 1.14.4.5  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.4  2005/12/09 16:53:17  martius
 *   camera is working now
 *
 *   Revision 1.14.4.3  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *   Revision 1.14.4.2  2005/11/24 16:19:12  fhesse
 *   include corrected
 *
 *   Revision 1.14.4.1  2005/11/15 12:30:07  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>

// include ode library
#include <ode/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include "simulation.h"

// used passive spheres
#include "passivesphere.h"


#include "joint.h"



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
    global.odeConfig.noise=0.1;
    //  global.odeConfig.setParam("gravity", 0);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
    s1->setPosition(osg::Vec3(0,0,0));
    global.obstacles.push_back(s1);
     
    s2 = new PassiveSphere(odeHandle, osgHandle, 0.5);
    s2->setPosition(osg::Vec3(1,0,0));
    global.obstacles.push_back(s2);
    
     j = new HingeJoint(s1->getMainPrimitive(), s2->getMainPrimitive(), Pos(0.5,0,0.5), Axis(0,0,1));
     j->init(odeHandle, osgHandle);
    
  
    showParams(global.configs);
  }


  virtual void addCallback(GlobalData& globalData, bool draw, bool pause) {
    if(j) j->update();
  };

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	case 'a': dBodyAddForce(s1->getMainPrimitive()->getBody(),5,0,0); break;
	case 'd': dBodyAddForce(s1->getMainPrimitive()->getBody(),0,5,0); break;
	case 'g': if(j) delete(j); j=0; break;
	default:
	  return false;
	  break;
	} 
    }
    return false;
  }


  PassiveSphere* s1;
  PassiveSphere* s2;
  Joint* j;

};


int main (int argc, char **argv)
{ 
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}
 
