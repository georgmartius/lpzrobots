#/***************************************************************************
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
 *
 ***************************************************************************/
#include <stdio.h>


// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>
#include <selforg/matrix.h>
#include <selforg/controller_misc.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// arena
#include <ode_robots/playground.h>

// used passive spheres and boxes
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>
#ifdef PASSIVECYLINDER
#include <ode_robots/passivecylinder.h>
#endif

using namespace std;

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class ThisSim : public Simulation {
public:

  int height;
  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-2.4246, 20.8109, 7.10493),  Pos(-167.137, -3.08307, 0));
    setCameraMode(Static);

    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("noise", 0);
    global.odeConfig.setParam("simstepsize", 0.01);
    global.odeConfig.setParam("realtimefactor", 0.1);
    global.odeConfig.setParam("gravity", 0);
    global.odeConfig.setParam("cameraspeed", 400);

    AbstractGround* playground;
    playground = new Playground(odeHandle, osgHandle, osg::Vec3(8, 6, 3), 5, false);
    playground->setColor(Color(2,2,2,.5));
    playground->setPosition(osg::Vec3(0,0,5)); // playground positionieren und generieren
    global.obstacles.push_back(playground);


    //// BOXES
    AbstractObstacle* o;
    o = new PassiveBox(odeHandle, osgHandle, osg::Vec3(1,1,4));
    o->setPose(osg::Matrix::translate(6,0,9));
    global.obstacles.push_back(o);
    o = new PassiveBox(odeHandle, osgHandle, osg::Vec3(4,1,1));
    o->setPose(osg::Matrix::translate(3,-2,6.5));
    global.obstacles.push_back(o);
    //// Spheres
    o = new PassiveSphere(odeHandle, osgHandle, 1);
    o->setPose(osg::Matrix::translate(6,-4,7.5));
    global.obstacles.push_back(o);
    o = new PassiveSphere(odeHandle, osgHandle, 1);
    o->setPose(osg::Matrix::translate(5,-5,6.5));
    global.obstacles.push_back(o);
    //// Capsules
    o = new PassiveCapsule(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::translate(8,2,7.5));
    global.obstacles.push_back(o);
    o = new PassiveCapsule(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::translate(6,4,6.5));
    global.obstacles.push_back(o);

    o = new PassiveCapsule(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::translate(8,6,8.5));
    global.obstacles.push_back(o);
    o = new PassiveCapsule(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::rotate(M_PI/2.0, 0,1,0)*osg::Matrix::translate(4,8,6));
    global.obstacles.push_back(o);
    o = new PassiveCapsule(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::rotate(M_PI/2.0, 0,1,0)*osg::Matrix::translate(3.5,10,6));
    global.obstacles.push_back(o);
    o = new PassiveCapsule(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::rotate(M_PI/2.0, 0,1,0)*osg::Matrix::translate(3,12,6));
    global.obstacles.push_back(o);

#ifdef PASSIVECYLINDER
    //// Cylinder
    o = new PassiveCylinder(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::translate(-8,2,7.5));
    global.obstacles.push_back(o);
    o = new PassiveCylinder(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::translate(-6,4,6.5));
    global.obstacles.push_back(o);

    o = new PassiveCylinder(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::translate(-8,6,8.5));
    global.obstacles.push_back(o);
    o = new PassiveCylinder(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::rotate(M_PI/2.0, 0,1,0)*osg::Matrix::translate(-4,8,6));
    global.obstacles.push_back(o);
    o = new PassiveCylinder(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::rotate(M_PI/2.0, 0,1,0)*osg::Matrix::translate(-3.5,10,6));
    global.obstacles.push_back(o);
    o = new PassiveCylinder(odeHandle, osgHandle, .5, 2);
    o->setPose(osg::Matrix::rotate(M_PI/2.0, 0,1,0)*osg::Matrix::translate(-3,12,6));
    global.obstacles.push_back(o);
#endif

  }


  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
  };



  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       GlobalData& globalData, int key, bool down)
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

