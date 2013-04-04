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
 *   Revision 1.3  2011-11-11 15:40:51  martius
 *   test for collorschema
 *
 *   Revision 1.2  2011/06/03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.1  2010/03/21 10:19:56  martius
 *   test_graphics moved to tests/graphics
 *
 *   Revision 1.6  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.5  2010/03/05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *   Revision 1.4  2009/03/25 15:44:23  guettler
 *   ParallelSplitShadowMap: corrected light direction (using directional light), complete ground is now shadowed
 *
 *   Revision 1.3  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.2  2009/03/09 16:50:49  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2009/02/04 09:35:46  martius
 *   test for new graphics stuff.
 *   At the moment for OsgBoxTex
 *
 *
 *
 ***************************************************************************/
#include <stdio.h>

// include ode library
#include <ode-dbl/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>


// used arena
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/joint.h>

/************/
#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/octaplayground.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:


  AbstractObstacle* playground;
  double hardness;
  Substance s;
  OSGBoxTex* b;
  OSGBox* b2;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));

    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("noise",0.1);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("gravity", -3);

    // use Playground as boundary:
    playground = new Playground(odeHandle, osgHandle,
                                osg::Vec3(10, .2, 1));
    playground->setPosition(osg::Vec3(0,0,0.2));
    global.obstacles.push_back(playground);

    playground = new OctaPlayground(odeHandle, osgHandle);
    playground->setPosition(osg::Vec3(15,0,0.2));
    global.obstacles.push_back(playground);



    // add passive spheres as obstacles
    for (int i=0; i< 1/*2*/; i+=1){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.3);
      // s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      s1->setPosition(osg::Vec3(0,0,1+i*5));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }

    b = new OSGBoxTex(5,1,2);
    b->setTexture(0,TextureDescr("Images/dusty.rgb",1,1));
    b->setTexture(1,TextureDescr("Images/tire_full.rgb",3,1));
    b->setTexture(2,TextureDescr("Images/whitemetal_farbig_small.rgb",1,1));
    b->setTexture(3,TextureDescr("Images/wall.rgb",1,1));
    b->setTexture(4,TextureDescr("Images/really_white.rgb",1,1));
    b->setTexture(5,TextureDescr("Images/light_chess.rgb",-1,-1));
    b->init(osgHandle.changeColor(Color(1,1,0)));
    b->setMatrix(osg::Matrix::translate(0,-2,2));

    b2 = new OSGBox(5,1,2);
    b2->setTexture(TextureDescr("Images/light_chess.rgb",1,1));
    b2->init(osgHandle);
    b2->setMatrix(osg::Matrix::translate(7,0,2));




  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    b->setMatrix(osg::Matrix::rotate(globalData.time/2,1,0,0)*osg::Matrix::translate(0,-2,2));
    b2->setMatrix(osg::Matrix::rotate(globalData.time/2,1,0,0)*osg::Matrix::translate(7,0,2));
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'i':
          if(playground) {
            s.hardness*=1.5;
            cout << "hardness " << s.hardness << endl;
            playground->setSubstance(s);
          }
          return true;
          break;
        case 'j':
          if(playground) {
            s.hardness/=1.5;
            cout << "hardness " << s.hardness << endl;
            playground->setSubstance(s);
          }
          return true;
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
  return sim.run(argc, argv) ? 0 : 1;

}

