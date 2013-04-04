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
 *   Revision 1.3  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.2  2010/09/30 17:07:08  martius
 *   tests and vision experiments improved
 *
 *   Revision 1.1  2010/09/24 09:25:00  martius
 *   collissions tests changed
 *
 *   Revision 1.2  2010/09/23 08:34:58  martius
 *   new colors
 *
 *   Revision 1.1  2010/09/23 08:32:02  martius
 *   new test for collisions
 *
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

// controller and robot
#include <selforg/semox.h>
#include <selforg/one2onewiring.h>
#include <ode_robots/odeagent.h>

#include <ode_robots/skeleton.h>

using namespace std;

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class ThisSim : public Simulation {
public:

  enum OType {OBox, OSphere, OCaps};
  int height;

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    addColorAliasFile("overwriteGroundColor.txt");
    setGroundTexture("Images/whiteground.jpg");
    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-7.90217, 11.727, 5.82483),  Pos(-144.937, -21.2825, 0));
    setCameraMode(Static);

    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("noise", 0);
    global.odeConfig.setParam("simstepsize", 0.002);


    // AbstractGround* playground;
    // playground = new Playground(odeHandle, osgHandle, osg::Vec3(8, 0.2, 1), 1);
    // playground->setGroundColor(Color(2,2,2,1));
    // playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
    // global.obstacles.push_back(playground);

    height=0;


  }


  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
  };

  virtual void addObject(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                         GlobalData& globalData, OType type){
    OdeHandle handle2 = odeHandle;
    OsgHandle osgHandle2;
    double radius = 1;
    double size = .5;
    int subtype = rand()%4;
    switch (subtype){
    case 0:
      handle2.substance.toMetal(1);
      osgHandle2 = osgHandle.changeColor(Color(0.5,0.5,0.5));
      break;
    case 1:
      handle2.substance.toPlastic(1);
      osgHandle2 = osgHandle.changeColor(Color(1,1,1));
      break;
    case 2:
      handle2.substance.toRubber(10);
      osgHandle2 = osgHandle.changeColor(Color(0.2,0.2,0.2));
      break;
    default:
      handle2.substance.toFoam(5);
      osgHandle2 = osgHandle.changeColor(Color(1,1,0));
      break;
    }

    AbstractObstacle* o;
    Pos dim((random_minusone_to_one(0)+1.1)*size, (random_minusone_to_one(0)+1.1)*size, (random_minusone_to_one(0)+1.1)*size);
    switch (type){
    case OBox:
      o = new PassiveBox(handle2, osgHandle2, dim, dim.x()*dim.y()*dim.z());
      break;
    case OSphere:
      o = new PassiveSphere(handle2, osgHandle2, dim.x()/2.0, 2.0/3.0*M_PI*pow(dim.x(),3));
      break;
    case OCaps:
      o = new PassiveCapsule(handle2, osgHandle2, dim.x()/2.0, dim.z()/2.0, M_PI*sqr(dim.x())*dim.z()/8);
      break;
    }
    Pos pos((random_minusone_to_one(0))*radius, (random_minusone_to_one(0))*radius, 4+height%3);
    height++;
    o->setPosition(pos);
    globalData.obstacles.push_back(o);
  }

  virtual void addHumanoid(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                         GlobalData& globalData){

    SkeletonConf conf = Skeleton::getDefaultConfVelServos();
    conf.powerFactor=0.1;
    OsgHandle skelOsgHandle=osgHandle.changeColorSet(0);

    Skeleton* human = new Skeleton(odeHandle, skelOsgHandle, conf, "Humanoid");
    human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)
                 *osg::Matrix::translate(0,0,4));

    globalData.configs.push_back(human);
    SeMoXConf cc = SeMoX::getDefaultConf();
    cc.modelExt=true;
    cc.cInit = 1.2;
    SeMoX* controller = new SeMoX(cc);

    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(globalData);
    agent->init(controller, human, wiring);
    globalData.configs.push_back(controller);
    globalData.agents.push_back(agent);

  }


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'b': addObject(odeHandle, osgHandle, globalData, OBox); break;
        case 'k': addObject(odeHandle, osgHandle, globalData, OSphere); break;
        case 'c': addObject(odeHandle, osgHandle, globalData, OCaps); break;
        case 'r': addHumanoid(odeHandle, osgHandle, globalData); break;
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

