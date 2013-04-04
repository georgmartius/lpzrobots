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
 *   Revision 1.2  2010/03/23 18:43:54  martius
 *   lpzviewer: added checking function
 *   camerasensor new initialization
 *   twowheeled allows full customization of camera
 *   optical flow improved multiscale check
 *
 *   Revision 1.1  2010/03/21 10:17:06  martius
 *   test simulation for camera
 *
 *   Revision 1.4  2010/03/19 17:46:21  martius
 *   camerasensors added
 *   camera works great now. Near and far plane fixed by hand and optimal positioning
 *   many image processings added
 *
 *   Revision 1.3  2010/03/17 17:26:36  martius
 *   robotcameramanager uses keyboard and respects resize
 *   (robot) camera is has a conf object
 *   image processing implemented, with a few standard procedures
 *
 *   Revision 1.2  2010/03/16 17:12:08  martius
 *   includes of ode/ changed to ode-dbl/
 *   more testing code added
 *
 *   Revision 1.1  2010/03/05 14:28:41  martius
 *   test simulation for camera and other sensors
 *
 *
 *
 ***************************************************************************/
#include <stdio.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>
#include <selforg/invertmotorspace.h>
#include <selforg/one2onewiring.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>


// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/joint.h>

#include <ode_robots/camera.h>
#include <ode_robots/imageprocessors.h>


#include <ode_robots/nimm2.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:


  AbstractObstacle* playground;
  double hardness;
  Substance s;
  OSGBoxTex* b;

  PassiveBox* box;
  Camera* cam;
  Camera* cam2;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));

    global.odeConfig.setParam("controlinterval",20);
    fprintf(stderr, "****\nAttention, the controlinterval is set to 20\n");
    fprintf(stderr, " see the independend update of robot cameras\n****\n");



    // use Playground as boundary:
    playground = new Playground(odeHandle, osgHandle,
                                osg::Vec3(10, .2, 1));
    playground->setPosition(osg::Vec3(0,0,0.1));
    global.obstacles.push_back(playground);

    // add passive spheres as obstacles
    for (int i=0; i< 1/*2*/; i+=1){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.3);
      // s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      s1->setPosition(osg::Vec3(0,0,1+i*5));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }

    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, Nimm2::getDefaultConf(), "Robot");
    vehicle->setColor(Color(1,1,0));
    vehicle->place(Pos(-3,2,0.3));
    AbstractController *controller = new InvertMotorSpace(10);
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);


    box = new PassiveBox(odeHandle, osgHandle);
    box->setPose(osg::Matrix::rotate(M_PI/2, 1,0,0) * osg::Matrix::translate(-3,0,0.5));
    global.obstacles.push_back(box);

    CameraConf camc2 = Camera::getDefaultConf();
    camc2.width  = 256;
    camc2.height = 256;
    camc2.scale  = .5;
    //    camc2.processors.push_back(new BWImageProcessor(true,1));
    camc2.processors.push_back(new HSVImgProc(false,1));
    //    camc2.processors.push_back(new BWImageProcessor(true,1, BWImageProcessor::Saturation));
    camc2.processors.push_back(new ColorFilterImgProc(true,.5,
                                                      HSVImgProc::Red+20, HSVImgProc::Green-20,100));
    cam2 = new Camera(camc2);

//     osgViewer::ViewerBase::Views views;
//     viewer->getViews(views);
//     assert(views.size()>0);

    cam2->init(odeHandle, osgHandle.changeColor(Color(0.5,0,0)), box->getMainPrimitive(),
               osg::Matrix::translate(0,0,0.5)*osg::Matrix::rotate(M_PI/2, 0,1,0));

    CameraConf camc = Camera::getDefaultConf();
    camc.width = 512;
    camc.height = 64;
    camc.scale  = .9;
    cam = new Camera(camc);
    cam->init(odeHandle, osgHandle.changeColor(Color(0,0,0)), box->getMainPrimitive(),
              osg::Matrix::translate(0,0,0.5));


    b = new OSGBoxTex(5,1,2);
    b->setTexture(0,"Images/dusty.rgb",1,1);
    b->setTexture(1,"Images/tire_full.rgb",3,1);
    b->setTexture(2,"Images/whitemetal_farbig_small.rgb",1,1);
    b->setTexture(3,"Images/wall.rgb",1,1);
    b->setTexture(4,"Images/really_white.rgb",1,1);
    b->setTexture(5,"Images/light_chess.rgb",-1,-1);
    b->init(osgHandle.changeColor(Color(1,1,0)));
    b->setMatrix(osg::Matrix::translate(0,-2,2));


  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    b->setMatrix(osg::Matrix::rotate(globalData.time/2,1,0,0)*osg::Matrix::translate(0,-2,2));
    box->setPose(osg::Matrix::rotate(M_PI/2, 1,0,0) * osg::Matrix::rotate(globalData.time/3,0,0,1) * osg::Matrix::translate(-3,0,0.6));
    cam->update();
    cam2->update();
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

  virtual void end(GlobalData& globalData){
    delete cam;
    delete cam2;
    delete b;
  }
};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}

