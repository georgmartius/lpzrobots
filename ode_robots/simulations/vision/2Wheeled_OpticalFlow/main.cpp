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
 *   Revision 1.4  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.3  2010/03/25 16:38:07  martius
 *   vision experiments
 *
 *   Revision 1.2  2010/03/24 16:51:38  martius
 *   QuickMP uses now the number of processors in the system
 *   optical flow improved
 *   video recording works with offscreen rendering
 *   Make system: Optimization -O1 is switched on by default (add a debug version without optimization)
 *
 *   Revision 1.1  2010/03/23 18:42:26  martius
 *   new simulation for vision
 *
 *
 ***************************************************************************/
#include <stdio.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>
#include <selforg/invertmotorspace.h>
#include <selforg/semox.h>
#include <selforg/crossmotorcoupling.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>


// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

#include <ode_robots/camera.h>
#include <ode_robots/imageprocessors.h>
#include <ode_robots/camerasensors.h>
#include <ode_robots/opticalflow.h>

#include <ode_robots/twowheeled.h>


#include <osg/Light>
#include <osg/LightSource>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:


  AbstractObstacle* playground;


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    int numSeeingRobots=1;
    int numBlindRobots=0;

    int numBalls=5;

    setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));

    global.odeConfig.setParam("controlinterval",4);

    // use Playground as boundary:
    playground = new Playground(odeHandle, osgHandle,
                                osg::Vec3(10, .2, 1));
    playground->setPosition(osg::Vec3(0,0,0.1));
    global.obstacles.push_back(playground);

    // add passive spheres as obstacles
    for (int i=0; i< numBalls; i++){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(1,1,0)), 0.3);
      // s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      s1->setPosition(osg::Vec3(0,-2+i,1));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }

    for(int i=0; i<numSeeingRobots; i++){
      // the twowheeled robot is derived from Nimm2 and has a camera onboard
      TwoWheeledConf twc = TwoWheeled::getDefaultConf();
      twc.n2cfg.force=2;
      twc.n2cfg.speed=8;

      twc.camcfg.width  = 256;
      twc.camcfg.height = 128;
      twc.camcfg.fov    = 120;
      // get rid of the image processing
      twc.camcfg.removeProcessors();

      OpticalFlowConf ofc = OpticalFlow::getDefaultConf();
      ofc.dims    = Sensor::X;
      ofc.verbose = 1;
      ofc.points  = OpticalFlow::getDefaultPoints(3);
      twc.camSensor     = new OpticalFlow(ofc);

      OdeRobot* vehicle = new TwoWheeled(odeHandle, osgHandle, twc,
                                         "CamRobot_" + itos(i));
      vehicle->setColor(Color(1,.7,0));
      vehicle->place(osg::Matrix::rotate(M_PI, 0,0,1)
                     *osg::Matrix::translate(3,-4+2*i,0.3));

      SeMoXConf cc = SeMoX::getDefaultConf();
      cc.modelExt = true;
      SeMoX *semox = new SeMoX(cc);
      CrossMotorCoupling* controller = new CrossMotorCoupling(semox, semox, 0.0);
      std::list<int> perm;
      perm += 1;
      perm += 0;
      controller->setCMC(CrossMotorCoupling::getPermutationCMC(perm));
      controller->setParam("rootE",3);
      controller->setParam("gamma_teach",0.005);

      // AbstractController *controller = new Braitenberg(Braitenberg::Aggressive, 2, 3);
//       AbstractWiring* wiring = new SelectiveOne2OneWiring(new ColorUniformNoise(0.1),
//                                                           new select_from_to(2,3));
      AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
      OdeAgent* agent = new OdeAgent( i==0 ? plotoptions : std::list<PlotOption>(),0.5);
      agent->init(controller, vehicle, wiring);
      global.configs.push_back(controller);
      global.agents.push_back(agent);
    }


    for(int i=0; i<numBlindRobots; i++){
      // this robot has no camera
      OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, Nimm2::getDefaultConf(),
                                    "BlindRobot_" + itos(i));
      vehicle->setColor(Color(1,1,0));
      vehicle->place(Pos(-3,-4+2*i,0.3));
      AbstractController *controller = new InvertMotorSpace(10);
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      OdeAgent* agent = new OdeAgent();
      agent->init(controller, vehicle, wiring);
      global.agents.push_back(agent);
    }


  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
  }

  virtual void end(GlobalData& globalData){
  }

  osg::LightSource* makeLights(osg::StateSet* stateset)
  {
    // create a directional light (infinite distance place at 45 degrees)
    osg::Light* myLight = new osg::Light;
    myLight->setLightNum(1);
    myLight->setPosition(osg::Vec4(1.0,1.0,1.0,0.0f));
    myLight->setDirection(osg::Vec3(-1.0, -1.0, -1.0));
    myLight->setAmbient(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    myLight->setDiffuse(osg::Vec4(.8f,.8f,.8f,1.0f));
    myLight->setConstantAttenuation(1.0f);

    osg::LightSource* lightS = new osg::LightSource;
    lightS->setLight(myLight);
    lightS->setLocalStateSetModes(osg::StateAttribute::ON);

    lightS->setStateSetModes(*stateset,osg::StateAttribute::ON);

    return lightS;
  }


};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}

