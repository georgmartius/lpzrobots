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
#include <ode_robots/octaplayground.h> // arena

// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

#include <ode_robots/camera.h>
#include <ode_robots/imageprocessors.h>
#include <ode_robots/camerasensors.h>

#include <ode_robots/twowheeled.h>
#include <ode_robots/fourwheeled.h>


#include <osg/Light>
#include <osg/LightSource>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    int numSeeing2wheeled = 0;
    int numSeeing4wheeled = 1;
    int numBlindRobots    = 0;

    int numBalls          = 5;

    bool useSquareGround  = false;
    bool useCorridor      = true;
    double radius         = 10;

    if(useCorridor)
      setCameraHomePos(Pos(-2.14663, 10.6543, 2.19406),  Pos(131.61, -13.1261, 0));
    else
      setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));

    global.odeConfig.setParam("controlinterval",4);

    addParameterDef("attraction", &attraction, 0.001);
    global.configs.push_back(this);

    if(useSquareGround){
      // use Playground as boundary:
      Playground* playground = new Playground(odeHandle, osgHandle,
                                              osg::Vec3(10, .2, 1));
      playground->setPosition(osg::Vec3(0,0,0.1));
      playground->setGroundSubstance(Substance(0.8,0,40,0.5));
      global.obstacles.push_back(playground);
    }else if(useCorridor){       // use circular Corridor
      // outer ground
      OdeHandle wallHandle = odeHandle;
      wallHandle.substance.toMetal(0.1);
      OctaPlayground* outer = new OctaPlayground(wallHandle, osgHandle,
                                                 osg::Vec3(radius+1, 0.2, 1), 12);
      outer->setPosition(osg::Vec3(0,0,0.1));
      outer->setGroundSubstance(Substance(0.3,0.005,40,0.5));
      global.obstacles.push_back(outer);
      // inner walls (without ground
      OctaPlayground* inner = new OctaPlayground(wallHandle, osgHandle,
                                                 osg::Vec3(radius-2, 0.2, 1), 12, false);
      inner->setPosition(osg::Vec3(0,0,0.1));
      global.obstacles.push_back(inner);
    }

    // add passive spheres as obstacles
    for (int i=0; i< numBalls; i++){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(1,1,0)), 0.3);
      // s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      if(useCorridor) s1->setPosition(osg::Vec3(sin(i/3.0)*radius,cos(i/3.0)*radius,1));
      else  s1->setPosition(osg::Vec3(i%5,-2+i/5,1));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }

    for(int i=0; i<numSeeing2wheeled; i++){
      // the twowheeled robot is derived from Nimm2 and has a camera onboard
      TwoWheeledConf twc = TwoWheeled::getDefaultConf();
      twc.n2cfg.force=2;
      twc.n2cfg.speed=8;

      twc.camcfg.width  = 256;
      twc.camcfg.height = 128;
      twc.camcfg.fov    = 120;
      // get rid of the image processing
      delete twc.camcfg.processors.back();
      twc.camcfg.processors.pop_back();

      MotionCameraSensorConf mc = MotionCameraSensor::getDefaultConf();
      mc.values = MotionCameraSensor::Size | MotionCameraSensor::SizeChange;
      twc.camSensor     = new MotionCameraSensor(mc);

      OdeRobot* vehicle = new TwoWheeled(odeHandle, osgHandle, twc,
                                         "CamRobot_" + itos(i));
      vehicle->setColor(Color(1,.7,0));
      if(useCorridor)
        vehicle->place(osg::Vec3(sin(i/2.0-1)*radius,cos(i/2.0-1)*radius,0.3));
      else
        vehicle->place(osg::Matrix::rotate(M_PI, 0,0,1)
                       * osg::Matrix::translate(3,-4+2*i,0.3));


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
      OdeAgent* agent = new OdeAgent( i==0 ? plotoptions : std::list<PlotOption>(),0.1);
      agent->init(controller, vehicle, wiring);
      global.configs.push_back(controller);
      global.agents.push_back(agent);
    }

    /// FOURWHEELED
    for(int i=0; i<numSeeing4wheeled; i++){
      FourWheeledConf fwc = FourWheeled::getDefaultConf();
      fwc.twoWheelMode = true;
      fwc.useBumper    = false;
      OdeRobot* robot = new FourWheeled(odeHandle, osgHandle,
                                        fwc,
                                        "4WCamRobot_" + itos(i));

      CameraConf camcfg = Camera::getDefaultConf();
      camcfg.width  = 256;
      camcfg.height = 128;
      camcfg.fov    = 120;
      camcfg.camSize = 0.08;
      camcfg.processors.push_back(new HSVImgProc(false,1));
      // filter only Yellow color
      camcfg.processors.push_back(new ColorFilterImgProc(true, .5,
                                  HSVImgProc::Red+20, HSVImgProc::Green-20,100));
      Camera* cam = new Camera(camcfg);
      MotionCameraSensorConf mc = MotionCameraSensor::getDefaultConf();
      mc.values = MotionCameraSensor::SizeChange;
      auto camSensor = std::make_shared<MotionCameraSensor>(mc);
      camSensor->setInitData(cam, odeHandle, osgHandle, osg::Matrix::rotate(-M_PI/2,0,0,1)
                             * osg::Matrix::translate(0.2,0, 0.40) );
      robot->addSensor(camSensor);

      robot->setColor(Color(1,.7,0));
      if(useCorridor)
        robot->place(osg::Vec3(sin(i/2.0-1)*radius,cos(i/2.0-1)*radius,0.3));
      else
        robot->place(osg::Matrix::rotate(M_PI, 0,0,1)
                       * osg::Matrix::translate(3,-4+2*i,0.3));


      SeMoXConf cc = SeMoX::getDefaultConf();
      cc.modelExt = true;
      SeMoX *semox = new SeMoX(cc);
      //      AbstractController* controller = semox;
      CrossMotorCoupling* controller = new CrossMotorCoupling(semox, semox, 0.0);
      std::list<int> perm;
      perm += 1;
      perm += 0;
      controller->setCMC(CrossMotorCoupling::getPermutationCMC(perm));
      //  controller->setParam("rootE",3);
      controller->setParam("gamma_teach",0.005);

      AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
      OdeAgent* agent = new OdeAgent( i==0 ? plotoptions : std::list<PlotOption>(),0.1);
      agent->init(controller, robot, wiring);
      global.configs.push_back(controller);
      global.agents.push_back(agent);
    }


    for(int i=0; i<numBlindRobots; i++){
      // this robot has no camera
      OdeRobot* robot = new Nimm2(odeHandle, osgHandle, Nimm2::getDefaultConf(),
                                    "BlindRobot_" + itos(i));
      robot->setColor(Color(1,1,0));
      robot->place(Pos(-3,-4+2*i,0.3));
      AbstractController *controller = new InvertMotorSpace(10);
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      OdeAgent* agent = new OdeAgent();
      agent->init(controller, robot, wiring);
      global.agents.push_back(agent);
    }


  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    // move all balls to robot
    Pos rpos = (*globalData.agents.begin())->getRobot()->getPosition();
    FOREACH(ObstacleList, globalData.obstacles, o){
      PassiveSphere* s = dynamic_cast<PassiveSphere*>(*o);
      if(s){
        Pos spos = s->getMainPrimitive()->getPosition();
        s->getMainPrimitive()->applyForce((rpos-spos)*attraction);
      }
    }
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
    myLight->setAmbient(osg::Vec4(.9f,.9f,.9f,.9f));
    myLight->setDiffuse(osg::Vec4(.7f,.7f,.7f,.7f));
    myLight->setConstantAttenuation(1.0f);

    osg::LightSource* lightS = new osg::LightSource;
    lightS->setLight(myLight);
    lightS->setLocalStateSetModes(osg::StateAttribute::ON);

    lightS->setStateSetModes(*stateset,osg::StateAttribute::ON);

    return lightS;
  }

  paramval attraction;
};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}
