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
 *   Revision 1.3  2010-03-30 14:33:05  martius
 *   nice working version for fourwheeled
 *
 *   Revision 1.2  2010/03/29 17:19:59  martius
 *   added two wheeled
 *
 *   Revision 1.1  2010/03/29 16:26:45  martius
 *   new simulation for Ball playing
 *
 *   Revision 1.3  2010/03/29 07:17:36  martius
 *   nimm4 body inverted
 *
 *   Revision 1.2  2010/03/26 14:17:15  martius
 *   fourwheeled in 2wheeled mode addded
 *
 *   Revision 1.1  2010/03/25 16:38:07  martius
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
#include <selforg/crossmotorcoupling.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/selectivenoisewiring.h>

#include "semoxhebmod.h"

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
#include <ode_robots/addsensors2robotadapter.h>
#include <ode_robots/fourwheeled.h>


#include <osg/Light>
#include <osg/LightSource>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

template<typename T> 
std::vector<T> mkVector(const T* v, int len){
  return std::vector<T>(v,v+len);  
}

class ThisSim : public Simulation {
public:

  AbstractController* controller;
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
      setCameraHomePos(Pos(-7.38466, 10.1764, 3.17434),  Pos(-96.9417, -12.6582, 0));
    else
      setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));

    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("noise",0.05);

    addParameterDef("attraction", &attraction, 0.000);
    addParameterDef("friction", &friction, 1);

    addParameterDef("sizefactor", &sizefactor, .5);
    addParameterDef("posfactor", &posfactor, 1);

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
      wallHandle.substance.toSnow(.3);
      OctaPlayground* outer = new OctaPlayground(wallHandle, osgHandle.changeAlpha(0.2), 
                                                 osg::Vec3(radius+2, 0.2, 1), 12);
      outer->setTexture("");
      outer->setPosition(osg::Vec3(0,0,0.1));
      outer->setGroundSubstance(Substance(0.4,0.005,40,0.5));
      global.obstacles.push_back(outer);
      // inner walls (without ground
      OctaPlayground* inner = new OctaPlayground(wallHandle, osgHandle.changeColor(0.1,0.4,0.1), 
                                                 osg::Vec3(radius-2, 0.2, 1), 12, false);
      inner->setTexture("");
      inner->setPose(osg::Matrix::rotate(M_PI/12,0,0,1) * osg::Matrix::translate(0,0,0.1));
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


    /// TWOWHEELED
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
            
      twc.camSensor     = new MotionCameraSensor(2, MotionCameraSensor::Position | 
                                                       MotionCameraSensor::Size | 
                                                       MotionCameraSensor::SizeChange);

      OdeRobot* vehicle = new TwoWheeled(odeHandle, osgHandle, twc, 
                                         "CamRobotTwo_" + itos(i));
      vehicle->setColor(Color(1,.7,0));
      if(useCorridor) 
        vehicle->place(osg::Vec3(sin(i/2.0-1)*radius,cos(i/2.0-1)*radius,0.3));
      else
        vehicle->place(osg::Matrix::rotate(M_PI, 0,0,1) 
                       * osg::Matrix::translate(3,-4+2*i,0.3));

      SeMoXHebModConf cc = SeMoXHebMod::getDefaultConf();
      cc.modelExt = true;
      SeMoXHebMod *semox = new SeMoXHebMod(cc);
      controller = semox;
//       CrossMotorCoupling* controller = new CrossMotorCoupling(semox, semox, 0.0);
//       std::list<int> perm;
//       perm += 1;
//       perm += 0;
//       controller->setCMC(CrossMotorCoupling::getPermutationCMC(perm));        
//       controller->setParam("gamma_teach",0.005);
      controller->setParam("gamma_teach",0.01);
      //  controller->setParam("rootE",3);
      
      AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
      OdeAgent* agent = new OdeAgent( i==0 ? plotoptions : std::list<PlotOption>(),0.1);
      agent->init(controller, vehicle, wiring);
      global.configs.push_back(controller);
      global.agents.push_back(agent);
    }


    /// FOURWHEELED
    for(int i=0; i<numSeeing4wheeled; i++){
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
      CameraSensor* camSensor = new MotionCameraSensor(2, MotionCameraSensor::Position | 
                                                       MotionCameraSensor::Size | 
                                                       MotionCameraSensor::SizeChange);
      camSensor->setInitData(cam, odeHandle, osgHandle, osg::Matrix::rotate(-M_PI/2,0,0,1)
			     * osg::Matrix::translate(0.2,0, 0.40) );
      std::list<Sensor*> sensors;
      sensors.push_back(camSensor);      
      FourWheeledConf fwc = FourWheeled::getDefaultConf();      
      fwc.twoWheelMode = true;
      fwc.useBumper    = false;
      fwc.force        = 5;
      OdeRobot* robot = new FourWheeled(odeHandle, osgHandle, 
                                        fwc, 
                                        "4WCamRobot_" + itos(i));
      OdeRobot* vehicle = new AddSensors2RobotAdapter(odeHandle, osgHandle, robot, sensors);
      vehicle->setColor(Color(1,.7,0));
      if(useCorridor) 
        vehicle->place(osg::Vec3(sin(i/2.0+.5)*radius,cos(i/2.0+.5)*radius,0.3));
      else
        vehicle->place(osg::Matrix::rotate(M_PI, 0,0,1) 
                       * osg::Matrix::translate(3,-4+2*i,0.3));

      
      SeMoXHebModConf cc = SeMoXHebMod::getDefaultConf();
      cc.modelExt = true;
      SeMoXHebMod *semox = new SeMoXHebMod(cc);
      controller = semox;
//       controller = new SineController();
//       CrossMotorCoupling* controller = new CrossMotorCoupling(semox, semox, 0.0);
//       std::list<int> perm;
//       perm += 1;
//       perm += 0;
//       controller->setCMC(CrossMotorCoupling::getPermutationCMC(perm));        
//       controller->setParam("gamma_teach",0.005);
      controller->setParam("gamma_teach",0.005);
      //  controller->setParam("rootE",3);
      
      double noise[] = {.5,.5};      
      AbstractWiring* wiring = new SelectiveNoiseWiring(new WhiteUniformNoise(),
                                                        mkVector(noise,2));
      
      //    AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
      OdeAgent* agent = new OdeAgent( i==0 ? plotoptions : std::list<PlotOption>());
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

    showParams(global.configs);
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control){
      SeMoXHebMod* semox = dynamic_cast<SeMoXHebMod*>(controller);
      if(semox){
        matrix::Matrix desired = semox->getLastSensorValues();
        // size: \dot size = -(size - 1) // set point is 1
        desired.val(5,0) += - (desired.val(4,0)-1)* sizefactor;
        // desired.val(5,0) += - (desired.val(4,0)-0.3)* sizefactor;// set point is 0.3
        // position: \dot pos = -(pos) // set point is 0
        desired.val(2,0) += - (desired.val(3,0)) * posfactor;
        semox->setSensorTeaching(desired);
      }
    }
    // ball friction and move all balls to robot
    Pos rpos = (*globalData.agents.begin())->getRobot()->getPosition();
    FOREACH(ObstacleList, globalData.obstacles, o){
      PassiveSphere* s = dynamic_cast<PassiveSphere*>(*o);
      if(s){
        Pos spos = s->getMainPrimitive()->getPosition();
        s->getMainPrimitive()->applyForce((rpos-spos)*attraction);
        Pos svel = s->getMainPrimitive()->getVel();
        s->getMainPrimitive()->applyForce(-svel*friction);        

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
  paramval friction;
  paramval sizefactor;
  paramval posfactor;
};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}

