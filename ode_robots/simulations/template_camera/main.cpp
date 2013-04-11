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
 *   Revision 1.9  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.8  2010/03/25 16:39:51  martius
 *   primitive has addForce/addTorque function
 *
 *   Revision 1.7  2010/03/24 16:51:38  martius
 *   QuickMP uses now the number of processors in the system
 *   optical flow improved
 *   video recording works with offscreen rendering
 *   Make system: Optimization -O1 is switched on by default (add a debug version without optimization)
 *
 *   Revision 1.6  2010/03/23 18:43:54  martius
 *   lpzviewer: added checking function
 *   camerasensor new initialization
 *   twowheeled allows full customization of camera
 *   optical flow improved multiscale check
 *
 *   Revision 1.5  2010/03/23 07:32:48  martius
 *   global colorbased optical flow is now called motioncamerasensor
 *   optical flow camerasensor started
 *
 *   Revision 1.4  2010/03/22 14:33:19  martius
 *   osghandle changeColor() with single rgba values
 *   camerasensors windowfunction bug
 *
 *   Revision 1.3  2010/03/21 21:48:59  martius
 *   camera sensor bugfixing (reference to osghandle)
 *   twowheeled robot added (nimm2 with camera)
 *   sense function added to robots (before control): sensors (type Sensor) are checked here
 *   position and optical flow camera sensors added
 *
 *   Revision 1.2  2010/03/21 10:19:56  martius
 *   test_graphics moved to tests/graphics
 *
 *   Revision 1.1  2010/03/20 17:14:26  martius
 *   simulation with camera
 *
 ***************************************************************************/
#include <stdio.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>

#include <selforg/braitenberg.h>

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
#include <ode_robots/camerasensors.h>
#include <ode_robots/opticalflow.h>

#include <ode_robots/nimm2.h>
#include <ode_robots/twowheeled.h>
#include <ode_robots/addsensors2robotadapter.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:


  AbstractObstacle* playground;


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    bool twowheeled=false;
    bool nimm2vision=true;
    bool nimm2=true;

    setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));

    global.odeConfig.setParam("controlinterval",4);

    // use Playground as boundary:
    playground = new Playground(odeHandle, osgHandle,
                                osg::Vec3(10, .2, 1.4));
    playground->setTexture("Images/wall.rgb");
    playground->setPosition(osg::Vec3(0,0,0.1));
    global.obstacles.push_back(playground);

    // add passive spheres as obstacles
    for (int i=0; i< 1/*2*/; i+=1){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(1,1,0)), 0.3);
      // s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      s1->setPosition(osg::Vec3(0,0,1+i*5));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }

    if(twowheeled) {
      // the twowheeled robot is derived from Nimm2 and has a camera onboard
      //  we can change the imageprocessing and the camerasensor in the cfg
      TwoWheeledConf twc = TwoWheeled::getDefaultConf();
      // here we can change e.g. the image processing
      //    // for example we could exchange the light detection sensor to have 4 sensors
      //    twc.camcfg.processors.pop_back(); // remove the standard LineImgProc
      //    // and add a new one
      //    twc.camcfg.processors.push_back(new LineImgProc(true,20, 4);
      //    set a different camerasensor
      //    twc.camSensor = new MotionCameraSensor(3);
      OdeRobot* vehicle = new TwoWheeled(odeHandle, osgHandle, twc, "Twowheeled");
      vehicle->setColor(Color(1,.7,0));
      vehicle->place(osg::Matrix::rotate(M_PI, 0,0,1)*osg::Matrix::translate(3,4,0.3));
      //      AbstractController *controller = new InvertMotorSpace(10);
      AbstractController *controller = new Braitenberg(Braitenberg::Aggressive, 2, 3);
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      OdeAgent* agent = new OdeAgent();
      agent->init(controller, vehicle, wiring);
      global.agents.push_back(agent);
    }

    if(nimm2vision){
      // This code shows how to add a camera to an existing robot.
      // Here we have the most flexibility on which camera sensor we use and so on
      OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, Nimm2::getDefaultConf(),
                                    "Seeing_Nimm2");
      CameraConf camc = Camera::getDefaultConf();
      camc.width = 256;
      camc.height = 128;
      camc.scale  = 1;
      camc.fov    =  120;
      camc.camSize = 0.08;
      camc.processors.push_back(new HSVImgProc(false,1));
      //    camc2.processors.push_back(new BWImageProcessor(true,1, BWImageProcessor::Saturation));
      camc.processors.push_back(new ColorFilterImgProc(true,.5,
                                                       HSVImgProc::Red+20,
                                                       HSVImgProc::Green-20,100));
      CameraSensor* camsensor;
      int sensorType = 4;
      switch(sensorType) {
      case 1: /// Left and right side brighness (of Yellow)
        camc.processors.push_back(new LineImgProc(true,20, 2));
        //camc.processors.push_back(new AvgImgProc(true,20, 15));
        camsensor = new DirectCameraSensor();
        break;
      case 2: /// Using the position of Yellow object
        camsensor = new PositionCameraSensor(PositionCameraSensor::PositionAndSize);
        break;
      case 3: /// Motion detection (global optical flow if Yellow object(s))
        camsensor = new MotionCameraSensor(3, MotionCameraSensor::PositionAndSize);
        break;
      case 4: /// Optical flow of raw image
        camc.processors.clear(); // no preprocessing
        OpticalFlowConf ofc = OpticalFlow::getDefaultConf();
        ofc.points = OpticalFlow::getDefaultPoints(1);
        ofc.dims   = Sensor::X;
        ofc.verbose = 3;
        ofc.maxFlow = 0.12;
        ofc.fieldSize = 24;
        camsensor = new OpticalFlow(ofc);
        break;
      default:
        assert(0);
      }
      Camera* cam = new Camera(camc);
      osg::Matrix camPos = osg::Matrix::rotate(M_PI/2,0,0,1)
        * osg::Matrix::translate(-0.2,0,0.40);
      OsgHandle camOsgHandle = osgHandle.changeColor(Color(0.2,0.2,0.2));
      camsensor->setInitData(cam, odeHandle, camOsgHandle,camPos);
      std::list<Sensor*> sensors;
      sensors.push_back(camsensor);
      // we put the camerasensor now onto the robot (which does not support it by itself)
      OdeRobot* robot = new AddSensors2RobotAdapter( odeHandle, osgHandle, vehicle, sensors);
      //      robot->place(Pos(-3,-1,0.3));
      robot->place(osg::Matrix::rotate(M_PI, 0,0,1)*osg::Matrix::translate(3,0,0.3));
      //AbstractController *controller = new InvertMotorSpace(10);
      AbstractController *controller = new SineController();
      controller->setParam("amplitude",0.4);
      controller->setParam("phaseshift",2);
      controller->setParam("period",300);
      //AbstractController *controller = new Braitenberg(Braitenberg::Aggressive, 2, 3);
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      OdeAgent* agent = new OdeAgent(plotoptions,0); // no noise
      agent->init(controller, robot, wiring);
      global.configs.push_back(controller);
      global.agents.push_back(agent);
    }

    if(nimm2){
      // this robot has no camera
      OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, Nimm2::getDefaultConf(), "Robot");
      vehicle->setColor(Color(1,1,0));
      vehicle->place(Pos(3,2,0.3));
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
};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}

