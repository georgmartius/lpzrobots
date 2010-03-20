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
 *   Revision 1.1  2010-03-20 17:14:26  martius
 *   simulation with camera
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

#include <ode_robots/nimm2.h>
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
    setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));

    global.odeConfig.setParam("controlinterval",4);

    // use Playground as boundary:
    playground = new Playground(odeHandle, osgHandle,
				osg::Vec3(10, .2, 1));
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


    // this robot will get a camera 
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, Nimm2::getDefaultConf(), "Seeing_Robot");
    CameraConf camc = Camera::getDefaultConf();
    camc.width = 256;
    camc.height = 64;
    camc.scale  = 1;
    camc.fov    =  90;
    camc.camSize = 0.08;
    camc.processors.push_back(new HSVImgProc(false,1));
    //    camc2.processors.push_back(new BWImageProcessor(true,1, BWImageProcessor::Saturation));
    camc.processors.push_back(new ColorFilterImgProc(true,.5, 
                                                      HSVImgProc::Red+20, HSVImgProc::Green-20,100));
    //    camc.processors.push_back(new LineImgProc(true,10, 16));    
    camc.processors.push_back(new LineImgProc(true,20, 2, 5.0));    
    //    camc.processors.push_back(new AvgImgProc(true,20, 15));    
    Camera* cam = new Camera(camc);
    CameraSensor* camsensor = 
      new DirectCameraSensor(cam, odeHandle, osgHandle.changeColor(Color(0.2,0.2,0.2)), 
                             osg::Matrix::rotate(M_PI/2,0,0,1)*osg::Matrix::translate(-0.15,0,0.45));
    std::list<Sensor*> sensors;
    sensors.push_back(camsensor);
    // we put the camerasensor now onto the robot (which does not support it by itself)
    OdeRobot* robot = new AddSensors2RobotAdapter( odeHandle, osgHandle, vehicle, sensors);
    robot->place(Pos(-3,-1,0.3));
    //AbstractController *controller = new InvertMotorSpace(10);
    //AbstractController *controller = new SineController();
    AbstractController *controller = new Braitenberg(Braitenberg::Aggressive, 2, 3);
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(plotoptions);
    agent->init(controller, robot, wiring);
    global.configs.push_back(controller);
    global.agents.push_back(agent);

    {
      // this robot has no camera
      OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, Nimm2::getDefaultConf(), "Robot");
      vehicle->setColor(Color(1,1,0));
      vehicle->place(Pos(-3,2,0.3));
      AbstractController *controller = new InvertMotorSpace(10);
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      OdeAgent* agent = new OdeAgent();
      agent->init(controller, vehicle, wiring);
      global.agents.push_back(agent);
    }

    showParams(global.configs);
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

