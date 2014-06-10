/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
 *    mai00bvz@studserv.uni-leipzig.de                                     *
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

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include <selforg/controller_misc.h>

// used robot
#include <ode_robots/fourwheeled.h>

#include <ode_robots/speaker.h>
#include <ode_robots/soundsensor.h>

// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>

// used controller
#include <selforg/sinecontroller.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  Speaker* speaker;
  double value;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    global.odeConfig.setParam("noise",0);

    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(6, 0.2, 0.5));
    playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
    // register playground in obstacles list
    global.obstacles.push_back(playground);

    // use FourWheeled vehicle as robot:
    FourWheeledConf fc = FourWheeled::getDefaultConf();
    fc.twoWheelMode = true;
    fc.useBumper    = false;
    fc.irFront      = false;
    FourWheeled* vehicle = new FourWheeled(odeHandle, osgHandle,
                                        fc, "TestVehicle");

    vehicle->addSensor(std::make_shared<SoundSensor>(Sensor::X, SoundSensor::Angle, 1, 1, 5));
    vehicle->place(osg::Matrix::translate(0,0,0));
    global.configs.push_back(vehicle);

    AbstractController *controller = new SineController();
    controller->setParam("period",300);
    controller->setParam("phaseshift",0.);
    global.configs.push_back(controller);

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);


    PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle);
    s->setPose(osg::Matrix::translate(-1,-1,1));
    global.obstacles.push_back(s);
    speaker = new Speaker(10);
    speaker->init(s->getMainPrimitive());
    value=0.5;
    speaker->set(&value,1);



  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control)
      speaker->act(globalData);
  };


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'k':
          value+=.1;
          speaker->set(&value,1);
          std::cout << "louder" << std::endl;;
          break;
        case 'K':
          value-=.1;
          speaker->set(&value,1);
          std::cout << "quiter" << std::endl;
          break;
        default:
          return false;
          break;
        }
    }
    value=clip(value,0.0,1.0);
    return false;
  }



};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}
