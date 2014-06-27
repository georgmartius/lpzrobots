/***************************************************************************
 *   Copyright (C) 2014 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    der@informatik.uni-leipzig.de                                        *
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

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

// used robot
#include <ode_robots/fourwheeled.h>

#include <ode_robots/contactsensor.h>

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
  double value;
  double value2;
  std::list<Primitive*> primitives;
  std::list<Sensor*> sensors;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    global.odeConfig.setParam("noise",0);

    value=0;
    value2=0;

    bool balls=true;

    if(balls){
      // normal
      ContactSensor* cs;
      Sphere* b1 = new Sphere(1);
      b1->init(odeHandle,1, osgHandle);
      b1->setPose(TRANSM(0,0,2));
      primitives.push_back(b1);
      cs = new ContactSensor(false,60);
      cs->setInitData(odeHandle, osgHandle,TRANSM(0,0,-1));

      cs->init(b1);
      sensors.push_back(cs);

      // binary
      Sphere* b2 = new Sphere(1);
      b2->init(odeHandle,1, osgHandle);
      b2->setPose(TRANSM(3,0,2));
      primitives.push_back(b2);
      cs = new ContactSensor(true);
      cs->setInitData(odeHandle, osgHandle,TRANSM(0,0,-1));
      cs->init(b2);
      sensors.push_back(cs);

      // with sphere
      Sphere* b3 = new Sphere(1);
      b3->init(odeHandle,1, osgHandle);
      b3->setPose(TRANSM(6,0,2));
      primitives.push_back(b3);
      cs = new ContactSensor(false, 10, 0.1, true);
      cs->setInitData(odeHandle, osgHandle,TRANSM(0,0,-1));
      cs->init(b3);
      sensors.push_back(cs);
    }
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    FOREACH(std::list<Primitive*>, primitives,p){
      (*p)->update();
    }

    printf("-------------------\n");
    FOREACHIa( sensors, s, i){
      (*s)->sense(globalData);
      (*s)->update();
      std::list<sensor> ss = (*s)->getList();
      printf("Sensor %i: ", i);
      for(auto val: ss){
        printf("\t%f",val);
      }
      printf("\n");
    }
  };
};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}
