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
 *
 ***************************************************************************/
#include <stdio.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

#include <ode_robots/speedsensor.h>
#include <ode_robots/relativepositionsensor.h>

// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class ThisSim : public Simulation {
public:
  double value;
  double value2;
  std::list<SpeedSensor*> ss;
  std::list<Primitive*> primitives;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(7.75018, -4.30236, 3.88123),  Pos(65.2963, -18.5703, 0));
    bool speed=true;

    if(speed){
      Box* b1 = new Box(0.5,0.2,0.1);
      Box* b2 = new Box(0.5,0.2,0.1);
      Box* b3 = new Box(0.5,0.2,0.1);
      b1->init(odeHandle,1, osgHandle);
      b2->init(odeHandle,1, osgHandle);
      b3->init(odeHandle,1, osgHandle);
      b1->setPose(TRANSM(0,0,2));
      b2->setPose(ROTM(M_PI/2,0,1,0) * TRANSM(1,0,2));
      b3->setPose(ROTM(M_PI/4,1,0,0) * TRANSM(2,0,2));
      primitives.push_back(b1);
      primitives.push_back(b2);
      primitives.push_back(b3);
      SpeedSensor* s;
      s = new SpeedSensor(1.0, SpeedSensor::Translational);    s->init(b1); ss += s;
      s = new SpeedSensor(1.0, SpeedSensor::TranslationalRel); s->init(b1); ss += s;
      s = new SpeedSensor(1.0, SpeedSensor::Translational);    s->init(b2); ss += s;
      s = new SpeedSensor(1.0, SpeedSensor::TranslationalRel); s->init(b2); ss += s;
      s = new SpeedSensor(1.0, SpeedSensor::Translational);    s->init(b3); ss += s;
      s = new SpeedSensor(1.0, SpeedSensor::TranslationalRel); s->init(b3); ss += s;
    }

  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    FOREACH(std::list<Primitive*>, primitives,p){
      (*p)->update();
    }
    FOREACH(std::list<SpeedSensor*>, ss,s){
      (*s)->sense(globalData);
    }
    if(globalData.sim_step==20){
      int i=0;
      if(control){
        FOREACH(std::list<SpeedSensor*>, ss,s){
          std::list<sensor> vals = (*s)->get();
          printf("Sensor %i: ", i);
          FOREACHC(std::list<sensor>, vals,v){
            printf("\t%f",*v);
          }
          printf("\n");
          i++;
        }
      }
    }
    if(globalData.sim_step>40){
      simulation_time_reached=true;
    }

  };

};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}

