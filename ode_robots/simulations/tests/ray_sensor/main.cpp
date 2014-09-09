/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
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
 ***************************************************************************/
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>

#include <ode_robots/passivebox.h>
#include <ode_robots/raysensor.h>
#include <ode_robots/irsensor.h>


using namespace lpzrobots;
using namespace osg;
using namespace std;


class ThisSim : public Simulation
{
public:
  PassiveBox* b1;
  PassiveBox* b2;
  RaySensor* rs;
  IRSensor* ir;

  ThisSim()
  {
    setTitle("Ray-Test");
    setCaption("Simulator by Martius et al");
  }


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-26.6849, 17.3789, 16.7798),  Pos(-120.46, -24.7068, 0));
    setCameraMode(Static);

    global.odeConfig.setParam("noise",0.0);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("gravity",0);

    Playground* playground = new Playground(odeHandle, osgHandle,
                                            osg::Vec3(20 , 0.2, .5));
    playground->setPosition(osg::Vec3(0,0,0));
    global.obstacles.push_back(playground);


    b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(1, 1, 1));
    b1->setPosition(Vec3(0,0,1.1));
    global.obstacles.push_back(b1);
    rs = new RaySensor(0.1,5, RaySensor::drawAll);
    rs->setInitData(odeHandle, osgHandle, ROTM(M_PI,0,1,0) * TRANSM(0,0,-.5));
    rs->init(b1->getMainPrimitive());

    b2 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(1, 1, 1));
    b2->setPosition(Vec3(2,0,1.1));
    global.obstacles.push_back(b2);
    ir = new IRSensor(1.0, 0.1,2, RaySensor::drawAll);
    ir->setInitData(odeHandle, osgHandle, ROTM(M_PI,0,1,0) * TRANSM(0,0,-.5));
    ir->init(b2->getMainPrimitive());

  }

  void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(globalData.sim_step < 4) return;
    rs->update();
    if(control){
      rs->sense(globalData);
    }
    if(rs->getList().front()!=1.0)
      cerr << "ERROR Ray: " << rs->getList().front() << "!= 1.0" <<endl;
    if(control){ // second call in the same timestep
      rs->sense(globalData);
      if(rs->getList().front()!=1.0)
      cerr << "ERROR Ray: " << rs->getList().front() << "!= 1.0 (second call)" <<endl;
    }

    ir->update();
    if(control){
      ir->sense(globalData);
    }
    double p_last = sin(0.5*(globalData.time-(globalData.odeConfig.simStepSize*globalData.odeConfig.controlInterval)));
    double v = ir->getList().front();
    double s = (2-(p_last+1))/2;
    if(fabs(v - s) > 10e-4){
      cerr << "ERROR IR:  " << v << "!= " << s <<  endl;
    }

    double p = sin(0.5*globalData.time);
    b2->setPosition(Vec3(2,0,p+1.1));
  }

  virtual void usage() const {
  };


};


int main (int argc, char **argv){
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
