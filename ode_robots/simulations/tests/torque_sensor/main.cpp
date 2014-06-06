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

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

// used robot
#include <ode_robots/fourwheeled.h>

#include <ode_robots/torquesensor.h>
#include <ode_robots/angularmotor.h>
#include <ode_robots/joint.h>

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
  TorqueSensor* ts;
  std::list<Joint*> joints;
  std::list<Primitive*> primitives;
  AngularMotor* amotor;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    global.odeConfig.setParam("noise",0);

    bool fixed=false;
    bool hinge=false;
    bool universalfixed=false;
    bool universal=false;
    bool vehicle=true;

    value=0;
    value2=0;

    if(fixed){
      Box* b1 = new Box(1,1,1);
      Box* b2 = new Box(1,1,1);
      Box* b3 = new Box(1,1,1);
      b1->init(odeHandle,1, osgHandle, Primitive::Geom | Primitive::Draw);
      b2->init(odeHandle,1, osgHandle);
      b3->init(odeHandle,1, osgHandle);
      b1->setPose(TRANSM(0,0,0.5));
      b2->setPose(TRANSM(0,0,1.7)); // *ROTM(M_PI/10,1,0,0));
      b3->setPose(TRANSM(-0.5,-0.5,4));
      primitives.push_back(b1);
      primitives.push_back(b2);
      primitives.push_back(b3);
      FixedJoint* j = new FixedJoint(b2,b1,(b1->getPosition()+b2->getPosition())*0.5);
      j->init(odeHandle,osgHandle,true);
      joints.push_back(j);
      ts = new TorqueSensor(1);
      ts->init(b1, j);
    }

    if(hinge){
      Box* b1 = new Box(1,1,1);
      Box* b2 = new Box(1,1,1);
      b1->init(odeHandle,1, osgHandle, Primitive::Geom | Primitive::Draw);
      b2->init(odeHandle,1, osgHandle);
      Pose m = ROTM(M_PI/10,0,0,1);
      b1->setPose(TRANSM(0,0,3)*m);
      b2->setPose(TRANSM(0,0,1)*m); // *ROTM(M_PI/10,1,0,0));
      primitives.push_back(b1);
      primitives.push_back(b2);
      HingeJoint* j = new HingeJoint(b2,b1,(b1->getPosition()+b2->getPosition())*0.5,
                                     Axis(1,0,0)*m);
      j->init(odeHandle,osgHandle,true);
      joints.push_back(j);
      ts = new TorqueSensor(1);
      ts->init(b1,j);

      amotor = new AngularMotor1Axis(odeHandle,j,10);
    }

    // bug of amotor with fixed object
    if(universalfixed){
      Box* b1 = new Box(1,1,1);
      Box* b2 = new Box(1,1,1);
      b1->init(odeHandle,1, osgHandle, Primitive::Geom | Primitive::Draw);
      b2->init(odeHandle,1, osgHandle);
      Pose m = ROTM(M_PI/10,0,0,1);
      b1->setPose(TRANSM(0,0,3)*m);
      b2->setPose(TRANSM(0,0,1)*m); // *ROTM(M_PI/10,1,0,0));
      primitives.push_back(b1);
      primitives.push_back(b2);
      UniversalJoint* j = new UniversalJoint(b2,b1,(b1->getPosition()+b2->getPosition())*0.5,
                                             Axis(0,1,0)*m, Axis(1,0,0)*m);
      j->init(odeHandle,osgHandle,true);
      joints.push_back(j);
      ts = new TorqueSensor(1);
      ts->init(b1,j);

      amotor = new AngularMotor2Axis(odeHandle,j,10,10);
    }

    if(universal){
      Box* b1 = new Box(1,1,1);
      Box* b2 = new Box(1,1,1);
      b1->init(odeHandle,1, osgHandle);
      b2->init(odeHandle,10, osgHandle);
      Pose m = ROTM(M_PI/10,0,0,1);
      b1->setPose(TRANSM(0,0,4)*m);
      b2->setPose(TRANSM(0,0,0.5)*m); // *ROTM(M_PI/10,1,0,0));
      primitives.push_back(b1);
      primitives.push_back(b2);
      UniversalJoint* j = new UniversalJoint(b2,b1,Pos(0,0,3)*m,
                                             Axis(0,1,0)*m, Axis(1,0,0)*m);
      j->init(odeHandle,osgHandle,true);
      joints.push_back(j);
      ts = new TorqueSensor(10,8);
      ts->init(b1,j);

      amotor = new AngularMotor2Axis(odeHandle,j,10,10);
    }



    if(vehicle){
      // use FourWheeled vehicle as robot:
      FourWheeledConf fc = FourWheeled::getDefaultConf();
      fc.twoWheelMode = true;
      fc.useBumper    = false;
      fc.irFront      = true;
      FourWheeled* fw = new FourWheeled(odeHandle, osgHandle,
                                        fc, "TestVehicle");
      ts = new TorqueSensor(1);
      // attach to main primitive (does not matter) and to 0th joint
      fw->addSensor(std::shared_ptr<Sensor>(ts), Attachement(-1,0));
      fw->place(osg::Matrix::translate(0,0,0));
      global.configs.push_back(fw);

      AbstractController *controller = new SineController();
      controller->setParam("period",300);
      controller->setParam("phaseshift",0.);
      global.configs.push_back(controller);

      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

      OdeAgent* agent = new OdeAgent(global);
      agent->init(controller, fw, wiring);
      global.agents.push_back(agent);
    }
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    FOREACH(std::list<Joint*>, joints,j){
      (*j)->update();
    }
    FOREACH(std::list<Primitive*>, primitives,p){
      (*p)->update();
    }
    ts->sense(globalData);
    if(control){
      std::list<sensor> ss = ts->getList();
      printf("Sensor: ");
      FOREACHC(std::list<sensor>, ss,s){
        printf("\t%f",*s);
      }
      printf("\n");
    }
  };


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'k':
          value+=.1;
          if(amotor)
            amotor->set(0,value);
          printf("value0: %f\n", value);
          break;
        case 'K':
          value-=.1;
          if(amotor)
            amotor->set(0,value);
          printf("value0: %f\n", value);
          break;
        case 'i':
          value2+=.1;
          if(amotor)
            amotor->set(1,value2);
          printf("value1: %f\n", value2);
          break;
        case 'I':
          value2-=.1;
          if(amotor)
            amotor->set(1,value2);
          printf("value1: %f\n", value2);
          break;
        default:
          return false;
          break;
        }
    }
    return false;
  }



};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}

