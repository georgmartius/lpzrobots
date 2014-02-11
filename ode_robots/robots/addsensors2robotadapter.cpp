/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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

#include <ode-dbl/ode.h>
#include <assert.h>
#include <osg/Matrix>

#include "addsensors2robotadapter.h"
#include <ode_robots/torquesensor.h>

using namespace osg;
using namespace std;

namespace lpzrobots {

  AddSensors2RobotAdapter::
  AddSensors2RobotAdapter(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                          OdeRobot* robot, const std::list<Sensor*>& sensors,
                          const std::list<Motor*>& motors,
                          bool sensors_before_rest)
    : OdeRobot(odeHandle, osgHandle, robot->getName(), robot->getRevision()),
      robot(robot), sensors_before_rest(sensors_before_rest),
      initialized(false), askedfornumber(0)
  {
    assert(robot);
    Configurable::addConfigurable(robot);
    Inspectable* i = dynamic_cast<Inspectable*>(robot);
    if(i) Inspectable::addInspectable(i);

    for (auto &i: sensors){
      this->sensors.push_back(SensorAttachment(i,Attachement()));
    }
    for (auto &i: motors){
      this->motors.push_back(MotorAttachment(i,Attachement()));
    }
    //    copyParameters(*robot);
  };


  AddSensors2RobotAdapter::~AddSensors2RobotAdapter(){
    for(auto &i: sensors){
      if(i.first) delete i.first;
    }
    sensors.clear();
    delete robot;
  }

  void AddSensors2RobotAdapter::attachSensor(SensorAttachment& sa){
    Primitive* p;
    if(sa.second.primitiveIndex<0){
      p = robot->getMainPrimitive();
    } else {
      assert((int)robot->getAllPrimitives().size() > sa.second.primitiveIndex);
      p = robot->getAllPrimitives()[sa.second.primitiveIndex];
    }
    Joint* j=0;
    if(sa.second.jointIndex>=0){
      assert((int)robot->getAllJoints().size() > sa.second.jointIndex);
      j = robot->getAllJoints()[sa.second.jointIndex];
    }
    sa.first->init(p, j);
  }

  void AddSensors2RobotAdapter::attachMotor(MotorAttachment& ma){
    Primitive* p;
    if(ma.second.primitiveIndex<0){
      p = robot->getMainPrimitive();
    } else {
      assert((int)robot->getAllPrimitives().size() > ma.second.primitiveIndex);
      p = robot->getAllPrimitives()[ma.second.primitiveIndex];
    }
    // Todo: add joint to motors
    // Joint* j=0;
    // if(ma.second.jointIndex>=0){
    //   assert((int)robot->getAllJoints().size() > ma.second.jointIndex);
    //   j = robot->getAllJoints()[ma.second.jointIndex];
    // }
    ma.first->init(p);
  }

  void AddSensors2RobotAdapter::addSensor(Sensor* sensor, Attachement attachement){
    assert(!askedfornumber);
    assert(sensor);
    SensorAttachment sa(sensor,attachement);
    if(initialized){
      attachSensor(sa);
    }
    sensors.push_back(sa);
  }

  void AddSensors2RobotAdapter::addMotor(Motor* motor, Attachement attachement){
    assert(!askedfornumber);
    assert(motor);
    MotorAttachment ma(motor,attachement);
    if(initialized){
      attachMotor(ma);
    }
    motors.push_back(ma);
  }

  void AddSensors2RobotAdapter::addTorqueSensors(double maxtorque, int avg ){
    if(!initialized){
      cerr << "call addTorqueSensors() after robot->place()!";
      assert(initialized);
    }
    int numJoints = robot->getAllJoints().size();
    for(int j=0; j<numJoints; j++){
      addSensor(new TorqueSensor(maxtorque, avg), Attachement(-1,j));
    }
  }


  void AddSensors2RobotAdapter::update(){
    assert(initialized);
    robot->update();
    for(auto& i: sensors){
      i.first->update();
    }
  }

  int AddSensors2RobotAdapter::getSensorNumber(){
    assert(initialized);
    int s=0;
    for ( auto &i: sensors){
      s += i.first->getSensorNumber();
    }
    askedfornumber=true;
    return robot->getSensorNumber() + s;
  }

  int AddSensors2RobotAdapter::getSensors(sensor* sensors_, int sensornumber){
    assert(initialized);
    int len = 0;

    if(!sensors_before_rest){
      len += robot->getSensors(sensors_ + len,sensornumber - len);
    }
    for(auto& i: sensors){
      len += i.first->get(sensors_ + len, sensornumber - len);
    }
    if(sensors_before_rest){
      len += robot->getSensors(sensors_ + len,sensornumber - len);
    }

    return len;
  };

  int AddSensors2RobotAdapter::getMotorNumber() {
    assert(initialized);
    int s=0;
    for(auto& i: motors){
      s += i.first->getMotorNumber();
    }
    askedfornumber=true;
    return robot->getMotorNumber() + s;
  }

  void AddSensors2RobotAdapter::setMotors(const motor* motors_, int motornumber) {
    assert(initialized);
    int len = 0;
    assert(motornumber >= robot->getMotorNumber());
    robot->setMotors(motors_, robot->getMotorNumber());
    len += robot->getMotorNumber();
    for(auto& i: motors){
      len += i.first->set(motors_ + len, motornumber - len);
    }

  }

  void AddSensors2RobotAdapter::place(const osg::Matrix& pose){
    robot->place(pose);
    for( auto &i: sensors){
      attachSensor(i);
    }
    for( auto &i: motors){
      attachMotor(i);
    }
    initialized=true;
  }


  void AddSensors2RobotAdapter::sense(GlobalData& globalData){
    assert(initialized);
    for(auto& i: sensors){
      i.first->sense(globalData);
    }
    robot->sense(globalData);
  }

  void AddSensors2RobotAdapter::doInternalStuff(GlobalData& globalData){
    assert(initialized);
    for(auto &i: motors){
      i.first->act(globalData);
    }
    robot->doInternalStuff(globalData);
  }

  void AddSensors2RobotAdapter::notifyOnChange(const paramkey& key){
    if(initialized && robot)
      robot->notifyOnChange(key);
  }


}

