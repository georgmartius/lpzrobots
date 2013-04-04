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

using namespace osg;
using namespace std;

namespace lpzrobots {

  AddSensors2RobotAdapter::
  AddSensors2RobotAdapter(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                          OdeRobot* robot, const std::list<Sensor*>& sensors,
                          const std::list<Motor*>& motors,
                          bool sensors_before_rest)
    : OdeRobot(odeHandle, osgHandle, robot->getName(), robot->getRevision()),
      robot(robot), sensors(sensors), sensors_before_rest(sensors_before_rest),
      initialized(false), askedfornumber(0)
  {
    assert(robot);
    Configurable::addConfigurable(robot);
    Inspectable* i = dynamic_cast<Inspectable*>(robot);
    if(i) Inspectable::addInspectable(i);

    //    copyParameters(*robot);
  };


  AddSensors2RobotAdapter::~AddSensors2RobotAdapter(){
    FOREACH(list<Sensor*>, sensors, i){
      if(*i) delete *i;
    }
    sensors.clear();
    delete robot;
  }

  void AddSensors2RobotAdapter::addSensor(Sensor* sensor){
    assert(!askedfornumber);
    assert(sensor);
    if(initialized){
      Primitive* p = robot->getMainPrimitive();
      sensor->init(p);
    }
    sensors.push_back(sensor);
  }


  void AddSensors2RobotAdapter::addMotor(Motor* motor){
    assert(!askedfornumber);
    assert(motor);
    if(initialized){
      Primitive* p = robot->getMainPrimitive();
      motor->init(p);
    }
    motors.push_back(motor);
  }

  void AddSensors2RobotAdapter::update(){
    assert(initialized);
    robot->update();
    FOREACHC(list<Sensor*>, sensors, i){
      (*i)->update();
    }
  }

  int AddSensors2RobotAdapter::getSensorNumber(){
    assert(initialized);
    int s=0;
    FOREACHC(list<Sensor*>, sensors, i){
      s += (*i)->getSensorNumber();
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
    FOREACH(list<Sensor*>, sensors, i){
      len += (*i)->get(sensors_ + len, sensornumber - len);
    }
    if(sensors_before_rest){
      len += robot->getSensors(sensors_ + len,sensornumber - len);
    }

    return len;
  };

  int AddSensors2RobotAdapter::getMotorNumber() {
    assert(initialized);
    int s=0;
    FOREACHC(list<Motor*>, motors, i){
      s += (*i)->getMotorNumber();
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
    FOREACH(list<Motor*>, motors, i){
      len += (*i)->set(motors_ + len, motornumber - len);
    }

  }

  void AddSensors2RobotAdapter::place(const osg::Matrix& pose){
    robot->place(pose);
    Primitive* p = robot->getMainPrimitive();
    FOREACH(list<Sensor*>, sensors, i){
      (*i)->init(p);
    }
    FOREACH(list<Motor*>, motors, i){
      (*i)->init(p);
    }
    initialized=true;
  }


  void AddSensors2RobotAdapter::sense(GlobalData& globalData){
    assert(initialized);
    FOREACH(list<Sensor*>, sensors, i){
      (*i)->sense(globalData);
    }
    robot->sense(globalData);
  }

  void AddSensors2RobotAdapter::doInternalStuff(GlobalData& globalData){
    assert(initialized);
    FOREACH(list<Motor*>, motors, i){
      (*i)->act(globalData);
    }
    robot->doInternalStuff(globalData);
  }

  void AddSensors2RobotAdapter::notifyOnChange(const paramkey& key){
    if(initialized && robot)
      robot->notifyOnChange(key);
  }


}

