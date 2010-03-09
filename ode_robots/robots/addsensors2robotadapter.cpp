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
 *   $Log$
 *   Revision 1.4  2010-03-09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.3  2007/11/07 13:20:16  martius
 *   also motors can be added
 *
 *   Revision 1.2  2007/09/06 18:47:59  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.1  2007/08/24 11:49:06  martius
 *   initial
 *
 *                                                                 *
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
    robot(robot), sensors(sensors), sensors_before_rest(sensors_before_rest)  
  {  
    assert(robot);
  };


  AddSensors2RobotAdapter::~AddSensors2RobotAdapter(){
    FOREACH(list<Sensor*>, sensors, i){
      if(*i) delete *i;
    }    
    sensors.clear();
    delete robot;
  }

  void AddSensors2RobotAdapter::addSensor(Sensor* sensor){
    assert(sensor); 
    sensors.push_back(sensor);
  }


  void AddSensors2RobotAdapter::addMotor(Motor* motor){
    assert(motor);
    motors.push_back(motor); 
  }


  int AddSensors2RobotAdapter::getSensorNumber(){ 
    int s=0;
    FOREACHC(list<Sensor*>, sensors, i){
      s += (*i)->getSensorNumber();
    }
    return robot->getSensorNumber() + s;
  }

  int AddSensors2RobotAdapter::getSensors(sensor* sensors_, int sensornumber){
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
    int s=0;
    FOREACHC(list<Motor*>, motors, i){
      s += (*i)->getMotorNumber();
    } 
    return robot->getMotorNumber() + s;
  }

  void AddSensors2RobotAdapter::setMotors(const motor* motors_, int motornumber) {
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
  }

  void AddSensors2RobotAdapter::doInternalStuff(GlobalData& globalData){
    FOREACH(list<Sensor*>, sensors, i){
      (*i)->sense(globalData);
    }
    FOREACH(list<Motor*>, motors, i){
      (*i)->act(globalData);
    }
    robot->doInternalStuff(globalData);
  }

}

