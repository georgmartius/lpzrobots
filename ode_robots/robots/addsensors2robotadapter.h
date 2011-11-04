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
#ifndef __ADDSENSORS2ROBOTADAPTER__
#define __ADDSENSORS2ROBOTADAPTER__

#include "oderobot.h"
#include "sensor.h"
#include "motor.h"

namespace lpzrobots {

  /** Robot adapter to add sensors and also motors to robots without the
      need to modify the robot itself. 
      Examples are Speed sensors, microphons and beepers and so forth            
      // TODO: add inspectable, make configurable work!
      // Maybe it is easier to add this into OdeRobot itself!
  */
  class AddSensors2RobotAdapter : public OdeRobot {
  public:
  
    /**
     * constructor of adapter
     * @param robot robot the wrap and plug sensors in
     * @param sensors list of sensors to add
     * @param motors list of motors to add
     */
    AddSensors2RobotAdapter( const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
			     OdeRobot* robot, 
			     const std::list<Sensor*>& sensors = std::list<Sensor*>(),
			     const std::list<Motor*>& motors  = std::list<Motor*>(), 
			     bool sensors_before_rest = false);

    virtual ~AddSensors2RobotAdapter();
    
    /// adds a sensor to the robot. Must be called before placement of the robot, otherwise it has no affect
    virtual void addSensor(Sensor* sensor);

    /// adds a motor to the robot. Must be called before placement of the robot, otherwise it has no affect
    virtual void addMotor(Motor* motor);

    virtual void update();

    virtual void place(const osg::Matrix& pose);

    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2){
      return robot->collisionCallback(data,o1,o2);
    }

    virtual void setColor(const Color& col) { 
      robot->setColor(col); 
    }

    virtual int getSensorNumber();
    virtual int getSensors(sensor* sensors_, int sensornumber);

    virtual int getMotorNumber();
    virtual void setMotors(const motor* motors_, int motornumber);

    void sense(GlobalData& globalData);
    void doInternalStuff(GlobalData& globalData);

    virtual Primitive* getMainPrimitive() const { return robot->getMainPrimitive();}
    virtual std::vector<Primitive*> getAllPrimitives() const { return robot->getAllPrimitives();}    
      
  protected:
    OdeRobot* robot;
    std::list<Sensor*> sensors;
    std::list<Motor*> motors;
    bool sensors_before_rest;
    bool initialized;
    bool askedfornumber;
  };

}

#endif
