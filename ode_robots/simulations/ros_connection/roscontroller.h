/***************************************************************************
 *  Copyright (C) 2014                                                     *
 *    Oswald Berthold                                                      *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *  part of LpzRobots                                                      *
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
#ifndef __ROSCONTROLLER_H
#define __ROSCONTROLLER_H

#include <stdio.h>
#include <selforg/abstractcontroller.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>

/**
 * class for robot control via ROS
 *
 */
class ROSController : public AbstractController {
public:
  /**
     @param port Port number to listen for controller
     @param robotname name of robot to send to controller
   */
  ROSController(const std::string& name);

  virtual ~ROSController();

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual int getSensorNumber() const {return number_sensors;}
  virtual int getMotorNumber() const {return number_motors;}

  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber);
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  void motorsCallback(const std_msgs::Float64MultiArray::ConstPtr& motormsg);

  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    Configurable::parse(f);
    return true;
  }

protected:
  int number_sensors;
  int number_motors;
  bool gotmotor;
  ros::Publisher sensor_pub;
  ros::Subscriber motor_sub;
  motor* motorValues;
};

#endif
