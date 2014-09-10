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

/* Simple lpz controller stub for using ROS  */

#include <ros/ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float64MultiArray.h>

#include <sstream>
#include <unistd.h>
#include "roscontroller.h"

ROSController::ROSController(const std::string& name)
  : AbstractController(name,"0.1") {
}

ROSController::~ROSController(){}

void ROSController::init(int sensornumber, int motornumber, RandGen* randGen){
  char * p=0;
  int argc=0;
  ros::init(argc, &p, getName());

  ros::NodeHandle n;

  sensor_pub = n.advertise<std_msgs::Float64MultiArray>("sensors", 1);
  motor_sub  = n.subscribe("/motors", 1, &ROSController::motorsCallback, this);
  motorValues = (motor*)malloc(sizeof(motor)*motornumber);
  memset(motorValues,0,sizeof(double)*motornumber);

  gotmotor = false;
}

void ROSController::motorsCallback(const std_msgs::Float64MultiArray::ConstPtr& motormsg) {
  // std::cerr << "got something: [" << motormsg->data[0] << ", " << motormsg->data[1] << "]" << std::endl;
  // std::cerr << "data size: " << motormsg->data.size() << ", " << number_motors << std::endl;
  int len=std::min((int)motormsg->data.size(), number_motors);
  for(int k=0;k<len;k++){
    motorValues[k] = motormsg->data[k];
    // std::cout << motorValues[k] << std::endl;
  }
  gotmotor = true;
}

void ROSController::step(const sensor* sensors, int sensornumber,
                        motor* motors, int motornumber){

  if(!ros::ok()) {
    return;
    std::cerr << "Ros connection error" << std::endl;
  }

  if(number_motors == 0) {
    number_motors = motornumber;
  }

  std_msgs::Float64MultiArray msg;
  //   msg.layout.dim_length = 1;
  msg.data.clear();
  for(int k=0;k<sensornumber;k++){
    msg.data.push_back(sensors[k]);
  }
  // msg.data = std::vector<double>(sensors[0], sensors[sensornumber-1]);
  // memcpy(msg.data,sensors, msg.data);

  sensor_pub.publish(msg);
  // spin once so to harvest incoming data
  ros::spinOnce();
  // here we need to wait
  // while(!gotmotor) {}
  // (FIXME) give the external controller 10ms to compute motor response
  // to the stimulus
  usleep(10000);

  memcpy(motors,motorValues,sizeof(double)*motornumber);
  // gotmotor = false;
}
void ROSController::stepNoLearning(const sensor* s, int number_sensors,
                                  motor* m, int number_motors){
  step(s, number_sensors, m, number_motors);
}
