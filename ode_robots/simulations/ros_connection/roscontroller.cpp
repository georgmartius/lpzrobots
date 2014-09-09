#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <sstream>
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
}

void ROSController::motorsCallback(const std_msgs::Float64MultiArray::ConstPtr& motormsg) {
  std::cerr << "got something" << std::endl;
  int len=std::min((int)motormsg->data.size(),number_motors);
  for(int k=0;k<len;k++){
    motorValues[k]=motormsg->data[k];
  }
}

void ROSController::step(const sensor* sensors, int sensornumber,
                        motor* motors, int motornumber){

  if(!ros::ok()) {
    return;
    std::cerr << "Ros connection error" << std::endl;
  }

  std_msgs::Float64MultiArray msg;
  //   msg.layout.dim_length = 1;
  msg.data.clear();
  for(int k=0;k<sensornumber;k++){
    msg.data.push_back(sensors[k]);
  }
    //msg.data = std::vector<double>(sensors[0], sensors[sensornumber-1]);
  //  memcpy(msg.data,sensors, msg.data);

  sensor_pub.publish(msg);

  // here we need to wait

  // ros::spinOnce();
  memcpy(motors,motorValues,sizeof(double)*motornumber);
}
void ROSController::stepNoLearning(const sensor* s, int number_sensors,
                                  motor* m, int number_motors){
  step(s, number_sensors, m, number_motors);
}


