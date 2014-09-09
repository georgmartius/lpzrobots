#include <ros/ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float64MultiArray.h>

#include <sstream>
#include <unistd.h>
#include "roscontroller.h"

// void callback(const std_msgs::Float64MultiArrayPtr& msg) {
//   // std::cerr << "got something" << std::endl;
//   ROS_INFO("I heard: [%f]", msg->data[0]);
// }

ROSController::ROSController(const std::string& name)
  : AbstractController(name,"0.1") {
}

ROSController::~ROSController(){}

void ROSController::init(int sensornumber, int motornumber, RandGen* randGen){
  // char * p=0;
  char* p = {"blub"};
  int argc=1;
  // ros::init(argc, &p, getName());
  ros::init(argc, &p, "blub");

  ros::NodeHandle n;

  // motor_sub  = n.subscribe("motors", 1000, callback);
  sensor_pub = n.advertise<std_msgs::Float64MultiArray>("sensors", 1);
  motor_sub  = n.subscribe("/motors", 1, &ROSController::motorsCallback, this);
  motorValues = (motor*)malloc(sizeof(motor)*motornumber);
  memset(motorValues,0,sizeof(double)*motornumber);

  gotmotor = false;
}

void ROSController::motorsCallback(const std_msgs::Float64MultiArray::ConstPtr& motormsg) {
  std::cerr << "got something: [" << motormsg->data[0] << ", " << motormsg->data[1] << "]" << std::endl;
  std::cerr << "data size: " << motormsg->data.size() << ", " << number_motors << std::endl;
  int len=std::min((int)motormsg->data.size(), number_motors);
  for(int k=0;k<len;k++){
    motorValues[k] = motormsg->data[k];
    std::cout << motorValues[k] << std::endl;
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

  // std::cout << motor_sub << std::endl;

  std_msgs::Float64MultiArray msg;
  //   msg.layout.dim_length = 1;
  msg.data.clear();
  for(int k=0;k<sensornumber;k++){
    msg.data.push_back(sensors[k]);
  }
    //msg.data = std::vector<double>(sensors[0], sensors[sensornumber-1]);
  //  memcpy(msg.data,sensors, msg.data);

  sensor_pub.publish(msg);
  // spin once so to harvest incoming data
  ros::spinOnce();
  // here we need to wait
  // while(!gotmotor) {}
  usleep(10000);

  memcpy(motors,motorValues,sizeof(double)*motornumber);
  // gotmotor = false;
}
void ROSController::stepNoLearning(const sensor* s, int number_sensors,
                                  motor* m, int number_motors){
  step(s, number_sensors, m, number_motors);
}


