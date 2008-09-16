#include<assert.h>
#include<iostream>
#include<vector>
using namespace std;

#include <selforg/sinecontroller.h>
#include <selforg/invertmotorspace.h>

const int MNumber = 2;
const int SNumber = 2;

/** The robot control should go here
    @param sensors list of sensor values (to be written) (doubles)
    @param sensornumber length of the sensors vector
    @param motors list of motor values (doubles) (to send to the robot)
    @param motornumber length of the motors vector

 */
void myrobot(double* sensors, int sensornumber, const double* motors, int motornumber){  
  assert(sensornumber >= 2 && motornumber >= 2); // check dimensions
  //the robot consits here just of a bit noise
  sensors[0]=motors[0]+(double(rand())/RAND_MAX-0.5)*0.3;
  sensors[1]=motors[1]+(double(rand())/RAND_MAX-0.5)*0.3;

}


int main(){

  AbstractController* controller = new InvertMotorSpace(10);
  controller->init(2,2); // initialise with 2 motors and 2 sensors
  controller->setParam("epsA",0.01); // set parameter epsA (learning rate for Model A)
  controller->print(stderr,0); // print parameters (see Configurable) to stderr

  
  // sensor and motor arrays (doubles*)
  double sensors[SNumber];
  double motors[MNumber];
  
  memset(motors,0,sizeof(double)*MNumber);  // clear motors

  // the robot is here respresented by the function myrobot
  for(int i=0; i < 1000; i++){
    // call robot with motors and receive sensors 
    myrobot(sensors, SNumber, motors, MNumber);
    cout << i << " S0: " << sensors[0] << ", " <<" S1: " << sensors[1];

    // print some internal parameters of the controller
    list<Inspectable::iparamval> list_ = controller->getInternalParams();  
    vector<Inspectable::iparamval> v(list_.begin(), list_.end());  
    cout << i << " C00: " << v[4] << ", " <<" C01: " << v[5] << ", " <<
      " C10: " << v[6] << ", " <<" C11: " << v[7]  << endl;
    
    // call controller with sensors and receive motors (both dimension 2)    
    controller->step(sensors, SNumber, motors, MNumber); 
    cout << i << " Motor values: " << motors[0] << ", " << motors[1] << endl;
  }
  delete controller;
  return 0;
}


