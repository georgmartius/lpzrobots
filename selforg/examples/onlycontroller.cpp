#include<iostream>
#include<vector>
using namespace std;

#include "selforg/sinecontroller.h" 
#include "selforg/invertmotorspace.h" 


void myrobot(const sensor* sensors, motor* motors){  
    sensors[0]=motors[0]+(double(rand())/RAND_MAX-0.5)*0.3;
    sensors[1]=motors[1]+(double(rand())/RAND_MAX-0.5)*0.3;
    //    sensors[2]=motors[1]+(double(rand())/RAND_MAX-0.5)*0.3;
}


int main(){
  
  AbstractController* controller = new InvertMotorSpace(10);
  controller->init(2,2); // initialise with 2 motors and 2 sensors
  controller->setParam("epsA",0.01); // set parameter epsA (learning rate for Model A)
  controller->print(stderr,0); // print parameters (see Configurable) to stderr

  
  // sensor and motor arrays (doubles*)
  sensor sensors[2] = {0.1,0.3};
  motor motors[2];

  // example no robot (short circuit)
  for(int i=0; i < 50000; i++){
    myrobot(sensors,motors);

    cout << i << " S0: " << sensors[0] << ", " <<" S1: " << sensors[1];
    list<Inspectable::iparamval> list_ = controller->getInternalParams();  
    vector<Inspectable::iparamval> v(list_.begin(), list_.end());  
    cout << i << " C00: " << v[4] << ", " <<" C01: " << v[5] << ", " <<
      " C10: " << v[6] << ", " <<" C11: " << v[7]  << endl;

    controller->step(sensors, 2, motors, 2);    
    cout << i << " Motor values: " << motors[0] << ", " << motors[1] << endl;
  }
  return 0;
}


