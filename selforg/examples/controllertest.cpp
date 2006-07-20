#include<iostream>
#include<vector>
using namespace std;

#include <selforg/sinecontroller.h>
#include <selforg/invertnchannelcontroller.h>

int main(){
  AbstractController* controller = new InvertNChannelController(5 );
  controller->init(2,2);
  controller->print(stderr,0);
  return 0;
  sensor sensors[3] = {0.1,-0.1,0.3};
  motor motors[2];
  // short circuit
  for(int i=0; i < 5; i++){
    sensors[0]=motors[0]+(double(rand())/RAND_MAX-0.5)*0.3;
    sensors[1]=motors[1]+(double(rand())/RAND_MAX-0.5)*0.3;
    sensors[2]=motors[1]+(double(rand())/RAND_MAX-0.5)*0.3;
    controller->stepNoLearning(sensors, 2, motors, 2);    
  }
  for(int i=0; i < 50000; i++){
    sensors[0]=motors[0]+(double(rand())/RAND_MAX-0.5)*0.3;
    sensors[1]=motors[1]+(double(rand())/RAND_MAX-0.5)*0.3;
    sensors[2]=motors[1]+(double(rand())/RAND_MAX-0.5)*0.3;
    //    cout << i << " S0: " << sensors[0] << ", " <<" S1: " << sensors[1];
    list<Inspectable::iparamval> list_ = controller->getInternalParams();  
    vector<Inspectable::iparamval> v(list_.begin(), list_.end());  
//     cout << i << " C00: " << list_[6] << ", " <<" C01: " << list_[7] << ", " <<
//       " C02: " << list_[8] << ", " <<" C10: " << list_[9] << ", " <<
//       " C11: " << list_[10] << ", " <<" C12: " << list_[11] << endl;
    cout << i << " C00: " << v[4] << ", " <<" C01: " << v[5] << ", " <<
      " C10: " << v[6] << ", " <<" C11: " << v[7]  << endl;

    controller->step(sensors, 2, motors, 2);    
    cout << i << " Motor values: " << motors[0] << ", " << motors[1] << endl;
  }
  cout << "Motor values: " << motors[0] << ", " << motors[1] << endl;
  return 0;
}


