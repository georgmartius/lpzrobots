#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <assert.h>

#include "simulation.h"

#include "shortcircuit.h"


ShortCircuit::ShortCircuit(const OdeHandle& odeHandle, int sensornumber, int motornumber)
  : AbstractRobot::AbstractRobot(odeHandle){

  sensorno = sensornumber; 
  motorno  = motornumber;  
  motors = (motor*)malloc(motorno * sizeof(motor));
  for(int i=0; i < motorno; i++){
    motors[i]=0.0;
  }
  
};

/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void ShortCircuit::setMotors(const motor* _motors, int motornumber){
  assert(motornumber == motorno);
  memcpy(motors, _motors, sizeof(motor) * motornumber);
};

/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int ShortCircuit::getSensors(sensor* sensors, int sensornumber){
  assert(sensornumber == sensorno);  
  int mini = min(sensorno,motorno); 
  for (int i=0; i< mini; i++){
    sensors[i]=motors[i]; // %motorno
  }
  for (int i=mini; i< sensorno; i++){
    sensors[i]=0;
  }
  return sensorno;
};

/** returns position of robot 
    @return position robot position in struct Position  
*/
Position ShortCircuit::getPosition(){
  Position pos;
  pos.x=0;
  pos.y=0;
  pos.z=0;
  return pos;
};

/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int ShortCircuit::getSegmentsPosition(vector<Position> &poslist){
  return 0;
};  

/**
 * draws the vehicle
 */
void ShortCircuit::draw(){};







