#include "basiccontroller.h"

using namespace std;
using namespace matrix;

void BasicController::step(const sensor* sensors, int sensornumber,
    motor* motors, int motornumber) {
  // Very simple controller, if IR sensors are higher than threshold then
  // the robot will turn oposite direction
  double threshold = 0.1;
  // Sensors index 0 and 1 are wheel speeds
  // Sensors index from 2 to 4 are left front IR sensors
  // from 5 to 9 are the font left infra red sensors.
  if (sensors[2] > threshold || sensors[3] > threshold || sensors[4] > threshold) {
    motors[0] = .1;
    motors[1] = 1.;
  }
  else if (sensors[5] > threshold || sensors[6] > threshold || sensors[7] > threshold) {
    motors[0] = 1.;
    motors[1] = .1;
  }
  else { // Move forward
    motors[0] = 1.;
    motors[1] = 1.;
  }
}

BasicController::BasicController(const std::string& name, const std::string& revision) : AbstractController(name, revision) {
  initialised=false;
}


void BasicController::init(int sensornumber, int motornumber, RandGen* randGen) {
  // Set the number of sensors and motors
  nSensors = sensornumber;
  nMotors  = motornumber;
  initialised=true;
}


int BasicController::getSensorNumber() const {
  return nSensors;
}

int BasicController::getMotorNumber() const {
  return nMotors;
}

void BasicController::stepNoLearning(const sensor* , int number_sensors,
    motor* , int number_motors) {
}

bool BasicController::store(FILE* f) const {
  return true;
}

bool BasicController::restore(FILE* f) {
  return true;
}

