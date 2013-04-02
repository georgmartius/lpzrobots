#include "basiccontroller.h"

using namespace std;
using namespace matrix;


BasicController::BasicController(const std::string& name, const std::string& revision) : AbstractController(name, revision) {
  initialised=false;
}


void BasicController::init(int sensornumber, int motornumber, RandGen* randGen) {
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

void BasicController::step(const sensor* sensors, int sensornumber,
    motor* motors, int motornumber) {

  if ((sensors[2] > .5) && (sensors[3] <= .5)) {
    motors[0] = .2;
    motors[1] = 1.0;
  }
  else if ((sensors[2] <= .5) && (sensors[3] > .5)) {
    motors[0] = 1.0;
    motors[1] = .2;
  }
  else if ((sensors[2] > .5) && (sensors[3] > .5)) {
    motors[0] = -1.0;
    motors[1] = 1.0;
  }
  else {
    motors[0] = 1.0;
    motors[1] = 1.0;
  }

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

