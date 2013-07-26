/***************************************************************************
 *   Copyright (C) 2005-2013 LpzRobots development team                    *
 *   Authors:                                                              *
 *    Simón Smith <artificialsimon at ed dot ac dot uk>                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
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

#include "basiccontroller.h"

#include <assert.h>
using namespace std;
using namespace matrix;

BasicController::BasicController(const std::string& name, const std::string& revision)
  : AbstractController(name, revision) {
  initialised=false;
  // add threshold parameter to configurable parameters, setable on console
  addParameterDef("threshold", &threshold, 0.2, 0, 1, "threshold for IR-sensor");
}

void BasicController::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors) {
  // Very simple controller, if IR sensors are higher than threshold then
  // the robot will turn oposite direction, or backward when to close
  // Sensors index 0 and 1 are wheel speeds
  // Sensors index from 2 to 4 are left front IR sensors
  // from 5 to 7 are the font left infra red sensors (4,5 being in the center)
  // from 8 and 9 are the back sensors
  if (sensors[4] > 2*threshold || sensors[5] > 2*threshold) { // move backward
    motors[0] = -1.;
    motors[1] = -1.;
  }else if (sensors[2] > threshold || sensors[3] > threshold || sensors[4] > threshold) {
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

void BasicController::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber) {
  stepNoLearning(sensors,sensornumber, motors, motornumber);
}


void BasicController::init(int sensornumber, int motornumber, RandGen* randGen) {
  assert(motornumber >=2 && sensornumber >=8);
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

bool BasicController::store(FILE* f) const {
  return true;
}

bool BasicController::restore(FILE* f) {
  return true;
}

