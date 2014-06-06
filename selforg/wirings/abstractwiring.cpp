/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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

#include "abstractwiring.h"
#include "stl_adds.h"



bool AbstractWiring::init(int robotsensornumber, int robotmotornumber, RandGen* _randGen){
  rsensornumber = robotsensornumber;
  rmotornumber  = robotmotornumber;
  noisenumber   = rsensornumber;
  randGen       = _randGen;
  noisenumber   = rsensornumber;
  bool rv= initIntern();

  mNoise.set(noisenumber,1);
  noisevals = (double*) mNoise.unsafeGetData(); // hack! we let the noiseval pointer point to the internal memory of the noisematrix.

  if(noiseGenerator)
    noiseGenerator->init(noisenumber, randGen);

  mRsensors.set(rsensornumber,1);
  mRmotors.set(rmotornumber,1);
  mCsensors.set(csensornumber,1);
  mCmotors.set(cmotornumber,1);
  if(plotMode & Controller) {
    addInspectableMatrix("x", &mCsensors, false, "sensor values");
    addInspectableMatrix("y", &mCmotors,  false, "motor values (controller output)");
  }
  if(plotMode & Robot) {
    addInspectableMatrix("x_R", &mRsensors, false, "bare sensor values");
    addInspectableMatrix("y_R", &mRmotors,  false, "motor values (as send to robot)");
  }
  if(plotMode & Noise) {
    addInspectableMatrix("n", &mNoise, false, "sensor noise");
  }
  // add sensor names and motor names
  //  std::list<std::string> sn = wireSensorNames()


  initialised = true;
  return rv;
}

#define ADDNAMES(infos, prefix) { FOREACHIa(infos, i, index) { \
      if(!i->name.empty()) \
        addInspectableDescription(std::string(prefix) +"[" + std::itos(index) + "]", i->name); }}


/// used by WiredController to pass infos to inspectable
void AbstractWiring::addSensorMotorInfosToInspectable(const std::list<SensorMotorInfo>& robotSensorInfos,
                                                      const std::list<SensorMotorInfo>& robotMotorInfos,
                                                      const std::list<SensorMotorInfo>& controllerSensorInfos,
                                                      const std::list<SensorMotorInfo>& controllerMotorInfos){
  if(plotMode & Controller) {
    ADDNAMES(controllerSensorInfos, "x");
    ADDNAMES(controllerMotorInfos,  "y");
  }
  if(plotMode & Robot) {
    ADDNAMES(robotSensorInfos, "x");
    ADDNAMES(robotMotorInfos,  "y");
  }
}


bool AbstractWiring::wireSensors(const sensor* rsensors, int rsensornumber,
                                 sensor* csensors, int csensornumber,
                                 double noiseStrength){
  assert(initialised);
  if(noiseGenerator) {
    memset(noisevals, 0 , sizeof(sensor) * noisenumber);
    noiseGenerator->add(noisevals, noiseStrength);
  }
  bool rv = wireSensorsIntern(rsensors, rsensornumber, csensors, csensornumber, noiseStrength);
  mRsensors.set(rsensors);
  mCsensors.set(csensors);
  return rv;
}

bool AbstractWiring::wireMotors(motor* rmotors, int rmotornumber,
                                const motor* cmotors, int cmotornumber){
  assert(initialised);
  bool rv = wireMotorsIntern(rmotors, rmotornumber, cmotors, cmotornumber);
  mRmotors.set(rmotors);
  mCmotors.set(cmotors);
  return rv;
}

std::list<SensorMotorInfo> AbstractWiring::wireSensorInfos(const std::list<SensorMotorInfo>& robotSensorInfos){
  return robotSensorInfos;
}

std::list<SensorMotorInfo> AbstractWiring::wireMotorInfos(const std::list<SensorMotorInfo>& robotMotorInfos){
  return robotMotorInfos;
}


