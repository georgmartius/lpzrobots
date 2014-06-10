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

#include "wiredcontroller.h"


#include <signal.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <algorithm>

#include "abstractcontroller.h"
#include "abstractwiring.h"

#include "motorbabbler.h"

#include "callbackable.h"

using namespace std;

WiredController::WiredController(const PlotOption& plotOption, double noisefactor, const iparamkey& name, const paramkey& revision)
  : Inspectable(name), Configurable(name, revision), noisefactor(noisefactor), plotEngine(plotOption) {
  internInit();
}


WiredController::WiredController(const std::list<PlotOption>& plotOptions, double noisefactor, const iparamkey& name, const paramkey& revision)
  : Inspectable(name), Configurable(name, revision), noisefactor(noisefactor), plotEngine(plotOptions) {
  internInit();
}

void WiredController::internInit(){
  controller = 0;
  wiring     = 0;

  cmotors=0;
  csensors=0;

  motorBabbler = 0;
  motorBabblingSteps = 0;

  t=1;
  initialised = false;
}

WiredController::~WiredController(){
  if(controller) delete controller;
  if(wiring) delete wiring;

  if(csensors) free(csensors);
  if(cmotors)  free(cmotors);
  if(motorBabbler) delete motorBabbler;
}


/// initializes the object with the given controller, robot and wiring
//  and initializes pipe to guilogger
bool WiredController::init(AbstractController* controller, AbstractWiring* wiring,
                           int robotsensornumber, int robotmotornumber,
                           const std::list<SensorMotorInfo>& robotSensorInfos,
                           const std::list<SensorMotorInfo>& robotMotorInfos,
                           RandGen* randGen){
  this->controller = controller;
  this->wiring     = wiring;
  assert(controller && wiring);

  rsensornumber = robotsensornumber;
  rmotornumber  = robotmotornumber;
  wiring->init(rsensornumber, rmotornumber, randGen);
  csensornumber = wiring->getControllerSensornumber();
  cmotornumber  = wiring->getControllerMotornumber();
  controller->init(csensornumber, cmotornumber, randGen);

  csensors      = (sensor*) malloc(sizeof(sensor) * csensornumber);
  cmotors       = (motor*)  malloc(sizeof(motor)  * cmotornumber);

  // wire infos and send to controller and for plotting
  const std::list<SensorMotorInfo>& controllerSensorInfos = wiring->wireSensorInfos(robotSensorInfos);
  const std::list<SensorMotorInfo>& controllerMotorInfos  = wiring->wireMotorInfos( robotMotorInfos);

  controller->sensorInfos(controllerSensorInfos);
  controller->motorInfos(controllerMotorInfos);
  wiring->addSensorMotorInfosToInspectable(robotSensorInfos, robotMotorInfos,
                                           controllerSensorInfos, controllerMotorInfos);

  plotEngine.addInspectable(this, true);
  plotEngine.addConfigurable(this);
  addInspectable(wiring);
  addInspectable(controller);
  addConfigurable(controller);

  Configurable* c_wiring = dynamic_cast<Configurable*>(wiring);
  if(c_wiring) addConfigurable(c_wiring);

  plotEngine.init(controller);

  initialised = true;
  return true;
}

void WiredController::startMotorBabblingMode (int steps, AbstractController* babblecontroller){
  if(babblecontroller){
    if(motorBabbler) delete motorBabbler;
    motorBabbler = babblecontroller;
    motorBabbler->init(csensornumber, cmotornumber);
  }else{
    if(!motorBabbler) {
      motorBabbler = new MotorBabbler();
      motorBabbler->init(csensornumber, cmotornumber);
    }
  }
  motorBabblingSteps=steps;
}

PlotOption WiredController::addPlotOption(PlotOption& plotOption) {
  return plotEngine.addPlotOption(plotOption);
}

bool WiredController::addAndInitPlotOption(PlotOption& plotOption) {
  return plotEngine.addAndInitPlotOption(plotOption);
}

bool WiredController::removePlotOption(PlotMode mode){
  return plotEngine.removePlotOption(mode);
}

void WiredController::writePlotComment(const char* cmt, bool addSpace){
  plotEngine.writePlotComment(cmt, addSpace);
}


// Plots controller sensor- and motorvalues and internal controller parameters.
void WiredController::plot(double time){
  plotEngine.plot(time);
};




//  Performs an step of the wired controller:
//   pushing sensor values through wiring,
//  controller step, pushing controller steps back through wiring
void WiredController::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber,
                           double noise, double time){
  assert(controller && wiring && sensors && csensors && cmotors && motors);

  if(sensornumber != rsensornumber){
    fprintf(stderr, "%s:%i: Got wrong number of sensors, expected %i, got %i!\n", __FILE__, __LINE__,
            rsensornumber, sensornumber);
  }

  wiring->wireSensors(sensors, rsensornumber, csensors, csensornumber, noise * noisefactor);
  if(motorBabblingSteps>0){
    motorBabbler->step(csensors, csensornumber, cmotors, cmotornumber);
    controller->motorBabblingStep(csensors, csensornumber, cmotors, cmotornumber);
    motorBabblingSteps--;
    if(motorBabblingSteps==0) stopMotorBabblingMode();
  }else{
    controller->step(csensors, csensornumber, cmotors, cmotornumber);
  }
  wiring->wireMotors(motors, rmotornumber, cmotors, cmotornumber);
  plot(time);
  // do a callback for all registered Callbackable classes
  callBack();
  t++;
}
