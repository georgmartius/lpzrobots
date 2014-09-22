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

#include "agent.h"
#include <signal.h>
#include <assert.h>
#include <time.h>
#include <string.h>

#include "abstractrobot.h"
#include "abstractcontroller.h"
#include "abstractwiring.h"

#include "callbackable.h"

using namespace std;

Agent::Agent(const PlotOption& plotOption, double noisefactor, const iparamkey& name, const paramkey& revision)
  : WiredController(plotOption, noisefactor, name, revision) {
  robot      = 0;
  rsensors=0; rmotors=0;
}


Agent::Agent(const std::list<PlotOption>& plotOptions, double noisefactor, const iparamkey& name, const paramkey& revision)
  : WiredController(plotOptions, noisefactor, name, revision){
  robot      = 0;
  rsensors=0; rmotors=0;
}

Agent::~Agent(){
  // closes all pipes of the agents due to pause mode or so
  trackrobot.close();
  if(robot) delete robot;
  if(rsensors) free(rsensors);
  if(rmotors)  free(rmotors);
}


bool Agent::init(AbstractController* controller, AbstractRobot* robot,
                 AbstractWiring* wiring, long int seed){
  this->robot   = robot;
  assert(robot);

  if(!seed) seed=rand();
  randGen.init(seed);

  rsensornumber = robot->getSensorNumber();
  rmotornumber  = robot->getMotorNumber();
  rsensors      = (sensor*) malloc(sizeof(sensor) * rsensornumber);
  rmotors       = (motor*)  malloc(sizeof(motor)  * rmotornumber);
  memset(rsensors,0, sizeof(motor)*rsensornumber);
  memset(rmotors,0, sizeof(motor)*rmotornumber);

  // add robot to inspectables
  Inspectable* in = dynamic_cast<Inspectable*>(robot);
  if(in) addInspectable(in);
  addConfigurable(robot);
  plotEngine.setName(robot->getName());
  setName(robot->getName() + "'s Agent");
  setNameOfInspectable(getName());

  return WiredController::init(controller,wiring, rsensornumber, rmotornumber,
                               robot->getSensorInfos(), robot->getMotorInfos(), &randGen);
}


void Agent::step(double noise, double time){
  assert(robot && rsensors && rmotors);

  int len =  robot->getSensors(rsensors, rsensornumber);
  if(len != rsensornumber){
    fprintf(stdout, "%s:%i: Got not enough sensors, expected %i, got %i!\n", __FILE__, __LINE__,
            rsensornumber, len);
  }

  WiredController::step(rsensors,rsensornumber, rmotors, rmotornumber, noise, time);
  robot->setMotors(rmotors, rmotornumber);
  trackrobot.track(robot, time);
}

// Sends only last motor commands again to robot.
void Agent::onlyControlRobot(){
  assert(robot && rmotors);
  robot->setMotors(rmotors, rmotornumber);
}


// sets the trackoptions which enable tracking of a robot
void Agent::setTrackOptions(const TrackRobot& trackrobot){
  this->trackrobot.close(); // close in case it is open.
  this->trackrobot = trackrobot;
  if(!robot){
    fprintf(stderr, "Agent.cpp: call setTrackOptions after init! <<<<<<<<<<<<<\n");
  }
  if (trackrobot.isTrackingSomething()){
    if(!this->trackrobot.open(robot)){
      fprintf(stderr, "Agent.cpp() ERROR: could not open trackfile! <<<<<<<<<<<<<\n");
    }else{
      // print all robot and wiring
      robot->print(this->trackrobot.file, "# ");
      Configurable* c = dynamic_cast<Configurable*>(wiring);
      if(c) c->print(this->trackrobot.file, "# ");
      // print all parameters of the controller to trackfile
      controller->print(this->trackrobot.file, "# ");
    }
  }
}

// sets the trackoptions which enable tracking of a robot
bool Agent::stopTracking(){
  bool rv =  trackrobot.isEnabled();
  trackrobot.close();
  return rv;
}
