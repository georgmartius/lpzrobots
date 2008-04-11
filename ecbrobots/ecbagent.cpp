/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    marcoeckert@web.de                                                   *
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
 *   $Log$
 *   Revision 1.2  2008-04-11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#include "ecbagent.h"

#include <selforg/abstractwiring.h>
#include <selforg/abstractcontroller.h>

using namespace std;

namespace lpzrobots {


ECBAgent::~ECBAgent() {
  // TODO: cleanup!
/*  delete (*robot);
  delete *controller;
  delete *wiring;

  Agent::~Agent();
  WiredController::~WiredController();*/
}

/** initializes the object with the given controller, robot and wiring
      and initializes the output options
  */
bool ECBAgent::init(AbstractController* controller, ECBRobot* robot, AbstractWiring* wiring) {


  this->robot      = robot;
  assert(robot);

  rsensornumber = robot->getMaxSensorNumber();
  rmotornumber  = robot->getMaxMotorNumber();
  rsensors      = (sensor*) malloc(sizeof(sensor) * rsensornumber);
  rmotors       = (motor*)  malloc(sizeof(motor)  * rmotornumber);

  // add robot to inspectables
  inspectables.push_back(robot);


  this->controller = controller;
  this->wiring     = wiring;
  assert(controller && wiring);

  wiring->init(rsensornumber, rmotornumber);
  csensornumber = wiring->getControllerSensornumber();
  cmotornumber  = wiring->getControllerMotornumber();
  controller->init(csensornumber, cmotornumber);

  csensors      = (sensor*) malloc(sizeof(sensor) * csensornumber);
  cmotors       = (motor*)  malloc(sizeof(motor)  * cmotornumber);

  inspectables.push_back(controller);
  inspectables.push_back(wiring);

  // copy plotoption list and add it one by one
  list<PlotOption> po_copy(plotOptions);
  plotOptions.clear();
  // open the all outputs
  for(list<PlotOption>::iterator i=po_copy.begin(); i != po_copy.end(); i++){
    addPlotOption(*i);
  }
  initialised = true;

  return true;
}


//  Performs an step of the agent, including sensor reading, pushing sensor values through wiring,
//  controller step, pushing controller steps back through wiring and sent resulting motorcommands
//  to robot.
//  @param noise Noise strength.
//  @param time (optional) current simulation time (used for logging)
void ECBAgent::step(double noise, double time){
  Agent::step(noise,time);
  ((ECBRobot*)robot)->writeMotors_readSensors();
}

}
