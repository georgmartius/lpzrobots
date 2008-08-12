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
 *   Revision 1.3  2008-08-12 11:45:29  guettler
 *   plug and play update, added some features for the ECBRobotGUI
 *
 *   Revision 1.2  2008/04/11 06:31:16  guettler
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

// overwritten from WiredController!
void ECBAgent::addPlotOption(const PlotOption& plotOption) {
  PlotOption po = plotOption;
  // if plotoption with the same mode exists -> delete it
  removePlotOption(po.mode);

  // this prevents the simulation to terminate if the child  closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN);
  po.open();
  if(po.pipe){
    // print start
    time_t t = time(0);
    fprintf(po.pipe,"# Start %s", ctime(&t));
    // print network description given by the structural information of the controller
    printNetworkDescription(po.pipe, "Selforg"/*controller->getName()*/, controller);
    // print ECBRobot specific infos
    char* infos = (char*) (ECBRobot*)robot)->getGuiInformation();
    fprintf(po.pipe,infos);
    // print interval
    fprintf(po.pipe, "# Recording every %dth dataset\n", po.interval);
    // print all configureables
    for(list<const Configurable*>::iterator i = po.configureables.begin(); i!= po.configureables.end(); i++){
      (*i)->print(po.pipe, "# ");
    }
    // print all parameters of the wiring if confirable
    Configurable* c = dynamic_cast<Configurable*>(wiring);
    if(c) c->print(po.pipe, "# ");
    // print all parameters of the controller
    controller->print(po.pipe, "# ");
    // print head line with all parameter names
    unsigned int snum = plotOption.whichSensors == Robot ? rsensornumber : csensornumber;
    printInternalParameterNames(po.pipe, snum, cmotornumber, inspectables);
  }
  plotOptions.push_back(po);
}

}
