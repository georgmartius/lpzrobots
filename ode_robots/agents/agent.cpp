/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   Revision 1.4  2005-07-21 11:30:59  fhesse
 *   started with blind motors
 *
 *   Revision 1.3  2005/07/18 14:44:27  martius
 *   noise moved into wiring
 *
 *   Revision 1.2  2005/07/18 10:14:04  martius
 *   noise moved to wiring
 *
 *   Revision 1.1  2005/07/14 15:57:53  fhesse
 *   now agent contains controller, robot and wiring, plotting ability included, therefore plotagent can be removed; ono2onewiring replaces one2oneagent
 *
 *                                                                 *
 ***************************************************************************/

#include "agent.h"
#include <signal.h>
#include "printInternals.h"
#include "printInternals.cpp" // TODO: avoid this kind of hack



  /// constructor
Agent::Agent(PlotMode plotmode/*=GuiLogger*/){
  controller = 0;
  robot      = 0;
  wiring     = 0;
    
  rsensors=0; rmotors=0; 
  csensors=0; cmotors=0; 
  this->plotmode=plotmode;

  pipe=0;
  numberInternalParameters=0;
}
  
Agent::~Agent(){
  CloseGui();
  if(rsensors) free(rsensors);
  if(rmotors)  free(rmotors);
  if(csensors) free(csensors);
  if(cmotors)  free(cmotors);
}


/// initializes the object with the given controller, robot and wiring
//  and initializes pipe to guilogger
bool Agent::init(AbstractController* controller, AbstractRobot* robot, AbstractWiring* wiring){
  this->controller = controller;
  this->robot      = robot;
  this->wiring     = wiring;
  if(!controller || !robot || !wiring) return false;
  else{
    rsensornumber = robot->getSensorNumber();
    rmotornumber  = robot->getMotorNumber();
    wiring->init(rmotornumber, rsensornumber);
    csensornumber = wiring->getControllerSensornumber();
    cmotornumber  = wiring->getControllerMotornumber();
    controller->init(csensornumber, cmotornumber);

    rsensors      = (sensor*) malloc(sizeof(sensor) * rsensornumber);
    rmotors       = (motor*)  malloc(sizeof(motor)  * rmotornumber);
    csensors      = (sensor*) malloc(sizeof(sensor) * csensornumber);
    cmotors       = (motor*)  malloc(sizeof(motor)  * cmotornumber);
    
    if(plotmode != NoPlot){
      if(!OpenGui()) return false;
      numberInternalParameters = printInternalParameterNames(pipe, csensornumber, cmotornumber, controller);
    }
    
    return true;
  }
}


bool Agent::OpenGui(){
  // this prevents the simulation to terminate if the child (guilogger) closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN); 
  // TODO: get the guilogger call from some  config
  if(plotmode == GuiLogger_File){
    pipe=popen("guilogger -l -m pipe -d 15  > /dev/null","w");
  }else{
    pipe=popen("guilogger -m pipe -d 3 > /dev/null","w");
  }
  if(pipe==0){
    fprintf(stderr, "%s:%i: could not open guilogger!\n", __FILE__, __LINE__);    
    return false;
  }else return true;
}

void Agent::CloseGui(){
    if (pipe) pclose(pipe);
    pipe=0;
}

// Plots controller sensor- and motorvalues and internal controller parameters.
void Agent::plot(const sensor* x, int sensornumber, const motor* y, int motornumber){
  if(!controller || !x || !y || plotmode==NoPlot || !pipe) return;
  if(sensornumber!=controller->getSensorNumber()) {
    fprintf(stderr, "%s:%i: Given sensor number does not match the one from controller!\n", 
	    __FILE__, __LINE__);
  }
  if(motornumber!=controller->getMotorNumber()) { 
    fprintf(stderr, "%s:%i: Given motor number does not match the one from controller!\n", 
	    __FILE__, __LINE__);
  }
  
  printInternalParameters(pipe, x, sensornumber, y, motornumber, 
			  numberInternalParameters, controller);
  fflush(pipe);
};



/// Performs an step of the agent, including sensor reading, pushing sensor values through wiring, 
//  controller step, pushing controller steps back through wiring and sent resulting motorcommands to robot.
//  @param noise Noise strength.
void Agent::step(double noise){

  if(!controller || !robot || !wiring || !rsensors || !rmotors || !csensors || !cmotors) {
    fprintf(stderr, "%s:%i: something is null: cont %x rob %x wiring %x rsens %x rmots %x csens %x cmots %x!\n", 
	    __FILE__, __LINE__, (unsigned int)controller, (unsigned int)robot,  
	    (unsigned int)wiring, (unsigned int)rsensors, (unsigned int)rmotors, 
	    (unsigned int)csensors, (unsigned int)cmotors);
  }
  
  int len =  robot->getSensors(rsensors, rsensornumber);
  if(len != rsensornumber){
    fprintf(stderr, "%s:%i: Got not enough sensors!\n", __FILE__, __LINE__);
  }
  
  wiring->wireSensors(rsensors, rsensornumber, csensors, csensornumber, noise);
  controller->step(csensors, csensornumber, cmotors, cmotornumber);
  wiring->wireMotors(rmotors, rmotornumber, cmotors, cmotornumber);
  robot->setMotors(rmotors, rmotornumber);
  plot(csensors, csensornumber, cmotors, cmotornumber);
}
