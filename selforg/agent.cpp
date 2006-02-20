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
 *   Revision 1.1.2.5  2006-02-20 17:33:15  martius
 *   include interval in logging output
 *
 *   Revision 1.1.2.4  2006/02/06 15:24:09  robot3
 *   avoid conversion from pointer to int
 *
 *   Revision 1.1.2.3  2006/01/31 16:45:37  martius
 *   neuronviz plotoption
 *
 *   Revision 1.1.2.2  2005/11/15 12:30:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.1.2.1  2005/11/14 17:37:56  martius
 *   moved to selforg
 *
 *   Revision 1.13  2005/11/07 17:04:20  martius
 *   agent can be constructed using a list of PlotOptions
 *   tracking file gets the controller parameter as well
 *
 *   Revision 1.12  2005/10/28 12:03:36  martius
 *   network description printed
 *
 *   Revision 1.11  2005/10/24 13:32:07  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.10  2005/10/06 17:11:36  martius
 *   switched to stl lists
 *
 *   Revision 1.9  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.8  2005/08/31 11:15:59  martius
 *   *** empty log message ***
 *
 *   Revision 1.7  2005/08/03 20:34:58  martius
 *   use if Inspectable interface
 *
 *   Revision 1.6  2005/07/26 17:01:47  martius
 *   flushing every 10
 *   guilogger is opened with nice -2
 *
 *   Revision 1.5  2005/07/26 08:36:53  fhesse
 *   delay for logging in file changed from 15 to 3
 *
 *   Revision 1.4  2005/07/21 11:30:59  fhesse
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

#include "abstractrobot.h"
#include "abstractcontroller.h"
#include "abstractwiring.h"

Agent::Agent(const PlotOption& plotOption){
  internInit();
  plotOptions.push_back(plotOption);
}


Agent::Agent(const list<PlotOption>& plotOptions)
  : plotOptions(plotOptions){
  internInit();
}

void Agent::internInit(){
  controller = 0;
  robot      = 0;
  wiring     = 0;
    
  rsensors=0; rmotors=0; 
  csensors=0; cmotors=0; 
  
  t=0;  
}
  
Agent::~Agent(){
  for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    (*i).close();
  }
  trackrobot.close();
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
    wiring->init(rsensornumber, rmotornumber);
    csensornumber = wiring->getControllerSensornumber();
    cmotornumber  = wiring->getControllerMotornumber();
    controller->init(csensornumber, cmotornumber);

    rsensors      = (sensor*) malloc(sizeof(sensor) * rsensornumber);
    rmotors       = (motor*)  malloc(sizeof(motor)  * rmotornumber);
    csensors      = (sensor*) malloc(sizeof(sensor) * csensornumber);
    cmotors       = (motor*)  malloc(sizeof(motor)  * cmotornumber);
    
    for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
      if((*i).open()){
	// print network description given by the structural information of the controller
	printNetworkDescription((*i).pipe, "Lpzrobots"/*controller->getName()*/, controller);
	// print interval
	fprintf((*i).pipe, "# Recording every %d dataset\n",(*i).interval);
	// print all parameters of the controller
	controller->print((*i).pipe, "# ");
	// print head line with all parameter names
	unsigned int snum = (*i).whichSensors == Robot ? rsensornumber : csensornumber;
	Inspectable* inspectables[2] = {controller, wiring};      
	printInternalParameterNames((*i).pipe, snum, cmotornumber, inspectables, 2);
      }
    }    
    return true;
  }
}


bool PlotOption::open(){
  // this prevents the simulation to terminate if the child  closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN); 
  // TODO: get the guilogger call from some config
  switch(mode){
  case GuiLogger_File:
    pipe=popen("guilogger -l -m pipe -d 5","w");
    break;
  case GuiLogger:
    pipe=popen("guilogger -m pipe -d 5","w");
    break;
  case NeuronViz:
    pipe=popen("neuronviz ","w");
    break;
  default:
    return false;
  }
  if(pipe==0){
    fprintf(stderr, "%s:%i: could not open plot tool!\n", __FILE__, __LINE__);    
    return false;
  }else return true;
}

void PlotOption::close(){
  if (pipe) pclose(pipe);
  pipe=0;
}

// Plots controller sensor- and motorvalues and internal controller parameters.
void Agent::plot(const sensor* rx, int rsensornumber, const sensor* cx, int csensornumber, 
		 const motor* y, int motornumber){
  assert(controller && rx && cx && y);
  
  Inspectable* inspectables[2] = {controller, wiring};
  for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    if( ((*i).pipe) && (t % (*i).interval == 0) ){
      if((*i).whichSensors == Robot){
	printInternalParameters((*i).pipe, rx, rsensornumber, y, motornumber, inspectables , 2);
      }else{
	printInternalParameters((*i).pipe, cx, csensornumber, y, motornumber, inspectables , 2);
      }
      if(t% ((*i).interval * 10)) fflush((*i).pipe);    
    }
  }
};



//  Performs an step of the agent, including sensor reading, pushing sensor values through wiring, 
//  controller step, pushing controller steps back through wiring and sent resulting motorcommands 
//  to robot.
//  @param noise Noise strength.
void Agent::step(double noise){

  if(!controller || !robot || !wiring || !rsensors || !rmotors || !csensors || !cmotors) {
    fprintf(stderr, "%s:%i: something is null: cont %i rob %i wiring %i rsens %i rmots %i csens %i cmots %i!\n", 
	    __FILE__, __LINE__, controller==0, robot==0,  
	    wiring==0, rsensors==0, rmotors==0, 
	    csensors==0, cmotors==0);
  }
  
  int len =  robot->getSensors(rsensors, rsensornumber);
  if(len != rsensornumber){
    fprintf(stderr, "%s:%i: Got not enough sensors, expected %i, got %i!\n", __FILE__, __LINE__, 
	    rsensornumber, len);
  }
  
  wiring->wireSensors(rsensors, rsensornumber, csensors, csensornumber, noise);
  controller->step(csensors, csensornumber, cmotors, cmotornumber);
  wiring->wireMotors(rmotors, rmotornumber, cmotors, cmotornumber);
  robot->setMotors(rmotors, rmotornumber);
  plot(rsensors, rsensornumber, csensors, csensornumber, cmotors, cmotornumber);
  
  trackrobot.track(robot);

  t++;
}


// sets the trackoptions which enable tracking of a robot
void Agent::setTrackOptions(const TrackRobot& trackrobot){
  this->trackrobot = trackrobot;
  if(!this->trackrobot.open(robot)){
    fprintf(stderr, "could not open trackfile!\n");
  }else{
    // print all parameters of the controller
    controller->print(this->trackrobot.file, "# ");                
  }
}
