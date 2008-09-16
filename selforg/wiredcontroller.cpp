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
 *   Revision 1.7  2008-09-16 15:36:40  martius
 *   added assert.h
 *
 *   Revision 1.6  2008/08/12 11:50:15  guettler
 *   plug and play update, added some features for the ECBRobotGUI
 *
 *   Revision 1.5  2008/08/01 14:42:04  guettler
 *   we try the trip to hell! make selforg AVR compatible...good luck (first changes)
 *
 *   Revision 1.4  2008/04/30 15:50:20  martius
 *   filename had 2time hour
 *
 *   Revision 1.3  2008/04/29 06:51:40  guettler
 *   fixed division by zero bug (t % (*i).interval), when ode_robots starts in
 *   pause modus
 *
 *   Revision 1.2  2008/04/17 14:54:35  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.1  2007/11/06 15:14:27  martius
 *   new class that composes controller and wiring
 *
 *                                                                 *
 ***************************************************************************/

#include "wiredcontroller.h"

#ifndef AVR

#include <signal.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include "printInternals.h"

#include "abstractcontroller.h"
#include "abstractwiring.h"

#include "callbackable.h"

using namespace std;

WiredController::WiredController(const PlotOption& plotOption, double noisefactor){
  internInit();
  if(plotOption.mode!=NoPlot) plotOptions.push_back(plotOption);
  this->noisefactor = noisefactor;
}


WiredController::WiredController(const std::list<PlotOption>& plotOptions, double noisefactor)
  : noisefactor(noisefactor), plotOptions(plotOptions){
  internInit();
}

void WiredController::internInit(){
  controller = 0;
  wiring     = 0;

  cmotors=0;
  csensors=0;

  initialised = false;
  t=1;
}

WiredController::~WiredController(){
  // closes all pipes of the agents due to pause mode or so
  for (int i = NoPlot; i < LastPlot; i++){
    removePlotOption((PlotMode)i);
  }
  if(csensors) free(csensors);
  if(cmotors)  free(cmotors);
}


/// initializes the object with the given controller, robot and wiring
//  and initializes pipe to guilogger
bool WiredController::init(AbstractController* controller, AbstractWiring* wiring,
			   int robotsensornumber, int robotmotornumber, RandGen* randGen){
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

void WiredController::addPlotOption(const PlotOption& plotOption) {
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

bool WiredController::removePlotOption(PlotMode mode) {
  // if plotoption with the same mode exists -> delete it
  list<PlotOption>::iterator po
      = find_if(plotOptions.begin(), plotOptions.end(), PlotOption::matchMode(mode));
  if(po != plotOptions.end()){
    (*po).close();
    plotOptions.erase(po);
    return true;
  }
  return false;
}


// Plots controller sensor- and motorvalues and internal controller parameters.
void WiredController::plot(const sensor* rx, int rsensornumber,
			   const sensor* cx, int csensornumber,
			   const motor* y, int motornumber, double time){
  assert(controller && rx && cx && y);

  for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    if( ((*i).pipe) && ((*i).interval>0) && (t % (*i).interval == 0) ){

      if((*i).whichSensors == Robot){
		printInternalParameters((*i).pipe, time, rx, rsensornumber, y, motornumber, inspectables);
      }else{
		printInternalParameters((*i).pipe, time, cx, csensornumber, y, motornumber, inspectables);
      }
      if(t% ((*i).interval * 10)) fflush((*i).pipe);
    } // else {
      //      if (!(*i).pipe) { // if pipe is closed
      // std::cout << "pipe is closed!" << std::endl;
      //      }
      // }
  }
};


void WiredController::writePlotComment(const char* cmt){
  for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    if( ((*i).pipe) && (t % (*i).interval == 0) && (strlen(cmt)>0)){ // for the guilogger pipe
      char last = cmt[strlen(cmt)-1];
      fprintf((*i).pipe, "# %s", cmt);
      if(last!=10 && last!=13) // print with or without new line
	fprintf((*i).pipe, "\n");
    }
  }
}

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
  controller->step(csensors, csensornumber, cmotors, cmotornumber);
  wiring->wireMotors(motors, rmotornumber, cmotors, cmotornumber);
  plot(sensors, rsensornumber, csensors, csensornumber, cmotors, cmotornumber,time);
  // do a callback for all registered Callbackable classes
  FOREACH(list<Callbackable*>, callbackables, i){
    (*i)->doOnCallBack();
  }
  t++;
}

void WiredController::addInspectable(const Inspectable* inspectable){
  if(!initialised){
    inspectables.push_back(inspectable);
  } else {
    std::cerr << "WiredController::addInspectable(const Inspectable* inspectable); failed, because WiredController was already initialised! " << std::endl;
  }
}

void WiredController::addCallbackable(Callbackable* callbackable){
  callbackables.push_back(callbackable);
}

bool PlotOption::open(){
  char cmd[255];
  // this prevents the simulation to terminate if the child  closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN);
  switch(mode){
  case File:
      struct tm *t;
      time_t tnow;
      time(&tnow);
      t = localtime(&tnow);
      char logfilename[255];
      sprintf(logfilename,"%s_%02i-%02i-%02i_%02i-%02i-%02i.log",
	      name.c_str(), t->tm_year%100, t->tm_mon+1 , t->tm_mday,
	      t->tm_hour, t->tm_min, t->tm_sec);
      pipe=fopen(logfilename,"w");
      if (pipe)
	std::cout << "Now logging to file \"" << logfilename << "\"." << std::endl;
      break;
  case GuiLogger_File:
    pipe=popen("guilogger -m pipe -l","w");
    break;
  case GuiLogger:
    pipe=popen("guilogger -m pipe","w");
    break;
  case ECBRobotGUI:
    pipe=popen("src","w");
    break;
  case NeuronViz:
    pipe=popen("neuronviz > /dev/null","w");  // TODO: Platform dependent
    break;
  case SoundMan:
    sprintf(cmd,"soundMan %s",parameter.c_str());
    pipe=popen(cmd,"w");
    break;
  default: // and NoPlot
    return false;
  }
  if(pipe==0){
    fprintf(stderr, "%s:%i: could not open plot tool!\n", __FILE__, __LINE__);
    return false;
  }else return true;
}


void PlotOption::close(){
  if (pipe) {
    switch(mode){
    case File:
      std::cout << "logfile closing...SUCCESSFUL" << std::endl;
      fclose(pipe);
      break;
    case GuiLogger:
    case GuiLogger_File:
      //std::cout << "guilogger pipe closing...maybe you must manually close the guilogger first!"
      //          << std::endl;
      // send quit message to pipe
      fprintf(pipe, "#QUIT\n");
      pclose(pipe);
      std::cout << "guilogger pipe closing...SUCCESSFUL" << std::endl;
      break;
    case NeuronViz:
      //std::cout << "neuronviz pipe closing...maybe you must manually close the neuronviz first!"
      //          << std::endl;
      // send quit message to pipe
      fprintf(pipe, "#QUIT\n");
      pclose(pipe);
      std::cout << "neuronviz pipe closing...SUCCESSFUL" << std::endl;
      break;

    case SoundMan:
      std::cout << "SoundMan closing...SUCCESSFUL" << std::endl;
      fclose(pipe);
      break;

    default:
      break;
    }
    pipe=0;
  }
}

void PlotOption::addConfigurable(const Configurable* c){
  configureables.push_back(c);
}

#else /* AVR */

#include <signal.h>
#include "printInternals.h"

#include "abstractcontroller.h"
#include "abstractwiring.h"
#include <string.h>

#include "callbackable.h"

using namespace std;

WiredController::WiredController(const PlotOption& plotOption, double noisefactor){
  internInit();
  if(plotOption.mode!=NoPlot) plotOptions.push_back(plotOption);
  this->noisefactor = noisefactor;
}


WiredController::WiredController(const std::list<PlotOption>& plotOptions, double noisefactor)
  : noisefactor(noisefactor), plotOptions(plotOptions){
  internInit();
}

void WiredController::internInit(){
  controller = 0;
  wiring     = 0;

  cmotors=0;
  csensors=0;

  initialised = false;
  t=1;
}

WiredController::~WiredController(){
  // closes all pipes of the agents due to pause mode or so
  for (int i = NoPlot; i < LastPlot; i++){
    removePlotOption((PlotMode)i);
  }
  if(csensors) free(csensors);
  if(cmotors)  free(cmotors);
}


/// initializes the object with the given controller, robot and wiring
//  and initializes pipe to guilogger
bool WiredController::init(AbstractController* controller, AbstractWiring* wiring,
			   int robotsensornumber, int robotmotornumber, RandGen* randGen){
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

void WiredController::addPlotOption(const PlotOption& plotOption) {
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

bool WiredController::removePlotOption(PlotMode mode) {
  // if plotoption with the same mode exists -> delete it
  list<PlotOption>::iterator po
      = find_if(plotOptions.begin(), plotOptions.end(), PlotOption::matchMode(mode));
  if(po != plotOptions.end()){
    (*po).close();
    plotOptions.erase(po);
    return true;
  }
  return false;
}


// Plots controller sensor- and motorvalues and internal controller parameters.
void WiredController::plot(const sensor* rx, int rsensornumber,
			   const sensor* cx, int csensornumber,
			   const motor* y, int motornumber, double time){
  assert(controller && rx && cx && y);

  for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    if( ((*i).pipe) && ((*i).interval>0) && (t % (*i).interval == 0) ){

      if((*i).whichSensors == Robot){
		printInternalParameters((*i).pipe, time, rx, rsensornumber, y, motornumber, inspectables);
      }else{
		printInternalParameters((*i).pipe, time, cx, csensornumber, y, motornumber, inspectables);
      }
      if(t% ((*i).interval * 10)) fflush((*i).pipe);
    } // else {
      //      if (!(*i).pipe) { // if pipe is closed
      // std::cout << "pipe is closed!" << std::endl;
      //      }
      // }
  }
};


void WiredController::writePlotComment(const char* cmt){
  for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    if( ((*i).pipe) && (t % (*i).interval == 0) && (strlen(cmt)>0)){ // for the guilogger pipe
      char last = cmt[strlen(cmt)-1];
      fprintf((*i).pipe, "# %s", cmt);
      if(last!=10 && last!=13) // print with or without new line
	fprintf((*i).pipe, "\n");
    }
  }
}

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
  controller->step(csensors, csensornumber, cmotors, cmotornumber);
  wiring->wireMotors(motors, rmotornumber, cmotors, cmotornumber);
  plot(sensors, rsensornumber, csensors, csensornumber, cmotors, cmotornumber,time);
  // do a callback for all registered Callbackable classes
  FOREACH(list<Callbackable*>, callbackables, i){
    (*i)->doOnCallBack();
  }
  t++;
}

void WiredController::addInspectable(const Inspectable* inspectable){
  if(!initialised){
    inspectables.push_back(inspectable);
  } else {
    std::cerr << "WiredController::addInspectable(const Inspectable* inspectable); failed, because WiredController was already initialised! " << std::endl;
  }
}

void WiredController::addCallbackable(Callbackable* callbackable){
  callbackables.push_back(callbackable);
}

bool PlotOption::open(){
  char cmd[255];
  // this prevents the simulation to terminate if the child  closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN);
  switch(mode){
  case File:
      struct tm *t;
      time_t tnow;
      time(&tnow);
      t = localtime(&tnow);
      char logfilename[255];
      sprintf(logfilename,"%s_%02i-%02i-%02i_%02i-%02i-%02i.log",
	      name.c_str(), t->tm_year%100, t->tm_mon+1 , t->tm_mday,
	      t->tm_hour, t->tm_min, t->tm_sec);
      pipe=fopen(logfilename,"w");
      if (pipe)
	std::cout << "Now logging to file \"" << logfilename << "\"." << std::endl;
      break;
  case GuiLogger_File:
    pipe=popen("guilogger -m pipe -l","w");
    break;
  case GuiLogger:
    pipe=popen("guilogger -m pipe","w");
    break;
  case NeuronViz:
    pipe=popen("neuronviz > /dev/null","w");  // TODO: Platform dependent
    break;
  case SoundMan:
    sprintf(cmd,"soundMan %s",parameter.c_str());
    pipe=popen(cmd,"w");
    break;
  default: // and NoPlot
    return false;
  }
  if(pipe==0){
    fprintf(stderr, "%s:%i: could not open plot tool!\n", __FILE__, __LINE__);
    return false;
  }else return true;
}


void PlotOption::close(){
  if (pipe) {
    switch(mode){
    case File:
      std::cout << "logfile closing...SUCCESSFUL" << std::endl;
      fclose(pipe);
      break;
    case GuiLogger:
    case GuiLogger_File:
      //std::cout << "guilogger pipe closing...maybe you must manually close the guilogger first!"
      //          << std::endl;
      // send quit message to pipe
      fprintf(pipe, "#QUIT\n");
      pclose(pipe);
      std::cout << "guilogger pipe closing...SUCCESSFUL" << std::endl;
      break;
    case NeuronViz:
      //std::cout << "neuronviz pipe closing...maybe you must manually close the neuronviz first!"
      //          << std::endl;
      // send quit message to pipe
      fprintf(pipe, "#QUIT\n");
      pclose(pipe);
      std::cout << "neuronviz pipe closing...SUCCESSFUL" << std::endl;
      break;

    case SoundMan:
      std::cout << "SoundMan closing...SUCCESSFUL" << std::endl;
      fclose(pipe);
      break;

    default:
      break;
    }
    pipe=0;
  }
}

void PlotOption::addConfigurable(const Configurable* c){
  configureables.push_back(c);
}

#endif /* !AVR */
