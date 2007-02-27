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
 *   Revision 1.14  2007-02-27 12:00:05  robot5
 *   Minor changes for SoundMan functionalities.
 *
 *   Revision 1.12  2007/02/01 15:53:16  martius
 *   inspectables list. Robot is added in case it is derived from Inspectable
 *
 *   Revision 1.11  2006/12/28 18:38:03  robot5
 *   Soundchanger call modified.
 *
 *   Revision 1.10  2006/12/13 09:13:24  martius
 *   agents get comments about changed parameter for logfile
 *
 *   Revision 1.9  2006/12/11 18:10:52  martius
 *   noisefactor and default constructor
 *
 *   Revision 1.8  2006/11/30 10:02:11  robot5
 *   Added support for Sndchanger (experimental). Startup with argument -s.
 *
 *   Revision 1.7  2006/11/23 13:04:10  martius
 *   bugfix in onlycontrolrobot because rmotor was not set to zero!
 *
 *   Revision 1.6  2006/11/17 13:46:59  martius
 *   list of configureables to appear in configuration file
 *
 *   Revision 1.5  2006/08/04 15:16:13  martius
 *   documentation
 *
 *   Revision 1.4  2006/08/03 07:35:53  martius
 *   quit message not in file mode
 *
 *   Revision 1.3  2006/08/02 09:36:34  martius
 *   neuronviz command updated
 *   removeplotoptions does not segfault anymore
 *   send #QUIT to pipes
 *
 *   Revision 1.2  2006/07/14 12:23:57  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.12  2006/06/25 21:56:38  martius
 *   examples added
 *
 *   Revision 1.1.2.11  2006/06/25 16:51:35  martius
 *   configureable has name and revision
 *   a robot is configureable by default
 *
 *   Revision 1.1.2.10  2006/05/29 20:43:28  martius
 *   better file name for logging
 *
 *   Revision 1.1.2.9  2006/05/29 12:11:12  fhesse
 *   Agent::initLoggingFile() call added in Agent::init()
 *
 *   Revision 1.1.2.8  2006/05/15 13:08:18  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.1.2.7  2006/03/31 16:28:32  fhesse
 *   in setTrackoptions: trackrobot.open() only when one of the track optoions is active
 *
 *   Revision 1.1.2.6  2006/02/24 14:46:00  martius
 *   recording inverval in plotoutput
 *
 *   Revision 1.1.2.5  2006/02/20 17:33:15  martius
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
#include <time.h>
#include <string.h>

using namespace std;

Agent::Agent(const PlotOption& plotOption, double noisefactor){
  internInit();
  if(plotOption.mode!=NoPlot) plotOptions.push_back(plotOption);
  this->noisefactor = noisefactor;
}


Agent::Agent(const std::list<PlotOption>& plotOptions, double noisefactor)
  : noisefactor(noisefactor), plotOptions(plotOptions){
  internInit();
}

void Agent::internInit(){
  controller = 0;
  robot      = 0;
  wiring     = 0;
    
  rsensors=0; rmotors=0; 
  csensors=0; cmotors=0; 
  
  initialised = false;
  t=0;
}
  
Agent::~Agent(){
  // closes all pipes of the agents due to pause mode or so
  for (int i = NoPlot; i < LastPlot; i++){
    removePlotOption((PlotMode)i);
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
  assert(controller && robot && wiring);

  rsensornumber = robot->getSensorNumber();
  rmotornumber  = robot->getMotorNumber();
  wiring->init(rsensornumber, rmotornumber);
  csensornumber = wiring->getControllerSensornumber();
  cmotornumber  = wiring->getControllerMotornumber();
  controller->init(csensornumber, cmotornumber);

  rsensors      = (sensor*) malloc(sizeof(sensor) * rsensornumber);
  rmotors       = (motor*)  malloc(sizeof(motor)  * rmotornumber);
  memset(rmotors,0,sizeof(motor)*rmotornumber);
  csensors      = (sensor*) malloc(sizeof(sensor) * csensornumber);
  cmotors       = (motor*)  malloc(sizeof(motor)  * cmotornumber);

  inspectables.push_back(controller);
  inspectables.push_back(wiring);  
  if(dynamic_cast<Inspectable*>(robot) !=0)
    inspectables.push_back((Inspectable*)robot);  

  // copy plotoption list and add it one by one
  list<PlotOption> po_copy(plotOptions);
  plotOptions.clear();
  // open the plotting pipe (and file logging) if configured
  for(list<PlotOption>::iterator i=po_copy.begin(); i != po_copy.end(); i++){
    addPlotOption(*i);
  }    
  initialised = true;
  return true;
}

void Agent::addPlotOption(const PlotOption& plotOption) {
  PlotOption po = plotOption;
  // if plotoption with the same mode exists -> delete it
  removePlotOption(po.mode);

  // this prevents the simulation to terminate if the child  closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN); 
  if(robot) {
    po.name = robot->getName();
  }
  po.open();
  if(po.pipe){
    // print network description given by the structural information of the controller
    printNetworkDescription(po.pipe, "Selforg"/*controller->getName()*/, controller);
    // print interval
    fprintf(po.pipe, "# Recording every %dth dataset\n", po.interval);
    // print all configureables
    for(list<const Configurable*>::iterator i = po.configureables.begin(); i!= po.configureables.end(); i++){
      (*i)->print(po.pipe, "# ");
    }    
    // print all parameters of the controller
    controller->print(po.pipe, "# ");
    // print all parameters of the controller
    robot->print(po.pipe, "# ");
    // print head line with all parameter names
    unsigned int snum = plotOption.whichSensors == Robot ? rsensornumber : csensornumber;
    printInternalParameterNames(po.pipe, snum, cmotornumber, inspectables);
  }    
  plotOptions.push_back(po);
}

bool Agent::removePlotOption(PlotMode mode) {
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
void Agent::plot(const sensor* rx, int rsensornumber, const sensor* cx, int csensornumber, 
		 const motor* y, int motornumber){
  assert(controller && rx && cx && y);
  
  for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    if( ((*i).pipe) && (t % (*i).interval == 0) ){
      if((*i).whichSensors == Robot){
	printInternalParameters((*i).pipe, rx, rsensornumber, y, motornumber, inspectables);
      }else{
	printInternalParameters((*i).pipe, cx, csensornumber, y, motornumber, inspectables);
      }
      if(t% ((*i).interval * 10)) fflush((*i).pipe);    
    } // else {
      //      if (!(*i).pipe) { // if pipe is closed
      // std::cout << "pipe is closed!" << std::endl;
      //      }
      // }
  }
};


void Agent::writePlotComment(const char* cmt){
  for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    if( ((*i).pipe) && (t % (*i).interval == 0) && (strlen(cmt)>0)){ // for the guilogger pipe      
      char last = cmt[strlen(cmt)-1];
      fprintf((*i).pipe, "# %s", cmt);
      if(last!=10 && last!=13) // print with or without new line
	fprintf((*i).pipe, "\n");	
    } 
  }
}

//  Performs an step of the agent, including sensor reading, pushing sensor values through wiring, 
//  controller step, pushing controller steps back through wiring and sent resulting motorcommands 
//  to robot.
//  @param noise Noise strength.
void Agent::step(double noise){
  assert(controller && robot && wiring && rsensors && rmotors && csensors && cmotors);
  
  int len =  robot->getSensors(rsensors, rsensornumber);
  if(len != rsensornumber){
    fprintf(stderr, "%s:%i: Got not enough sensors, expected %i, got %i!\n", __FILE__, __LINE__, 
	    rsensornumber, len);
  }
  
  wiring->wireSensors(rsensors, rsensornumber, csensors, csensornumber, noise * noisefactor);
  controller->step(csensors, csensornumber, cmotors, cmotornumber);
  wiring->wireMotors(rmotors, rmotornumber, cmotors, cmotornumber);
  robot->setMotors(rmotors, rmotornumber);
  plot(rsensors, rsensornumber, csensors, csensornumber, cmotors, cmotornumber);
  
  trackrobot.track(robot);

  t++;
}

// Sends only last motor commands again to robot. 
void Agent::onlyControlRobot(){
  assert(robot && rmotors);
  robot->setMotors(rmotors, rmotornumber);  
}


// sets the trackoptions which enable tracking of a robot
void Agent::setTrackOptions(const TrackRobot& trackrobot){
  this->trackrobot = trackrobot;
  if (trackrobot.trackPos || trackrobot.trackSpeed || trackrobot.trackOrientation){
    if(!this->trackrobot.open(robot)){
      fprintf(stderr, "could not open trackfile!\n");
    }else{
      // print all parameters of the controller to trackfile
      controller->print(this->trackrobot.file, "# ");                
    }
  }
}

void Agent::addInspectable(const Inspectable* inspectable){
  if(!initialised){
    inspectables.push_back(inspectable);
  }
}


bool PlotOption::open(){
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
      sprintf(logfilename,"%s_%02i-%02i-%02i_%02i-%02i-%02i-%02i.log",
	      name.c_str(), t->tm_year%100, t->tm_mon+1 , t->tm_mday,
	      t->tm_hour, t->tm_hour, t->tm_min, t->tm_sec);
      pipe=fopen(logfilename,"w");
      if (pipe)
	std::cout << "Now logging to file \"" << logfilename << "\"." << std::endl;
      break;
  case GuiLogger_File:
    pipe=popen("guilogger -m pipe -d 5 -l","w");
    break;
  case GuiLogger:
    pipe=popen("guilogger -m pipe -d 5","w");
    break;
  case NeuronViz:
    pipe=popen("neuronviz > /dev/null","w");  // TODO: Platform dependent
    break;

  case SoundMan_Disc:
    pipe=popen("cd ../../utils/; java SoundMan -disc","w");
    break;
  case SoundMan_Ampl:
    pipe=popen("cd ../../utils/; java SoundMan -ampl","w");
    break;
  case SoundMan_Freq:
    pipe=popen("cd ../../utils/; java SoundMan -freq","w");
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

    case SoundMan_Disc:
    case SoundMan_Ampl:
    case SoundMan_Freq:
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

