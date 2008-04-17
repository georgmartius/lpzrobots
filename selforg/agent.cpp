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
 *   Revision 1.22  2008-04-17 14:53:53  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.21  2007/11/06 14:58:48  martius
 *   major change!
 *   agent is now a robot with a wired controller,
 *   most code moved to wiredcontroller class
 *
 *   Revision 1.20  2007/08/29 11:33:20  martius
 *   simulation time enters logfile
 *
 *   Revision 1.19  2007/06/21 16:21:58  martius
 *   time is written here to logfile
 *
 *   Revision 1.18  2007/06/08 15:49:47  martius
 *   print start time into log
 *
 *   Revision 1.17  2007/05/09 15:14:46  robot3
 *   *** empty log message ***
 *
 *   Revision 1.16  2007/05/07 20:58:21  robot3
 *   added support for Interface Callbackable (to find in selforg/utils)
 *   classes can now register at agent to get a callback every step
 *
 *   Revision 1.15  2007/03/05 17:52:20  martius
 *   plotoptions hav optional string parameter
 *
 *   Revision 1.14  2007/02/27 12:00:05  robot5
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

#include "callbackable.h"

using namespace std;

Agent::Agent(const PlotOption& plotOption, double noisefactor)
  : WiredController(plotOption, noisefactor) {
  robot      = 0;
  rsensors=0; rmotors=0;
}


Agent::Agent(const std::list<PlotOption>& plotOptions, double noisefactor)
  : WiredController(plotOptions, noisefactor){
  robot      = 0;
  rsensors=0; rmotors=0;
}

Agent::~Agent(){
  // closes all pipes of the agents due to pause mode or so
  trackrobot.close(); 
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
  
  // add robot to inspectables
  Inspectable* in = dynamic_cast<Inspectable*>(robot);
  if(in) inspectables.push_back(in);
  
  return WiredController::init(controller,wiring, rsensornumber, rmotornumber, &randGen);
}

void Agent::addPlotOption(const PlotOption& plotOption) {
  PlotOption po = plotOption;
  if(robot) {
    po.addConfigurable(robot);
    po.setName(robot->getName());
  }
  WiredController::addPlotOption(po);
}

void Agent::step(double noise, double time){
  assert(robot && rsensors && rmotors);

  int len =  robot->getSensors(rsensors, rsensornumber);
  if(len != rsensornumber){
    fprintf(stderr, "%s:%i: Got not enough sensors, expected %i, got %i!\n", __FILE__, __LINE__,
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
  this->trackrobot = trackrobot;
  if (trackrobot.trackPos || trackrobot.trackSpeed || trackrobot.trackOrientation){
    if(!this->trackrobot.open(robot)){
      fprintf(stderr, "could not open trackfile!\n");
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

