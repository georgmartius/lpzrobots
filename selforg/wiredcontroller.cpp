/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    joergweide84@aol.com (robot12)                                       *
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
 *   Revision 1.19  2010-10-18 15:10:40  martius
 *   added motorbabbling
 *
 *   Revision 1.18  2010/07/02 15:57:25  martius
 *   wirings have new initIntern signature -> less errors can be made
 *   abstractwiring generates the noise of given length
 *
 *   Revision 1.17  2009/08/10 15:36:19  der
 *   plotoptions can again be added and initialized later
 *   ctrl-g and -f are working again
 *   ctrl-n added for neuronviz
 *
 *   Revision 1.16  2009/08/10 07:41:48  guettler
 *   - uses new BackCaller implementation
 *   - shortened signature of function plot (removed unnecessary
 *     parameters)
 *
 *   Revision 1.15  2009/08/07 09:33:38  martius
 *   init plotengine with controller
 *
 *   Revision 1.14  2009/08/05 22:57:09  martius
 *   use new plotoptionsengine entirely
 *   wirings provide the sensor and motors such that the entire
 *    old functionality (and more) is now available with through
 *    the separat plotoptionsengine.
 *
 *   Revision 1.13  2009/07/21 09:10:22  robot12
 *   add some comments
 *
 *   Revision 1.12  2009/06/02 09:55:24  robot12
 *   Splitting of WiredController and PlotOption into WiredController : public PlotOptionEngine and
 *   PlotOption (used by ga_tools). Further refactorings needed.
 *
 *   Revision 1.11  2009/05/11 17:08:01  martius
 *   flushing optimized
 *
 *   Revision 1.10  2009/03/31 15:52:33  martius
 *   plotted motor values are the ones sent to the robot (after wiring)
 *
 *   Revision 1.9  2009/03/27 06:16:57  guettler
 *   support for gcc 4.3 compatibility (has to be checked), StatisticTools moves from utils to statistictools
 *
 *   Revision 1.8  2009/03/25 11:55:32  robot1
 *   changed minor handling of PlotOptions
 *
 *   Revision 1.7  2008/09/16 15:36:40  martius
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


#include <signal.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <algorithm>

#include "printInternals.h"

#include "abstractcontroller.h"
#include "abstractwiring.h"

#include "motorbabbler.h"

#include "callbackable.h"

using namespace std;

WiredController::WiredController(const PlotOption& plotOption, double noisefactor) 
  : noisefactor(noisefactor), plotEngine(plotOption) {
  internInit();  
}


WiredController::WiredController(const std::list<PlotOption>& plotOptions, double noisefactor)
  : noisefactor(noisefactor), plotEngine(plotOptions) {
  internInit();
}

void WiredController::internInit(){
  controller = 0;
  wiring     = 0;

  cmotors=0;
  csensors=0;

  motorBabbler = 0;
  isMotorBabbling = false;

  t=1;
  initialised = false;
}

WiredController::~WiredController(){
  if(csensors) free(csensors);
  if(cmotors)  free(cmotors);
  if(motorBabbler) delete motorBabbler;
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


  plotEngine.addInspectable(this, true);
  plotEngine.addInspectable(wiring,true);
  plotEngine.addInspectable(controller); 

  plotEngine.addConfigurable(controller);
  Configurable* c_wiring = dynamic_cast<Configurable*>(wiring);
  if(c_wiring) plotEngine.addConfigurable(c_wiring);
  
  plotEngine.init(controller);

  initialised = true;
  return true;
}

void WiredController::startMotorBabblingMode (AbstractController* babblecontroller){
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
  isMotorBabbling=true;
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

void WiredController::addInspectable(const Inspectable* inspectable){  
  plotEngine.addInspectable(inspectable);
}

void WiredController::addConfigurable(const Configurable* c){
  plotEngine.addConfigurable(c);
}

void WiredController::writePlotComment(const char* cmt){
  plotEngine.writePlotComment(cmt);
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
  if(isMotorBabbling){
    motorBabbler->step(csensors, csensornumber, cmotors, cmotornumber);
    controller->motorBabblingStep(csensors, csensornumber, cmotors, cmotornumber);
  }else{
    controller->step(csensors, csensornumber, cmotors, cmotornumber);
  }
  wiring->wireMotors(motors, rmotornumber, cmotors, cmotornumber);
  plot(time);
  // do a callback for all registered Callbackable classes
  callBack();
  t++;
}

