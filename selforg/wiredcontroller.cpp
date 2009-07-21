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
 *   Revision 1.13  2009-07-21 09:10:22  robot12
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

#include "callbackable.h"

using namespace std;

WiredController::WiredController(const PlotOption& plotOption, double noisefactor) : PlotOptionEngine(plotOption) {
  internInit();
  this->noisefactor = noisefactor;
}


WiredController::WiredController(const std::list<PlotOption>& plotOptions, double noisefactor)
  : PlotOptionEngine(plotOptions), noisefactor(noisefactor) {
  internInit();
}

void WiredController::internInit(){
  controller = 0;
  wiring     = 0;

  cmotors=0;
  csensors=0;

  initialised = false;
}

WiredController::~WiredController(){
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
  // open the all outputs
  for(list<PlotOption>::iterator i=po_copy.begin(); i != po_copy.end(); i++){
    removePlotOption((*i).getPlotOptionMode());
    addPlotOption(*i);
  }
  initialised = true;
  return true;
}

PlotOption WiredController::addPlotOption(PlotOption& plotOption) {
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
    // print all parameters of the wiring if configurable
    Configurable* c = dynamic_cast<Configurable*>(wiring);
    if(c) c->print(po.pipe, "# ");
    // print all parameters of the controller
    controller->print(po.pipe, "# ");
    // print head line with all parameter names
    unsigned int snum = plotOption.whichSensors == Robot ? rsensornumber : csensornumber;
    printInternalParameterNames(po.pipe, snum, cmotornumber, inspectables);
  }
  else
    printf("Opening of pipe for PlotOption failed!\n");

  plotOptions.push_back(po);

  return po;
}

// bool WiredController::removePlotOption(PlotMode mode) {
//   // if plotoption with the same mode exists -> delete it
//
//   list<PlotOption>::iterator po
//     = find_if(plotOptions.begin(), plotOptions.end(), PlotOption::matchMode(mode));
// //   FOREACH (list<PlotOption>, plotOptions, po) {
// //     if(po != plotOptions.end()) {
// //         std::cout << "CLOSING now pipes......................." << std::endl;
// //         PlotOption p_tmp = (*po);
// //         std::cout << "p_tmp.mode "<< p_tmp.mode << " == " << mode << "???????????????" << std::endl;
// //         if (p_tmp.mode == mode) {
// //            (*po).close();
// //             plotOptions.erase(po);
// //         return true;
// //         }
// //       }
// //   }
//   return false;
// }



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
      (*i).flush(t);
    } // else {
    /*if (!(*i).pipe) { // if pipe is closed
      std::cout << "pipe is closed!" << std::endl;
           }
    */
  }
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
  controller->step(csensors, csensornumber, cmotors, cmotornumber);
  wiring->wireMotors(motors, rmotornumber, cmotors, cmotornumber);
  plot(sensors, rsensornumber, csensors, csensornumber, motors, rmotornumber, time);
  // do a callback for all registered Callbackable classes
  FOREACH(list<Callbackable*>, callbackables, i){
    (*i)->doOnCallBack();
  }
  t++;
}


void WiredController::addCallbackable(Callbackable* callbackable){
  callbackables.push_back(callbackable);
}
