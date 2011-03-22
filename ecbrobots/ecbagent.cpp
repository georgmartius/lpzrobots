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
 *   Revision 1.10  2011-03-22 16:37:05  guettler
 *   - adpaptions to enhanced configurable and inspectable interface
 *
 *   Revision 1.9  2011/03/21 17:30:35  guettler
 *   - adapted to enhanced configurable interface
 *
 *   Revision 1.8  2011/02/11 12:13:31  guettler
 *   - renamed init to preInit function
 *   - ECB are added as configurables to PlotOptionEngine automatically
 *
 *   Revision 1.7  2011/01/24 14:16:55  guettler
 *   - added comment
 *   - cosmetic changes
 *
 *   Revision 1.6  2010/11/10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.5  2009/08/11 15:49:05  guettler
 *   Current development state:
 *   - Support of communication protocols for XBee Series 1, XBee Series 2 and cable mode
 *   - merged code base from ecb_robots and Wolgang Rabes communication handling;
 *     ECBCommunicator (almost) entirely rewritten: Use of Mediator (MediatorCollegues: ECB),
 *     Callbackble (BackCaller: SerialPortThread)
 *   - New CThread for easy dealing with threads (is using pthreads)
 *   - New TimerThreads for timed event handling
 *   - SerialPortThread now replaces the cserialthread
 *   - QGlobalData, ECBCommunicator is now configurable
 *   - ECBAgent rewritten: new PlotOptionEngine support, adapted to new WiredController structure
 *   - ECBRobot is now Inspectables (uses new infoLines functionality)
 *   - ECB now supports dnsNames and new communication protocol via Mediator
 *   - Better describing command definitions
 *   - SphericalRobotECB: adapted to new ECB structure, to be tested
 *
 *   Revision 1.4  2009/03/25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.3  2008/08/12 11:45:29  guettler
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

#include <signal.h>

#include <selforg/abstractwiring.h>
#include <selforg/abstractcontroller.h>

using namespace std;

namespace lpzrobots {

  ECBAgent::ECBAgent(const PlotOption& plotOption /* = PlotOption(NoPlot)*/, double noisefactor /*= 1*/, const string& name, const string& revision) :
    Agent(plotOption, noisefactor, name, revision), internalInitialised(false) {
    addParameterDef("restartPlotEngine", &restartPlotEngine, true);
  }

  ECBAgent::ECBAgent(const std::list<PlotOption>& plotOptions, double noisefactor /*= 1*/, const string& name, const string& revision) :
    Agent(plotOptions, noisefactor, name, revision), internalInitialised(false) {
    addParameterDef("restartPlotEngine", &restartPlotEngine, true);
  }

  ECBAgent::~ECBAgent() {
    if (robot)
      delete getRobot();
    if (controller)
      delete controller;
    if (wiring)
      delete wiring;
  }

  ECBRobot* ECBAgent::getRobot() {
    return (ECBRobot*) robot;
  }

  bool ECBAgent::preInit(AbstractController *controller, ECBRobot *robot, AbstractWiring *wiring) {
    this->controller = controller;
    this->robot = robot;
    this->wiring = wiring;
    return true;
  }

  void ECBAgent::step(double noise, double time) {
    if (internalInitialised) {
      if (getRobot()->isInitialised()) {
        if (restartPlotEngine) {
          plotEngine.reInit();
          restartPlotEngine = false;
        }
        Agent::step(noise, time);
      } else { // not (anymore) initialised, something changed. Wait and close pipes of PlotEngine.
        plotEngine.closePipes();
        restartPlotEngine = true;
      }
    } else if (getRobot()->isInitialised()) {
      init();
      Agent::step(noise, time);
    } // else do nothing and wait
  }

  bool ECBAgent::init() {
    internalInitialised = Agent::init(controller, robot, wiring);
    if (internalInitialised) {
      callBack(CALLBACK_CONFIGURABLE_CHANGED);
      restartPlotEngine = false; // (re-)start is done by Agent::step(...)
    }
    return internalInitialised;
  }

  bool ECBAgent::isInitialized() {
    return internalInitialised;
  }

}
