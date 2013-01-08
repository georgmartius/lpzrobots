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
 *   Revision 1.9  2011-04-04 09:24:18  guettler
 *   renamed simStep to controlStep
 *
 *   Revision 1.8  2011/03/25 21:27:37  guettler
 *   - cleanup of agentList and communicator now handled, fixes the problem that
 *     PlotOptionEngine does not close all open pipes (e.g. GUILogger received no
 *     #QUIT)
 *
 *   Revision 1.7  2011/02/11 12:16:56  guettler
 *   - removed obsolete configurable parameters
 *
 *   Revision 1.6  2010/12/08 17:54:00  wrabe
 *   - some configurables removed (noise...)
 *
 *   Revision 1.5  2010/12/06 14:08:02  guettler
 *   -added some test variables for QConfigurable
 *
 *   Revision 1.4  2010/11/26 12:28:06  guettler
 *   - added bounds and description for paramval noise for testing purposes
 *
 *   Revision 1.3  2010/11/26 12:22:37  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *   Revision 1.2  2010/11/11 15:34:59  wrabe
 *   - some extensions for QMessageClient (e.g. quitServer())
 *   - fixed some includes
 *
 *   Revision 1.1  2010/11/10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.6  2009/08/11 18:50:33  guettler
 *   stopTimer optimised, added discoverXBeeHardwareVersionTimeout to QGlobalData
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
 *   Revision 1.3  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.2  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *   Revision 1.1  2007/07/10 15:02:33  robot3
 *   *** empty log message ***
 *
 *
 ***************************************************************************/
#include "QGlobalData.h"
#include "QECBCommunicator.h"

namespace lpzrobots {

  QGlobalData::QGlobalData() : Configurable("ECBRobots Control Parameters", "$Id$") {
    // set default communication values and register some of them to be configurable
    baudrate = 57600;
    portName = std::string("/dev/ttyUSB0");
    addParameterDef("maxfailures", &maxFailures, 4, 0,100, "number of allowed discarded messages before reset(ting) the device");
    addParameterDef("serialreadtimeout", &serialReadTimeout, 80, 1, 600);
    addParameterDef("cycletime", &cycleTime, 50, 0, 300);
    addParameterDef("noiseFactor", &noise, 0.05, 0, 1, "global noise factor used for all controllers added by their used wiring to motor and/or sensor values");
    controlStep = 0;
    addParameterDef("benchmarkmode", &benchmarkMode, false);
    addParameterDef("testmode", &testMode, false);
    paused = false;
    comm = 0;
    // deploy the Communicator
    comm = new QECBCommunicator(*this);
  }

  QGlobalData::~QGlobalData() {
    if (comm) {
      if (comm->isRunning()) {
        comm->shutdown();
        comm->quit();
      }
      while (comm->isRunning());
      delete comm;
    }
  }

  void QGlobalData::textLog(QString log, LOG_TYPE logType) {
    switch (logType) {
      case LOG_ERROR: // always log errors
        emit sig_textLog(log); // forward
        break;
      case LOG_WARNING:
        if (warningOutput)
          emit sig_textLog(log); // forward
        break;
      case LOG_VERBOSE:
        if (verboseOutput)
          emit sig_textLog(log); // forward
        break;
      case LOG_DEBUG:
        if (debugOutput)
          emit sig_textLog(log); // forward
        break;
    }
  }

} // namespace lpzrobots
