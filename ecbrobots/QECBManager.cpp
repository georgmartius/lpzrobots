/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    wolfgang.rabe@01019freenet.de                                        *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.12  2011-04-04 09:25:59  guettler
 *   - loopStateLabel now updates each control step, displaying current status and control step
 *   - addCallbackStep(..) can now call stopLoop() to stop control loop
 *
 *   Revision 1.11  2011/03/26 08:58:38  guettler
 *   - code adapted to:
 *     PlotOptionEngine initializes PlotOptions only if PlotOption itself is initialized
 *
 *   Revision 1.10  2011/03/25 22:53:07  guettler
 *   - autoload function did not allow changing the configurable values during the
 *     initialization phase of the loop, this is now supported, so
 *   - if you like to add configurable parameters which are used in
 *     QECBManager::start(..), just add them to globaldata, then the parameters can
 *     be changed before starting the control loop.
 *   - All other parameters of the ECBAgent and it's configurable childs (Robot, ECB,
 *     Controller, ...) are only configurable while the control loop is running (or paused).
 *
 *   Revision 1.9  2011/03/25 21:27:37  guettler
 *   - cleanup of agentList and communicator now handled, fixes the problem that
 *     PlotOptionEngine does not close all open pipes (e.g. GUILogger received no
 *     #QUIT)
 *
 *   Revision 1.8  2011/02/11 12:16:28  guettler
 *   - new signal/slots initlializationOfAgentDone(ECBAgent*) and stepDone() implemented, forwarded to QECBManager
 *   - QECBManager now supports addCallback() function again, divided into addCallbackAgentInitialized(...) and addCallbackStep(...)
 *
 *   Revision 1.7  2011/01/27 09:31:15  guettler
 *   - Guilogger opens again when desired by user (menu or CTRL+G)
 *
 *   Revision 1.6  2011/01/24 14:17:57  guettler
 *   - new menu entry start/stop MatrixViz
 *
 *   Revision 1.5  2010/12/14 10:10:12  guettler
 *   -autoload/autosave now uses only one xml file
 *   -fixed getName of TileWidget which produced invisible widgets in xml files
 *
 *   Revision 1.4  2010/12/09 17:00:08  wrabe
 *   - load / save function of ConfigurableState (configurable + GUI)
 *   - autoload / autosave function of ConfigurableState (configurable
 *     + GUI)
 *   - handling of equal Configurable names implemented for autoload
 *     and -save
 *   - bugfixing
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
 *                                                *
 *                                                                         *
 ***************************************************************************/

#include "QECBManager.h"

#include <selforg/agent.h>

#include "QGlobalData.h"

namespace lpzrobots {

  QECBManager::QECBManager(int argc, char** argv) :
    simulation_time_reached(false), commInitialized(false), argc(argc), argv(argv) {
    globalData.configs.push_back(&globalData);
  }

  QECBManager::~QECBManager() {
    FOREACH(AgentList, globalData.agents, a) {
      if (*a)
        delete (*a);
    }
  }

  QGlobalData& QECBManager::getGlobalData() {
    return globalData;
  }

  void QECBManager::handleStartParameters() {
    if (contains(argv, argc, "-g") != 0)
      globalData.plotOptions.push_back(PlotOption(GuiLogger));

    if (contains(argv, argc, "-f") != 0)
      globalData.plotOptions.push_back(PlotOption(File));

    if (contains(argv, argc, "-s") != 0)
      globalData.plotOptions.push_back(PlotOption(SoundMan));

    if (contains(argv, argc, "-u") != 0)
      globalData.plotOptions.push_back(PlotOption(ECBRobotGUI));

    if (contains(argv, argc, "-bench") != 0)
      globalData.benchmarkMode = true;

    if (contains(argv, argc, "-h") != 0) {
      printf("Usage: %s [-g] [-f] [-s] [-b] [-p port] [-v] [-d] [-bench] [-h]\n", argv[0]);
      printf("\t-g\tstart guilogger\n");
      printf("\t-f\twrite logfile\n");
      printf("\t-s\tenable SoundMan\n");
      printf("\t-u\tenable ECBRobotGUI\n");
      printf("\t-b baud\tset baud rate\n");
      printf("\t-p port\tuse given serial port ( e.g. /dev/ttyUSB0 ) \n");
      printf("\t-bench\tenable benchmark mode\n");
      printf("\n\t-h\tdisplay this help\n");
      exit(0);
    }

    int index = contains(argv, argc, "-p");

    if (index && index < argc) {
      globalData.portName = std::string(argv[index]);
      globalData.textLog("Using port " + QString(globalData.portName.c_str()));
    }

    index = contains(argv, argc, "-b");

    if (index && index < argc) {
      globalData.baudrate = atoi(argv[index]);
      globalData.textLog("Using baud rate " + globalData.baudrate);
    }
  }

  void QECBManager::initialize() {

    // neccessary values are stored in globalData
    globalData.textLog("QECBManager: handling console parameters...");
    handleStartParameters();

    connect(globalData.comm, SIGNAL(sig_initializationOfAgentDone(ECBAgent*)), this, SLOT(sl_initializationOfAgentDone(ECBAgent*)));
    connect(globalData.comm, SIGNAL(sig_stepDone()), this, SLOT(sl_stepDone()));
  }

  void QECBManager::cleanup() {
    for (ECBAgentList::iterator i = globalData.agents.begin(); i != globalData.agents.end(); i++) {
      delete (*i);
    }
    globalData.agents.clear();
    globalData.configs.clear();
    globalData.configs.push_back(&globalData);
    globalData.plotOptions.clear();
    globalData.controlStep = 0;
  }

  /// CONFIGURABLE INTERFACE

  void QECBManager::sl_textLog(QString log) {
    globalData.textLog(log); // forwarding
  }

  void QECBManager::startLoop() {
    emit sig_communicationStateWillChange(QECBCommunicator::STATE_STOPPED, QECBCommunicator::STATE_RUNNING);
    globalData.textLog("QECBManager: calling start function...");
    globalData.comm->initialize();
    commInitialized = true;
    start(globalData);
    globalData.textLog("QECBManager: starting communication...");
    globalData.comm->start();
    emit sig_communicationStateChanged(QECBCommunicator::STATE_RUNNING);
  }

  void QECBManager::stopLoop() {
    emit sig_communicationStateWillChange(QECBCommunicator::STATE_RUNNING, QECBCommunicator::STATE_STOPPED);
    globalData.textLog("QECBManager: stopping communication...");
    globalData.comm->shutdown();
    cleanup();
    emit sig_communicationStateChanged(QECBCommunicator::STATE_STOPPED);
  }

  void QECBManager::sl_GUIEventHandler(int eventId) {
    switch (eventId) {
      case EVENT_START_LOOP: // start loop
        startLoop();
        break;
      case EVENT_RESTART_LOOP: // restart
        stopLoop();
        startLoop();
        break;
      case EVENT_PAUSE_LOOP: // paused
        if (globalData.paused) { // paused, so continue now
          emit sig_communicationStateWillChange(QECBCommunicator::STATE_PAUSED, QECBCommunicator::STATE_RUNNING);
          globalData.paused = false;
          emit sig_communicationStateChanged(QECBCommunicator::STATE_RUNNING);
        } else { // not paused yet
          emit sig_communicationStateWillChange(QECBCommunicator::STATE_PAUSED, QECBCommunicator::STATE_RUNNING);
          globalData.paused = true;
          emit sig_communicationStateChanged(QECBCommunicator::STATE_PAUSED);
        }
        break;
      case EVENT_START_GUILOGGER:
        for (ECBAgentList::iterator i = globalData.agents.begin(); i != globalData.agents.end(); i++) {
          if (!(*i)->removePlotOption(GuiLogger)) {
            PlotOption po(GuiLogger, 5);
            (*i)->addAndInitPlotOption(po);
          }
        }
        globalData.textLog("All Guiloggers startet/stopped.");
        break;
      case EVENT_START_MATRIXVIZ:
        for (ECBAgentList::iterator i = globalData.agents.begin(); i != globalData.agents.end(); i++) {
          if (!(*i)->removePlotOption(MatrixViz)) {
            PlotOption po(MatrixViz, 5);
            (*i)->addAndInitPlotOption(po);
          }
        }
        globalData.textLog("All MatrixVizs startet/stopped.");
        break;
      case EVENT_STOP_LOOP: // stop
      default:
        stopLoop();
        break;
    }
  }


  void QECBManager::sl_stepDone() {
    // param control is reserved as a not yet implemented feature
    addCallbackStep(globalData,globalData.paused, true);
  }

  void QECBManager::sl_initializationOfAgentDone(ECBAgent* agent) {
    addCallbackAgentInitialized(globalData, agent);
  }



} // namespace lpzrobots
