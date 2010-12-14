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
 *   Revision 1.5  2010-12-14 10:10:12  guettler
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

    // init the PlotOptionEngine
    globalData.textLog("QECBManager: starting the PlotOptionEngine...");

  }

  void QECBManager::cleanup() {
    for (ECBAgentList::iterator i = globalData.agents.begin(); i != globalData.agents.end(); i++) {
      delete (*i);
    }
    globalData.agents.clear();
    globalData.configs.clear();
    globalData.configs.push_back(&globalData);
    globalData.plotOptions.clear();
  }

  /// CONFIGURABLE INTERFACE

  void QECBManager::sl_textLog(QString log) {
    globalData.textLog(log); // forwarding
  }

  void QECBManager::startLoop() {
    emit sig_communicationStateWillChange(QECBCommunicator::STATE_RUNNING);
    globalData.textLog("QECBManager: calling start function...");
    globalData.comm->initialize();
    commInitialized = true;
    start(globalData);
    globalData.textLog("QECBManager: starting communication...");
    globalData.comm->start();
    emit sig_communicationStateChanged(QECBCommunicator::STATE_RUNNING);
  }

  void QECBManager::stopLoop() {
    emit sig_communicationStateWillChange(QECBCommunicator::STATE_STOPPED);
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
          emit sig_communicationStateWillChange(QECBCommunicator::STATE_RUNNING);
          globalData.paused = false;
          emit sig_communicationStateChanged(QECBCommunicator::STATE_RUNNING);
        } else { // not paused yet
          emit sig_communicationStateWillChange(QECBCommunicator::STATE_PAUSED);
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
      case EVENT_STOP_LOOP: // stop
      default:
        stopLoop();
        break;
    }
  }

} // namespace lpzrobots
