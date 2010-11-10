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
 *   Revision 1.1  2010-11-10 09:32:00  guettler
 *   - port to Qt part 1
 *                                                *
 *                                                                         *
 ***************************************************************************/

#include "QECBManager.h"

#include <selforg/agent.h>

#include "QGlobalData.h"

namespace lpzrobots {

  QECBManager::QECBManager(int argc, char** argv) :
    Configurable("QECBManager", "$ID$"), simulation_time_reached(false), commInitialized(false), argc(argc), argv(argv) {
    globalData.configs.push_back(this);

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

    // deploy the Communicator
    // neccessary values are stored in globalData
    globalData.textLog("QECBManager: deploying QECBCommunicator...");
    globalData.comm = new QECBCommunicator(globalData);
    ;

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
    globalData.plotOptions.clear();
  }

  /// CONFIGURABLE INTERFACE

  Configurable::paramval QECBManager::getParam(const paramkey& key) const {
    /*if(key == "noise") return noise;
     else if(key == "cycletime") return cycletime;
     else if(key == "reset") return 0;
     else */return Configurable::getParam(key);
  }

  bool QECBManager::setParam(const paramkey& key, paramval val) {
    /*if(key == "noise") noise = val;
     else if(key == "cycletime"){
     cycletime=(long)val;
     } else if(key == "reset"){
     doReset=true;
     } else*/
    return Configurable::setParam(key, val);
    //return true;
  }

  Configurable::paramlist QECBManager::getParamList() const {
    paramlist list;
    /*list += pair<paramkey, paramval> (string("noise"), noise);
     list += pair<paramkey, paramval> (string("cycletime"), cycletime);
     */
    //list += pair<paramkey, paramval> (std::string("reset"), 0);
    return list;
  }

  void QECBManager::sl_textLog(QString log) {
    globalData.textLog(log); // forwarding
  }

  void QECBManager::startLoop() {
    globalData.textLog("QECBManager: calling start function...");
    globalData.comm->initialize();
    commInitialized = true;
    start(globalData);
    globalData.textLog("QECBManager: starting communication...");
    globalData.comm->start();
    emit sig_communicationStateChanged(QECBCommunicator::STATE_RUNNING);
  }

  void QECBManager::stopLoop() {
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
          globalData.paused = false;
          emit sig_communicationStateChanged(QECBCommunicator::STATE_RUNNING);
        } else { // not paused yet
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
