/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *  
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.3  2010-11-10 09:32:00  guettler
 *  - port to Qt part 1
 *
 *  Revision 1.2  2009/08/11 18:10:25  guettler
 *  BUGFIX: stopTimer
 *
 *  Revision 1.1  2009/08/11 15:49:05  guettler
 *  Current development state:
 *  - Support of communication protocols for XBee Series 1, XBee Series 2 and cable mode
 *  - merged code base from ecb_robots and Wolgang Rabes communication handling;
 *    ECBCommunicator (almost) entirely rewritten: Use of Mediator (MediatorCollegues: ECB),
 *    Callbackble (BackCaller: SerialPortThread)
 *  - New CThread for easy dealing with threads (is using pthreads)
 *  - New TimerThreads for timed event handling
 *  - SerialPortThread now replaces the cserialthread
 *  - QGlobalData, ECBCommunicator is now configurable
 *  - ECBAgent rewritten: new PlotOptionEngine support, adapted to new WiredController structure
 *  - ECBRobot is now Inspectables (uses new infoLines functionality)
 *  - ECB now supports dnsNames and new communication protocol via Mediator
 *  - Better describing command definitions
 *  - SphericalRobotECB: adapted to new ECB structure, to be tested
 *										   *
 *                                                                         *
 **************************************************************************/

#include "TimerThread.h"
#include <sys/time.h>
#include <iostream>

namespace lpzrobots {
  
  TimerThread::TimerThread(CallbackableType typeToCallback) :
    CThread("TimerThread", false), timeSpan(0), stopTime(0), cycledMode(false), callbackType(typeToCallback) {
  }
  
  TimerThread::~TimerThread() {
    stopTimer();
  }

  bool TimerThread::loop() {
    if (timeOfDayinMS() >= stopTime) {
      if (debug)
        cout << name << ": stop loop!" << endl;
      callBack(callbackType);
      stopTime = timeOfDayinMS() + timeSpan;
      return cycledMode;
    }
    usleep(500);
    return true;
  }

  void TimerThread::startTimer(unsigned long msec, bool cycledMode /* = false; */) {
    if (debug)
      cout << name << ": startTimer()!" << endl;
    this->timeSpan = msec;
    this->stopTime = timeOfDayinMS() + msec;
    this->cycledMode = cycledMode;
    start();
  }

  void TimerThread::setTimer(unsigned long msec, bool cycledMode /* = false; */) {
    if (debug)
      cout << name << ": setTimer()!" << endl;
    this->timeSpan = msec;
    this->stopTime = timeOfDayinMS() + msec;
    this->cycledMode = cycledMode;
  }

  void TimerThread::restartTimer() {
    if (debug)
      cout << name << ": restartTimer()!" << endl;
    if (timeSpan == 0 && debug)
      cout << name << ": Warning - Restart without setTimer() or startTimer() called!" << endl;
    stop();
    stopTime = timeOfDayinMS() + timeSpan;
    start();
  }

  void TimerThread::stopTimer() {
    if (debug)
      cout << name << ": stopTimer()!" << endl;
    stop();
  }

  unsigned long TimerThread::timeOfDayinMS() {
    struct timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec * 1000 + t.tv_usec / 1000;
  }

  bool TimerThread::initialise() {
    return true;
  }

}
