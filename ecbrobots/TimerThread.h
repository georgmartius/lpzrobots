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
 *  Revision 1.2  2010-11-10 09:32:00  guettler
 *  - port to Qt part 1
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
#ifndef _TIMERTHREAD_H_
#define _TIMERTHREAD_H_

/*
 *
 */
#include "CThread.h"
#include <selforg/backcaller.h>

namespace lpzrobots
{
  
  class TimerThread : public CThread, public BackCaller
  {
    public:
      static const CallbackableType TIMER_EXPIRED = 45322;

      TimerThread(CallbackableType typeToCallback = TIMER_EXPIRED);
      virtual ~TimerThread();

      virtual bool loop();

      virtual void startTimer(unsigned long msec, bool cycledMode = false);

      virtual void setTimer(unsigned long msec, bool cycledMode = false);

      virtual void restartTimer();

      virtual void stopTimer();

      virtual bool initialise();


    private:
      unsigned long timeSpan;
      unsigned long stopTime;
      bool cycledMode;
      CallbackableType callbackType;

      unsigned long timeOfDayinMS();

  };

}

#endif /* _TIMERTHREAD_H_ */
