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
 *   Revision 1.3  2010-11-10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.2  2009/08/11 19:27:47  guettler
 *   add support for paused threads
 *
 *   Revision 1.1  2009/08/11 15:49:05  guettler
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
 *                                                                         *
 ***************************************************************************/
#ifndef __CTHREAD_H_
#define __CTHREAD_H_

#include <pthread.h>
#include <string>


using namespace std;

namespace lpzrobots
{

  struct GlobalData;

  /** Thread-Klassenprototyp */
  class CThread
  {

    public:

      CThread(std::string name, bool debug = false);

      virtual ~CThread()
      {
        stopandwait();
      }
      ;

      /// thread is running?
      bool is_running()
      {
        return m_is_running;
      }

    protected:

      /// start  thread
      void start();
      /// stop  thread and wait for the thread to terminate
      void stopandwait();
      /// stop  thread (call also be called from inside)
      void stop();

      /**
       *
       * @return
       */
      virtual bool loop() = 0;

      /// is called at the beginning after initialisation
      virtual bool initialise() = 0;

      /**
       * pauses the thread if he loops (when he returns from loop())
       * @return
       */
      virtual void pause();

      /**
       * Resumes the thread (and he loops again ...)
       * @return
       */
      virtual void resume();

      virtual void setConfig(bool debug = false);

      bool isTerminated()
      {
        return terminated;
      }

      static void* CThread_run(void* p);

    protected:
      std::string name;
      bool debug;

    private:
      bool terminated;
      bool m_is_joined;
      bool m_is_running;
      bool m_is_paused;

      pthread_t thread;

      bool internInit();

      /// thread function
      bool run();
  };

}

#endif
