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
 *   Revision 1.2  2009-08-11 18:10:57  guettler
 *   more info in debug mode (threadid)
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
 *   - GlobalData, ECBCommunicator is now configurable
 *   - ECBAgent rewritten: new PlotOptionEngine support, adapted to new WiredController structure
 *   - ECBRobot is now Inspectables (uses new infoLines functionality)
 *   - ECB now supports dnsNames and new communication protocol via Mediator
 *   - Better describing command definitions
 *   - SphericalRobotECB: adapted to new ECB structure, to be tested
 *
 *   Revision 1.5  2009/03/25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.4  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.3  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.2  2008/04/08 09:09:09  martius
 *   fixed globaldata to pointer in classes
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#include "CThread.h"

#include "globaldata.h"
#include <iostream>

namespace lpzrobots
{

  CThread::CThread(std::string name, bool debug) :
    name(name), debug(debug), terminated(true), m_is_joined(false), m_is_running(false)
  {
  };

  void CThread::setConfig(bool debug)
  {
    this->debug = debug;
  }

  /// start thread
  void CThread::start()
  {
    if (debug)
      cout << name << "(CThread): called start!"  << endl;
    if (!terminated) // already running!
    {
      if (debug)
        cout << " - but I am already running!" << endl;
      return;
    }
    else if (debug)
      cout << endl;
    m_is_running = true;
    m_is_joined = false;
    terminated = false;
    // start thread using this static function
    pthread_create(&thread, NULL, CThread_run, this);
  }
  ;

  /// stop communication
  void CThread::stopandwait()
  {
    if (debug)
      cout << name << "(CThread): stop and wait! id="<< this->thread  << endl;
    if (!m_is_joined)
    {
      // set stop signal
      terminated = true;
      usleep(10);
      pthread_testcancel();
      usleep(10);
      //      pthread_cancel(thread);
      pthread_join(thread, NULL);
      m_is_joined = true;
      m_is_running = false;
    }
  }
  ;

  /// stop  communication
  void CThread::stop()
  {
    if (terminated)
      return;
    if (debug)
      std::cout << name << "(CThread): stop: id=" << this->thread << std::endl;
    terminated = true;
    pthread_testcancel();
  }
  ;

  // thread function
  bool CThread::run()
  {
    if (debug)
      cout << name << "(CThread): run! id=" << thread << endl;

    if (!internInit())
      stop();

    // init function of the derived class
    if (!initialise())
      stop();

    if (debug)
      std::cout << name << "(CThread): finished initialising. Starting the loop. id=" << thread << std::endl;

    /* main loop which calls the loop
     * function of the derived class
     */
    bool inLoop = true;

    while (!terminated && inLoop)
    {
      pthread_testcancel();
      inLoop = loop();
    }
    if (debug)
      std::cout << name << "(CThread): End of loop reached. id=" << thread << std::endl;

    m_is_running = false;
    return true;
  }
  ;

  bool CThread::internInit()
  {
    if (debug)
      std::cout << name << "(CThread): internal initialising... id=" << this->thread << std::endl;

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, 0);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, 0);

    return true;
  }

  /// redirection function, because we can't call member function direct
  void* CThread::CThread_run(void* p)
  {
    bool rv = false;
    CThread* ct = dynamic_cast<CThread*> ((CThread*) p);
    if (ct)
      rv = ct->run();
    else
    {
      cerr << "Error getting CThread to run!!!" << endl;
    }
    pthread_exit(&rv);
  }

} // end namespace lpzrobots


