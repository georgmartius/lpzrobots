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
 *   Revision 1.5  2009-08-11 15:49:05  guettler
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
#ifndef __GLOBALDATA_H
#define __GLOBALDATA_H

#include <vector>
#include <string>

//#include "ecbagent.h"
#include <selforg/configurable.h>
#include <selforg/plotoption.h>

namespace lpzrobots
{

  class ECBCommunicator;
  class ECBAgent;
  class ECBRobot;

  typedef std::vector<Configurable*> ConfigList; //!< list of Configurables
  typedef std::vector<ECBAgent*> ECBAgentList; //!< list of ECBAgents
  typedef ECBAgentList AgentList; //!< for compatibility reason: List of (ECB)Agents
  // 20090126; guettler: changed from vector to list for better method signature matching of WiredController
  typedef std::list<PlotOption> PlotOptionList;

  /**
   Data structure holding all essential global information.
   */
  struct GlobalData : public Configurable
  {
      GlobalData()
      {
        // set default communication values and register some of them to be configurable
        baudrate = 57600;
        portName = std::string("/dev/ttyUSB0");
        addParameterDef("maxfailures",&maxFailures,4);
        addParameterDef("serialreadtimeout", &serialReadTimeout, 80);
        addParameterDef("discovernodestimeout", &discoverNodesTimeout, 3000);
        addParameterDef("cycletime", &cycleTime, 50);
        addParameterDef("noise", &noise, 0.05);
        simStep = 0;
        addParameterDef("benchmarkmode", &benchmarkMode, false);
        addParameterDef("debug", &debug, false);
        addParameterDef("verbose", &verbose, false);
        addParameterDef("testmode", &testMode, false);
        pause = false;
        comm = 0;
        configs.push_back(this);
      }

      ConfigList configs;
      ECBAgentList agents;
      PlotOptionList plotOptions;

      paramval noise;
      long simStep;
      parambool benchmarkMode;
      parambool pause;
      parambool debug;
      parambool verbose;
      parambool testMode;

      // global settings for serial communication
      int baudrate;
      std::string portName;
      paramval maxFailures;

      paramval cycleTime; //!< time for one cycle from step to step (minimum)

      paramval serialReadTimeout; //!< read timeout for awaiting messages from ECBs in ms
      paramval discoverNodesTimeout; //!< read timeout for awaiting initial response (discover nodes - get node list) from each ECB/XBee in ms

      ECBCommunicator* comm;
  };

}

#endif
