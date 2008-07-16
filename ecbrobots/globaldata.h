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
 *   Revision 1.3  2008-07-16 07:38:42  robot1
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

#include <selforg/agent.h>


class Configurable;

namespace lpzrobots {

class ECBCommunicator;

typedef std::vector<Configurable*> ConfigList;       ///
typedef std::vector<Agent*> AgentList;         ///
typedef std::vector<PlotOption> PlotOptionList;

typedef unsigned char uint8;

// pretend to have some windows data types
typedef unsigned long DWORD;
typedef unsigned int UINT;

/**
  Data structure holding all essential global information.
*/
struct GlobalData
{
  GlobalData() {
  // set default communication values
    baudrate = 38400;
    portName = std::string("/dev/ttyS0");
    masterAddress=0;
    maxFailures=4;
    serialReadTimeout=50;
    cycleTime=50;
    noise=0.01;
    simStep=0;
    benchmarkMode=false;
    debug=false;
    verbose=false;
    testMode=false;
  }

  ConfigList configs;
  AgentList agents;
  PlotOptionList plotOptions;

  double noise;
  long simStep;
  bool benchmarkMode;
  bool pause;
  bool debug;
  bool verbose;
  bool testMode;

  // global settings for serial communication
  int baudrate;
  std::string portName;
  short masterAddress;
  int maxFailures;

  long cycleTime;
  int serialReadTimeout;
  ECBCommunicator* comm;

};

}

#endif
