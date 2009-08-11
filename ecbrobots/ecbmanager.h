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
 *   Revision 1.7  2009-08-11 19:26:45  guettler
 *   added TODO
 *
 *   Revision 1.6  2009/08/11 15:49:05  guettler
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
 *   Revision 1.4  2008/07/16 15:16:55  robot1
 *   minor bugfixes
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
 *                                                                         *
 ***************************************************************************/
#ifndef __ECBMANAGER_H
#define __ECBMANAGER_H

#include <selforg/configurable.h>

#include "globaldata.h"
#include "ecbagent.h"
#include <termios.h>

namespace lpzrobots {


// forward declaration begin
class ECBCommunicator;
// forward declaration end

class ECBManager : public Configurable {

public:

  ECBManager();

  /**
  * Use this constructor if you like to use your own ECBCommunicator
  */
  ECBManager(ECBCommunicator* comm);

  virtual ~ECBManager();

  /**
   * This starts the ECBManager. Do not overload it.
   * @return
   */
  bool run(int argc, char** argv);


  /// CONFIGURABLE INTERFACE
  // TODO: CHECK AND REMOVE
  virtual paramval getParam(const paramkey& key) const;

  virtual bool setParam(const paramkey& key, paramval val);

  virtual paramlist getParamList() const;

protected:
  bool simulation_time_reached;

    /**
   * Use this function to define the robots, controller, wiring of
   * the agents.
   * @param global The struct which should contain all neccessary objects
   * like Agents
   * @return true if all is ok!
   */
  virtual bool start(GlobalData& global) = 0;

   /** optional additional callback function which is called every
    * simulation step.
    * To use this method, just overload it.
    * @param globalData The struct which contains all neccessary objects
    * like Agents
    * @param pause indicates that simulation is paused
    * @param control indicates that robots have been controlled this timestep (default: true)
    */
  void addCallback ( GlobalData& globalData,bool pause, bool control ) {};


    /** add own key handling stuff here, just insert some case values
     * To use this method, just overload it
     * @param globalData The struct which contains all neccessary objects
     * like Agents
     * @param key The key number which is pressed
     * @return true if this method could handle the key,
     * otherwise return false
     */
  virtual bool command ( GlobalData& globalData, int key) { return false; };

  // Helper
  int contains(char **list, int len,  const char *str){
    for(int i=0; i<len; i++){
      if(strcmp(list[i],str) == 0) return i+1;
    }
    return 0;
  }


private:
  ECBCommunicator* comm;
  GlobalData globalData;
  bool commInitialized;

  struct termios term_orig;

  virtual void handleStartParameters ( int argc, char** argv );

  virtual bool loop();

  virtual void initConsole();
  virtual void restoreConsole();
  
};


}

#endif
