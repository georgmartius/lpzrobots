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
 *   Revision 1.8  2011-04-04 09:25:59  guettler
 *   - loopStateLabel now updates each control step, displaying current status and control step
 *   - addCallbackStep(..) can now call stopLoop() to stop control loop
 *
 *   Revision 1.7  2011/03/25 22:53:07  guettler
 *   - autoload function did not allow changing the configurable values during the
 *     initialization phase of the loop, this is now supported, so
 *   - if you like to add configurable parameters which are used in
 *     QECBManager::start(..), just add them to globaldata, then the parameters can
 *     be changed before starting the control loop.
 *   - All other parameters of the ECBAgent and it's configurable childs (Robot, ECB,
 *     Controller, ...) are only configurable while the control loop is running (or paused).
 *
 *   Revision 1.6  2011/02/11 12:16:28  guettler
 *   - new signal/slots initlializationOfAgentDone(ECBAgent*) and stepDone() implemented, forwarded to QECBManager
 *   - QECBManager now supports addCallback() function again, divided into addCallbackAgentInitialized(...) and addCallbackStep(...)
 *
 *   Revision 1.5  2011/01/24 14:17:57  guettler
 *   - new menu entry start/stop MatrixViz
 *
 *   Revision 1.4  2010/12/14 10:10:12  guettler
 *   -autoload/autosave now uses only one xml file
 *   -fixed getName of TileWidget which produced invisible widgets in xml files
 *
 *   Revision 1.3  2010/12/09 17:00:08  wrabe
 *   - load / save function of ConfigurableState (configurable + GUI)
 *   - autoload / autosave function of ConfigurableState (configurable
 *     + GUI)
 *   - handling of equal Configurable names implemented for autoload
 *     and -save
 *   - bugfixing
 *
 *   Revision 1.2  2010/11/26 12:22:37  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *   Revision 1.1  2010/11/10 09:32:00  guettler
 *   - port to Qt part 1
 *                                                *
 *                                                                         *
 ***************************************************************************/

#ifndef __QECBMANAGER_H_
#define __QECBMANAGER_H_

#include <QObject>

#include <selforg/configurable.h>

#include "QGlobalData.h"
#include "ecbagent.h"

#include "QECBCommunicator.h"

namespace lpzrobots {

  // forward declaration begin
  class QECBCommunicator;
  // forward declaration end

  class QECBManager : public QObject {

    Q_OBJECT

    public:

      QECBManager(int argc, char** argv);

      virtual ~QECBManager();

      /**
       * This initalizes and starts the ECBManager. Do not overload it.
       */
      void initialize();

      virtual QGlobalData& getGlobalData();

      enum EVENT {
        EVENT_START_LOOP,
        EVENT_RESTART_LOOP,
        EVENT_PAUSE_LOOP,
        EVENT_STOP_LOOP,
        EVENT_START_GUILOGGER,
        EVENT_START_MATRIXVIZ
      };

    public slots:

      void sl_GUIEventHandler(int);

      void sl_textLog(QString log);  // forwarded to QECBRobotsWindow, using sig_textLog(QString log)
      void sl_stepDone();
      void sl_initializationOfAgentDone(ECBAgent* agent);

    signals:

      void sig_communicationStateChanged(QECBCommunicator::ECBCommunicationState);
      void sig_communicationStateWillChange(QECBCommunicator::ECBCommunicationState fromState, QECBCommunicator::ECBCommunicationState toState);
      void sig_textLog(QString log);

    protected:
      bool simulation_time_reached;

      /**
       * Use this function to define the robots, controller, wiring of
       * the agents.
       * @param global The struct which should contain all neccessary objects
       * like Agents
       * @return true if all is ok!
       */
      virtual bool start(QGlobalData& global) = 0;


      /** optional additional callback function which is called when closed loop
       * is established (hardware ECBs, ECBAgent etc. are initialized)
       * To use this method, just overload it.
       * @param globalData The struct which contains all neccessary objects
       * like Agents
       * @agentInitialized the ECBAgent which is intialized
       */
      virtual void addCallbackAgentInitialized(QGlobalData& globalData, ECBAgent* agentInitialized) {
      }


      /** optional additional callback function which is called every
       * simulation step.
       * To use this method, just overload it.
       * @param globalData The struct which contains all neccessary objects
       * like Agents
       * @param paused indicates that simulation is paused
       * @param control indicates that robots have been controlled this timestep (default: true)
       */
      virtual void addCallbackStep(QGlobalData& globalData, bool pause, bool control) {
      }



      /** add own key handling stuff here, just insert some case values
       * To use this method, just overload it
       * @param globalData The struct which contains all neccessary objects
       * like Agents
       * @param key The key number which is pressed
       * @return true if this method could handle the key,
       * otherwise return false
       */
      virtual bool command(QGlobalData& globalData, int key) {
        return false;
      }
      ;

      /**
       * Stops the control loop which can be called from addCallbackStep.
       */
      virtual void stopLoop();

      // Helper
      int contains(char **list, int len, const char *str) {
        for (int i = 0; i < len; i++) {
          if (strcmp(list[i], str) == 0)
            return i + 1;
        }
        return 0;
      }

    private:
      QGlobalData globalData;
      bool commInitialized;

      int argc;
      char** argv;


      /**
       * Destroys all created ECBs, Agents and Controller.
       * After that you can call the start() function again.
       */
      void cleanup();



      virtual void handleStartParameters();

      virtual void startLoop();

  };

} // namespace lpzrobots

#endif /* __QECBMANAGER_H_ */
