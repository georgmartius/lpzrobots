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
 *   Revision 1.6  2011-04-04 09:26:36  guettler
 *   - renamed simStep to controlStep
 *   - loopStateLabel now updates each control step
 *
 *   Revision 1.5  2011/02/11 12:16:28  guettler
 *   - new signal/slots initlializationOfAgentDone(ECBAgent*) and stepDone() implemented, forwarded to QECBManager
 *   - QECBManager now supports addCallback() function again, divided into addCallbackAgentInitialized(...) and addCallbackStep(...)
 *
 *   Revision 1.4  2011/01/24 16:58:25  guettler
 *   - QMessageDispatchServer is now informed when client app closes itself
 *   - QMessageDispatchWindow actually closes if client app closes itself
 *   - hint: this should late be
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

#ifndef QECBCOMMUNICATOR_H_
#define QECBCOMMUNICATOR_H_

#include <selforg/mediator.h>
#include "QAbstractMessageClient.h"
#include "ECBCommunicationData.h"
#include "ECBCommunicationEvent.h"

namespace lpzrobots {

  class QGlobalData;
  class ECB;
  class ECBAgent;

  class QECBCommunicator : public QAbstractMessageClient, public Mediator {

    Q_OBJECT

    public:
      QECBCommunicator(QGlobalData& data);
      virtual ~QECBCommunicator();
      virtual void run();

      enum ECBCommunicationState {
        // specific states
        STATE_NOT_INITIALISED, //!< QECBCommunicator not initilized yet
        STATE_INITIALIZED, //!< connection to AbstractQMessageDispatchServer established, but loop not started yet
        STATE_READY_FOR_STEP_OVER_AGENTS, //!< indicates that all ECBs are updated and the loop over the agents can begin
        STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS, //!< awaiting package with current sensor informations of current handled ECB
        STATE_READY_FOR_SENDING_PACKAGE_MOTORS, //!< indicates that the thread is ready to send new motor values to an ECB
        // general states, they may include several specific states
        STATE_RUNNING, //!< state which indicates that the loop is running
        STATE_PAUSED, //!< state which indicates that all actions are paused
        STATE_STOPPED
      //!< state which indicates that all actions are stopped, quitted and leaved. Bye bye.
      };

      virtual bool initialize();

      /**
       * Closes the communication to the hardware related ECBs
       */
      virtual void shutdown();

      /**
       * defines a test mode callback function, which is called,
       * if globalData.testMode is true
       * @return false if you would like to stop the test mode (ECBCommunicator will stop).
       */
      virtual bool testModeCallback() {
        return false;
      }
      ;

      virtual ECBCommunicationState getCurrentCommState() { return currentCommState; }

    public slots:

      virtual void sl_messageReceived(struct _communicationMessage msg);
      virtual void sl_quitServer();
      virtual void sl_quitClient() { emit sig_quitClient(); }

    signals:
      void sig_sendMessage(struct _communicationMessage msg);
      void sig_initializationOfAgentDone(ECBAgent* agentInitialized);
      void sig_stepDone();

      void sig_quitServer();
      void sig_quitClient();

    private slots:
      void sl_TimerExpired();

    private:
      QGlobalData& globalData;
      ECBCommunicationState currentCommState;
      // bool paused is stored in QGlobalData
      bool communicationRunning;
      bool stopCommunication;

      unsigned long currentTime;
      unsigned long answerTime;
      long realtimeoffset;
      long lastBenchmarkTime;

      unsigned int currentECBIndex;

      QTimer timer;

      /**
       * Converts the package to send into the message format used by AbstractQMessageClient
       * @param commData CommunicationData to send
       * @param sourceECB which contains the DNSName
       */
      void send_CommandPackage(ECBCommunicationData commData, ECB* sourceECB);

      virtual void mediatorInformed(MediatorCollegue* source, MediatorEvent* event);

      void dispatchPackageCommand(ECBCommunicationEvent* event);
      void loopCallback(); // time synchronization
      long timeOfDayinMS();
      void resetSyncTimer();
      QString printBuffer(const QByteArray buffer);
      QString printBuffer(const uint8 buffer);
      QString printBuffer(const uint8 buffer[]);
      QString toHexNumberString(uint64 value, uint numberDigits);



  };

} // namespace lpzrobots

#endif /* QECBCOMMUNICATOR_H_ */
