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
 *   Revision 1.8  2011-04-04 09:26:36  guettler
 *   - renamed simStep to controlStep
 *   - loopStateLabel now updates each control step
 *
 *   Revision 1.7  2011/02/11 12:16:28  guettler
 *   - new signal/slots initlializationOfAgentDone(ECBAgent*) and stepDone() implemented, forwarded to QECBManager
 *   - QECBManager now supports addCallback() function again, divided into addCallbackAgentInitialized(...) and addCallbackStep(...)
 *
 *   Revision 1.6  2011/02/04 12:59:33  wrabe
 *   - corrected datalength of received package
 *
 *   Revision 1.5  2011/01/27 17:50:17  guettler
 *   - when timeout occurs, step for ECB ignored
 *
 *   Revision 1.4  2011/01/27 15:48:01  guettler
 *   - pause modus fixed
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

#include "QECBCommunicator.h"
#include "QGlobalData.h"
#include "ecb.h"
#include "assert.h"
#include <sys/time.h>

#include "ecbagent.h"
#include "commanddefs.h"

namespace lpzrobots {

  QECBCommunicator::QECBCommunicator(QGlobalData& globalData) :
    globalData(globalData), currentCommState(STATE_NOT_INITIALISED), communicationRunning(false), stopCommunication(
        false), currentTime(timeOfDayinMS()), answerTime(0), realtimeoffset(timeOfDayinMS()), lastBenchmarkTime(0),
        currentECBIndex(0) {
    connect(&timer, SIGNAL(timeout()), this, SLOT(sl_TimerExpired()));
  }

  QECBCommunicator::~QECBCommunicator() {
    // TODO Auto-generated destructor stub
  }

  void QECBCommunicator::shutdown() {
    stopCommunication = true;
    this->wait();
    currentCommState = STATE_NOT_INITIALISED;
    globalData.paused = false;
    timer.stop();
    // TODO: disconnect from AbstractQMessageDispatchServer
  }

  void QECBCommunicator::mediatorInformed(MediatorCollegue* source, MediatorEvent* event) {
    ECBCommunicationEvent* commEvent = static_cast<ECBCommunicationEvent*> (event);
    switch (commEvent->type) {
      case ECBCommunicationEvent::EVENT_REQUEST_SEND_COMMAND_PACKAGE: //!< ECB instance generated the package data to send out
        currentCommState = STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS;
        send_CommandPackage(commEvent->commPackage, static_cast<ECB*> (source));
        break;
      default:
        assert(false);
        break;
    }
  }

  void QECBCommunicator::send_CommandPackage(ECBCommunicationData commandPackage, ECB* sourceECB) {
    globalData.textLog("ECBCommunicator: Sending command package (" + printBuffer(commandPackage.command) + ") to "
        + sourceECB->getDNSName());
    _communicationMessage message;
    message.ecb_dns_name = sourceECB->getDNSName();
    message.data[0] = 0x01; // MsgGroup_ECB_ROBOT_FIRMWARE
    message.data[1] = commandPackage.command;
    message.data.append((const char*) commandPackage.data, commandPackage.dataLength);
    if (!globalData.paused)
      timer.start(globalData.serialReadTimeout);
    emit sig_sendMessage(message);
  }

  bool QECBCommunicator::initialize() {
    globalData.textLog("ECBCommunicator: initializing...");
    // TODO: establish connection to AbstractQMessageDispatchServer

    currentCommState = STATE_INITIALIZED;

    // after this the main loop starts
    return true;
  }

  void QECBCommunicator::run() {
    assert ( globalData.comm==this );
    if (currentCommState != STATE_INITIALIZED)
      globalData.textLog("QECBCommunicator: WARNING: initial communication state was not applied.");
    currentCommState = STATE_READY_FOR_SENDING_PACKAGE_MOTORS;
    stopCommunication = false;
    globalData.textLog("QECBCommunicator: entering communication loop...");
    while (!stopCommunication) {
      communicationRunning = true;
      switch (currentCommState) {
        case STATE_READY_FOR_STEP_OVER_AGENTS:
          if (!globalData.paused) {
            globalData.controlStep++;
            globalData.textLog("ECBCommunicator: loop! simStep=" + QString::number(globalData.controlStep));
            /// With this for loop all agents perform a controller step
            if (!globalData.testMode) {
              // sorgt dafür, dass der Zeittakt eingehalten wird:
              // Berechnung zu schnell -> warte,
              // Berechnung zu langsam -> Ausgabe, dass time leak stattfindet
              loopCallback();
              FOREACH ( AgentList,globalData.agents,a ) {
                ECBAgent* agent = *a;
                if (!agent->isInitialized() && agent->getRobot()->isInitialised()) {
                  agent->init();
                  emit sig_initializationOfAgentDone(agent);
                }
                if (agent->isInitialized())
                  agent->step(globalData.noise, globalData.controlStep);
              }
              emit sig_stepDone();
            } else {
              if (!this->testModeCallback())
                stopCommunication = true;
            }
            globalData.textLog("ECBCommunicator: AgentStep finished.");
            currentCommState = STATE_READY_FOR_SENDING_PACKAGE_MOTORS;
          }
          break;
        case STATE_READY_FOR_SENDING_PACKAGE_MOTORS: //!< indicates that the thread is ready to send new motor values to an ECB
          if (currentECBIndex < getNumberOfMediatorCollegues()) {
            if (globalData.paused)
              mediate(getMediatorCollegue(currentECBIndex), new ECBCommunicationEvent(
                  ECBCommunicationEvent::EVENT_REQUEST_SEND_MOTOR_STOP_PACKAGE));
            else
              mediate(getMediatorCollegue(currentECBIndex), new ECBCommunicationEvent(
                  ECBCommunicationEvent::EVENT_REQUEST_SEND_MOTOR_PACKAGE));
            currentCommState = STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS;
          } else {
            currentECBIndex = 0;
            currentCommState = STATE_READY_FOR_STEP_OVER_AGENTS;
          }
          break;
        case STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS: //!< awaiting package with current sensor informations of current handled ECB
          usleep(1);
          break;
        case STATE_NOT_INITIALISED:
        case STATE_STOPPED:
        default:
          break;
      }
      usleep(1);
    }
    globalData.textLog("QECBCommunicator: exiting communication loop...");
  }

  long QECBCommunicator::timeOfDayinMS() {
    struct timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec * 1000 + t.tv_usec / 1000;
  }

  void QECBCommunicator::resetSyncTimer() {
    realtimeoffset = timeOfDayinMS();
  }

  void QECBCommunicator::loopCallback() {
    /************************** Time Syncronisation ***********************/
    // Time syncronisation of real time and simulations time (not if on capture mode, or paused)
    if (!globalData.paused) {
      long currentTime = timeOfDayinMS();
      int benchmarkSteps = 10;
      long elapsed = currentTime - realtimeoffset;
      if (globalData.benchmarkMode) {
        globalData.textLog("Elapsed time: " + QString::number(elapsed) + "ms");
        if (globalData.benchmarkMode && (globalData.controlStep % benchmarkSteps == 0)) {
          globalData.textLog("Benchmark: " + QString::number(((double) benchmarkSteps) / ((double) (currentTime
              - lastBenchmarkTime)) * 1000.0) + " cycles/s");
          lastBenchmarkTime = currentTime;
        }
      }
      // difference between actual passed time and desired cycletime
      long diff = globalData.cycleTime - elapsed;
      if (diff > 10000 || diff < -10000) // check for overflow or other weird things
        resetSyncTimer();
      else {
        if (diff > 4) { // if less the 3 milliseconds we don't call usleep since it needs time
          usleep((diff - 2) * 1000);
        } else if (diff < 0) {
          globalData.textLog("Time leak of " + QString::number(abs(diff)) + "ms detected", QGlobalData::LOG_VERBOSE);
        }
      }
    } /*else {
     while (globalData.paused) {
     usleep(1000);
     }
     }*/
    resetSyncTimer();
  }

  void QECBCommunicator::dispatchPackageCommand(ECBCommunicationEvent* event) {
    globalData.textLog("dispatching received command package");
    ECBCommunicationData commData = event->commPackage;
    switch (commData.command) {
      // In both cases the event is mediated to the ECB.
      // The ECB decides itself if it must be initialised (then COMMAND_DIMENSION
      // is returned).
      case COMMAND_DIMENSION: // 00000010 Dimension data: number of sensors/motors
        timer.stop(); // stop TransmitTimer
        event->type = ECBCommunicationEvent::EVENT_PACKAGE_DIMENSION_RECEIVED;
        mediate(currentECBIndex, event);
        currentECBIndex++;
        currentCommState = STATE_READY_FOR_SENDING_PACKAGE_MOTORS;
        break;
      case COMMAND_SENSORS: // 00000011 Sensor data values
        timer.stop(); // stop TransmitTimer
        event->type = ECBCommunicationEvent::EVENT_PACKAGE_SENSORS_RECEIVED;
        mediate(currentECBIndex, event);
        currentECBIndex++;
        currentCommState = STATE_READY_FOR_SENDING_PACKAGE_MOTORS;
        break;
      case COMMAND_MESSAGE: // 00001001 Message data.
      {
        QString message("Message received: ");
        for (int i = 0; i < commData.dataLength; i++)
          message += (char) commData.data[i];
        globalData.textLog(message);
        delete event;
        break;
      }
      case COMMAND_PONG: // COMMAND_ommunication Test between PCOMMAND_ and ATmega
        timer.stop(); // stop TransmitTimer
        globalData.textLog("PONG.");
        delete event;
        break;
      case COMMAND_MOTOR_CURRENT_LIMIT: // ack for set of motor current limit
        timer.stop(); // stop TransmitTimer
        globalData.textLog("ack: motor current limit set!");
        delete event;
        break;

        // all following commands are not kind of interest, they should never occur.
      case COMMAND_DUMMY: // 00000000 Dummy command, do not use this!
      case COMMAND_RESET: // 00000001 Reset command
      case COMMAND_MOTORS: // 00000100 Motor data values
      case COMMAND_BEEP: // 00001000 Make a beep
      case COMMAND_TESTI2C: // 00001010 Test i2c communication.
      case COMMAND_PING: // COMMAND_ommunication Test between PCOMMAND_ and ATmega
      case COMMAND_MOTOR_START: // start motors of the md23-motorboard
      case COMMAND_MOTOR_STOP: // stop motors of the md23-motorboard
      case COMMAND_OSCI: // define the oscillation of the SphericalRobots-Weights
      default:
        globalData.textLog("Unidentified command in package or command which is not known how to be handled: "
            + QString(commData.command));
        delete event;
        break;
    }
  }

  void QECBCommunicator::sl_messageReceived(struct _communicationMessage msg) {
//    // if in paused mode, ignore package
//    if (globalData.paused) {
//      globalData.textLog("Package received, but not handled while in pause mode.");
//      return;
//    }
    //globalData.textLog("QECBCommunicator::sl_messageReceived(): "+printBuffer(msg.data));
    if (msg.data[0] != (char) 0x01)
      globalData.textLog("Message from MessageDispatchServer received, but not correct MsgGroup (" + toHexNumberString(
          msg.data[0], 1) + " instead of 0x01)", globalData.LOG_DEBUG);
    ECBCommunicationEvent* event = new ECBCommunicationEvent();
    event->commPackage.command = (uint8) msg.data[1];
    event->commPackage.dataLength = msg.data.size()-2;
    for (int i = 0; i < event->commPackage.dataLength; i++) { // copy data
      event->commPackage.data[i] = msg.data[i + 2];
    }
    // stopTimer is handled in dispatchPackageCommand(event)
    dispatchPackageCommand(event);
    // in general the data is forwarded to the currently registered ECB
    // The ECB instances are alternating.
  }

  void QECBCommunicator::sl_TimerExpired() {
    timer.stop();
    switch (currentCommState) {
      case STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS:
        mediate(currentECBIndex, new ECBCommunicationEvent(ECBCommunicationEvent::EVENT_COMMUNICATION_ANSWER_TIMEOUT));
        // got to next ECB
        currentECBIndex++;
        currentCommState = STATE_READY_FOR_SENDING_PACKAGE_MOTORS;
        break;
      default:
        globalData.textLog("WARNING: timer expired but state not handled (" + QString::number(currentCommState) + ")");
        break;
    }

    globalData.textLog("serialReadTimeout (" + QString::number(globalData.serialReadTimeout) + ") occured!");

  }

  QString QECBCommunicator::printBuffer(const QByteArray buffer) {
    QString hex;
    QString line;

    for (int i = 0; i < buffer.length(); i++) {
      line.append(QString::number((buffer[i] >> 4) & 0x0F, 16).toUpper());
      line.append(QString::number((buffer[i] >> 0) & 0x0F, 16).toUpper());
      line.append(" ");
    }
    return line;
  }

  QString QECBCommunicator::printBuffer(const uint8 buffer) {
    return printBuffer(QString::number(buffer).toLocal8Bit());
  }

  void QECBCommunicator::sl_quitServer() {
    emit sig_quitServer();
    globalData.textLog("QUIT from Server!", QGlobalData::LOG_ERROR);
  }

  QString QECBCommunicator::toHexNumberString(uint64 value, uint numberDigits) {
    QString hex;
    for (int i = numberDigits; i > 0; i--) {
      hex.append(QString::number((value >> (i * 4 - 4)) & 0x0F, 16).toUpper());
    }
    return hex;
  }

} // namespace lpzrobots
