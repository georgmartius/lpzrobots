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
 *   Revision 1.17  2009-08-11 19:31:53  guettler
 *   typo fix
 *
 *   Revision 1.16  2009/08/11 19:30:23  guettler
 *   use CThread pause functionality
 *
 *   Revision 1.15  2009/08/11 19:28:07  guettler
 *   stop/pause threads while paused
 *
 *   Revision 1.14  2009/08/11 19:00:56  guettler
 *   code cleanup
 *
 *   Revision 1.13  2009/08/11 18:50:33  guettler
 *   stopTimer optimised, added discoverXBeeHardwareVersionTimeout to GlobalData
 *
 *   Revision 1.12  2009/08/11 18:26:47  guettler
 *   BUGFIX: stopTimer if SerialPortThread calls back
 *
 *   Revision 1.11  2009/08/11 17:00:37  guettler
 *   fixed 16bit address gather
 *
 *   Revision 1.10  2009/08/11 16:21:31  guettler
 *   fixed typo
 *
 *   Revision 1.9  2009/08/11 15:49:05  guettler
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
 *   Revision 1.8  2009/03/25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.7  2008/07/30 06:03:47  robot1
 *   the new directory GUIs is added
 *
 *   Revision 1.6  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.5  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.4  2008/04/08 10:11:03  guettler
 *   alpha testing
 *
 *   Revision 1.3  2008/04/08 09:12:34  martius
 *   *** empty log message ***
 *
 *   Revision 1.2  2008/04/08 09:09:09  martius
 *   fixed globaldata to pointer in classes
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#include "ecbcommunicator.h"

#include <selforg/agent.h>

#include <sys/time.h>
#include <assert.h>
#include "ecbagent.h"
#include "ecb.h"
#include <unistd.h>
#include <sstream>

// include command definition: CSTOP, CSENS, ...
#include "commanddefs.h"

using namespace std;

namespace lpzrobots {

  unsigned int currentECBIndex;
  ECBCommunicator::ECBCommunicator(GlobalData& globalData) :
    CThread("ECBCommunicator", globalData.debug), globalData(&globalData), transmitBufferCheckSum(0), serialPortThread(
        0), ecbTransmitModeType(Undefined), currentCommState(STATE_NOT_INITIALISED), currentECBIndex(0), currentTime(
        timeOfDayinMS()), answerTime(0), threadsStoppedWhilePaused(false) {
    if (this->globalData->debug)
      std::cout << "New ECBCommunicator created." << std::endl;
    
    realtimeoffset = timeOfDayinMS();
    lastBenchmarkTime = realtimeoffset;
    this->globalData->comm = this;
    
    timerThread.addCallbackable(this, TimerThread::TIMER_EXPIRED);
  }

  ECBCommunicator::~ECBCommunicator() {
    delete serialPortThread;
  }
  ;

  void ECBCommunicator::setConfig(GlobalData& globalData) {
    CThread::setConfig(globalData.debug);
    this->globalData = &globalData;
    realtimeoffset = timeOfDayinMS();
    lastBenchmarkTime = realtimeoffset;
    this->globalData->comm = this;
    if (this->globalData->debug)
      std::cout << "config of ECBCommunicator updated." << std::endl;
  }

  /*void ECBCommunicator::flushInputBuffer ( int time )
   {
   if ( globalData->debug )
   std::cout << "ECBCommunicator: flushInputBuffer!" << std::endl;
   if ( time==-1 )
   time = globalData->cycleTime/2;
   CSerialThread::flushInputBuffer ( time );
   }*/

  bool ECBCommunicator::loop() {
    assert ( globalData->comm==this );
    if (globalData->pause) {
      serialPortThread->pause();
      timerThread.stopTimer();
      threadsStoppedWhilePaused = true;
      usleep(1000);
      return true;
    } else if (threadsStoppedWhilePaused==true)
    {
      threadsStoppedWhilePaused = false;
      serialPortThread->resume();
      timerThread.restartTimer();
    }

    switch (currentCommState) {
      case STATE_READY_FOR_STEP_OVER_AGENTS:
        globalData->simStep++;
        if (globalData->debug) {
          std::cout << "ECBCommunicator: loop! simStep=" << globalData->simStep << std::endl;
        }
        /// With this for loop all agents perform a controller step
        if (!globalData->testMode) {
          // sorgt dafür, dass der Zeittakt eingehalten wird:
          // Berechnung zu schnell -> warte,
          // Berechnung zu langsam -> Ausgabe, dass time leak stattfindet
          loopCallback();
          FOREACH ( AgentList,globalData->agents,a ) {
            ((ECBAgent*) (*a))->step(globalData->noise, globalData->simStep);
          }
        } else {
          return this->testModeCallback();
        }
        if (globalData->debug)
          cout << "ECBCommunicator: AgentStep finished." << endl;
        currentCommState = STATE_READY_FOR_SENDING_PACKAGE_MOTORS;
        break;
      case STATE_READY_FOR_SENDING_PACKAGE_MOTORS: //!< indicates that the thread is ready to send new motor values to an ECB
        if (currentECBIndex < getNumberOfMediatorCollegues()) {
          mediate(getMediatorCollegue(currentECBIndex), new ECBCommunicationEvent(
              ECBCommunicationEvent::EVENT_REQUEST_SEND_MOTOR_PACKAGE));
          currentCommState = STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS;
        } else {
          currentECBIndex = 0;
          currentCommState = STATE_READY_FOR_STEP_OVER_AGENTS;
        }
        break;
      case STATE_DISCOVER_XBEE_HW_VERSION: //!< awaiting package info with hw version (XBee series1, XBee series2, cable)
      case STATE_DISCOVER_XBEE_NODES: //!< awaiting package info which XBee nodes are in range
      case STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS: //!< awaiting package with current sensor informations of current handled ECB
        usleep(1);
        return true;
        break;
      case STATE_NOT_INITIALISED: //!< port not opened, SerialPortThread is not running
      case STATE_STOPPED:
      default:
        return false;
        break;
    }
    return true;
  }

  long ECBCommunicator::timeOfDayinMS() {
    struct timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec * 1000 + t.tv_usec / 1000;
  }

  void ECBCommunicator::resetSyncTimer() {
    realtimeoffset = timeOfDayinMS();
  }

  void ECBCommunicator::loopCallback() {
    /************************** Time Syncronisation ***********************/
    // Time syncronisation of real time and simulations time (not if on capture mode, or pause)
    if (!globalData->pause) {
      long currentTime = timeOfDayinMS();
      int benchmarkSteps = 10;
      long elapsed = currentTime - realtimeoffset;
      if (globalData->benchmarkMode || globalData->debug) {
        std::cout << "Elapsed time: " << elapsed << "ms" << std::endl;
        if (globalData->benchmarkMode && (globalData->simStep % benchmarkSteps == 0)) {
          std::cout << "Benchmark: " << (((double) benchmarkSteps) / ((double) (currentTime - lastBenchmarkTime))
              * 1000.0) << " cycles/s" << std::endl;
          lastBenchmarkTime = currentTime;
        }
      }
      // difference between actual passed time and desired cycletime
      long diff = globalData->cycleTime - elapsed;
      if (diff > 10000 || diff < -10000) // check for overflow or other weird things
        resetSyncTimer();
      else {
        if (diff > 4) { // if less the 3 milliseconds we don't call usleep since it needs time
          usleep((diff - 2) * 1000);
        } else if (diff < 0) {
          if (globalData->verbose) {
            printf("Time leak of %li ms detected\n", abs(diff));
          }
        }
      }
    } else {
      while (globalData->pause) {
        usleep(1000);
      }
    }
    resetSyncTimer();
  }

  void ECBCommunicator::push_Frame(uint8 c) {
    transmitBuffer.push_back(c);
  }
  void ECBCommunicator::push_FrameEscaped(uint8 c) {
    // Von der Pruefsumme ausgeschlossen sind das Startsymbol und das Laengenfeld,
    // deswegen erst ab dem 3. Zeichen die Pruefsumme bilden!
    if (2 < transmitBuffer.size())
      transmitBufferCheckSum += c;

    // Ist fuer dieses Zeichen eine Ausnahmebehandlung notwendig?
    if (c == 0x7E || c == 0x7D || c == 0x13 || c == 0x11) {
      transmitBuffer.push_back(0x7D);
      transmitBuffer.push_back((uint8) (c ^ 0x20));
    } else {
      transmitBuffer.push_back(c);
    }
  }
  bool ECBCommunicator::transmit() {
    // Schreibe die Pruefsumme
    push_FrameEscaped((uint8) (255 - transmitBufferCheckSum % 256));

    if (globalData->debug) {
      cout << "ECBCommunicator: transmitting now: ";
      printBuffer(transmitBuffer);
    }
    // Gebe die Nachricht ueber den Seriellen-Port aus
    bool ret = serialPortThread->writePort(transmitBuffer);
    if (!ret && debug)
      cout << "write to port failed!" << endl;

    // Loesche nun den Uebertragungs-Puffer und Reinitialisiere die benoetigten Variablen
    transmitBufferCheckSum = 0;
    transmitBuffer.clear();
    timerThread.restartTimer();
    currentTime = timeOfDayinMS();
    return ret;
  }

  bool ECBCommunicator::initialise() {
    if (globalData->debug)
      std::cout << "ECBCommunicator: (external) initialising..." << std::endl;
    serialPortThread = new SerialPortThread(globalData->portName, globalData->baudrate, globalData->debug);
    serialPortThread->addCallbackable(this, SerialPortThread::NEW_DATA_RECEIVED);
    serialPortThread->addCallbackable(this, SerialPortThread::DATA_CHECKSUM_ERROR);
    serialPortThread->openPort();
    if (serialPortThread->isOpened()) {
      currentCommState = STATE_DISCOVER_XBEE_HW_VERSION;
      // just wait a little bit; Xbee is responding immediately
      timerThread.setTimer(globalData->discoverXBeeHardwareVersionTimeout, false);
      // identify hardware revision
      send_XBeeATHV();
      // after this the main loop starts
    } else
      return false;
    return true;
  }

  void ECBCommunicator::send_XBeeATHV() {
    if (globalData->debug)
      cout << "send_XBeeATHV" << endl;
    currentCommState = STATE_DISCOVER_XBEE_HW_VERSION;
    // Sende Commando zum Abfragen der Hardware-Version eines XBee-Modules
    // Beachte, ein Kabel-Modul wird auch antworten!!!
    // Wird ein XBee-Modul angesprochen, so wird dies in der Regel antworten
    // und damit die Anbindung erneut Überschreiben!

    push_Frame(0x7E); // Startsymbol
    push_FrameEscaped(0x00); // Length MSB
    push_FrameEscaped(0x04); // Length LSB
    push_FrameEscaped(0x08); // API_ID - AT_Command
    push_FrameEscaped((uint8) 'R'); // Frame_ID - 'R' -> erwarte Antwort
    push_FrameEscaped((uint8) 'H'); // AT-Command QByte 1
    push_FrameEscaped((uint8) 'V'); // AT-Command QByte 2
    transmit();
  }

  void ECBCommunicator::send_XBeeATND() {
    if (globalData->debug)
      cout << "send_XBeeATND" << endl;
    currentCommState = STATE_DISCOVER_XBEE_NODES;
    push_Frame(0x7E); // Startsymbol
    push_FrameEscaped(0x00); // Length MSB
    push_FrameEscaped(0x04); // Length LSB
    push_FrameEscaped(0x08); // API AT-Command
    push_FrameEscaped(0x52); // Frame ('R')
    push_FrameEscaped(0x4E); // AT-Command 'N'
    push_FrameEscaped(0x44); // AT-Command 'D'
    transmit();
  }

  void ECBCommunicator::send_CommandPackage(CommunicationData commandPackage, ECB* sourceECB) {
    if (globalData->debug)
      cout << "ECBCommunicator: Sending command package (" << commandPackage.command << ") to "
          << sourceECB->getDNSName() << "" << endl;
    uint16 ECB_XBeeAddress16 = sourceECB->getNetworkAddress();
    uint64 ECB_XBeeAddress64 = sourceECB->getSerialNumber();
    // remember that the internal state of ECBCommunicator must be handled outside this function!
    switch (ecbTransmitModeType) {
      case Cable: {
        uint16 length = 1 + 2 + commandPackage.dataLength;

        push_Frame(0x7E); //  1: Startsymbol
        push_FrameEscaped((uint8) (length >> 8)); //  2: Length MSB
        push_FrameEscaped((uint8) (length >> 0)); //  3: Length LSB
        push_FrameEscaped(0x20); //  4: API_ID - Cable
        break;
      }
      case XBee: {
        uint16 length = 5 + 2 + commandPackage.dataLength;

        push_Frame(0x7E); //  1: Startsymbol
        push_FrameEscaped((uint8) (length >> 8)); //  2: Length MSB
        push_FrameEscaped((uint8) (length >> 0)); //  3: Length LSB
        push_FrameEscaped(0x01); //  4: API-ID
        push_FrameEscaped(0x01); //  5: Frame-ID - immer 0 -> kein ResponsePaket vom XBee
        push_FrameEscaped((uint8) (ECB_XBeeAddress16 >> 8)); //  6: DestinationAddress MSB
        push_FrameEscaped((uint8) (ECB_XBeeAddress16 >> 0)); //  7: DestinationAddress LSB
        push_FrameEscaped(0x00); //  8: Options - immer 1  -> kein ResponsePaket vom XBee
        break;
      }
      case XBeeS2: {
        uint16 length = 14 + 2 + commandPackage.dataLength;

        push_Frame(0x7E); //  1: Startsymbol
        push_FrameEscaped((uint8) (length >> 8)); //  2: Length MSB
        push_FrameEscaped((uint8) (length >> 0)); //  3: Length LSB
        push_FrameEscaped(0x10); //  4: API_ID - TransmitRequest XBeeSerie2
        push_FrameEscaped(0x00); //  5: Frame-ID - immer 0 -> kein ResponsePaket vom XBee
        push_FrameEscaped((uint8) (ECB_XBeeAddress64 >> 7 * 8)); //  6: 64_Bit_Destination_Network_Address
        push_FrameEscaped((uint8) (ECB_XBeeAddress64 >> 6 * 8)); //  7:
        push_FrameEscaped((uint8) (ECB_XBeeAddress64 >> 5 * 8)); //  8:
        push_FrameEscaped((uint8) (ECB_XBeeAddress64 >> 4 * 8)); //  9:
        push_FrameEscaped((uint8) (ECB_XBeeAddress64 >> 3 * 8)); // 10:
        push_FrameEscaped((uint8) (ECB_XBeeAddress64 >> 2 * 8)); // 11:
        push_FrameEscaped((uint8) (ECB_XBeeAddress64 >> 1 * 8)); // 12:
        push_FrameEscaped((uint8) (ECB_XBeeAddress64 >> 0 * 8)); // 13:
        push_FrameEscaped((uint8) (ECB_XBeeAddress16 >> 1 * 8)); // 14: 16_Bit_Destination_Network_Address
        push_FrameEscaped((uint8) (ECB_XBeeAddress16 >> 0 * 8)); // 15:
        push_FrameEscaped(0x00); // 16: Broadcast-Range
        push_FrameEscaped(0x01); // 17: OptionsByte - immer 1  -> kein ResponsePaket vom XBee
        break;
      }
      default:
        cerr << "ERROR: Unknown transmit type!" << endl;
        return;
        break;
    }//end switch
    // AnwenderDaten
    push_FrameEscaped(Application_MessageGroupCode); // MessageGroup_ID
    push_FrameEscaped(commandPackage.command); // command
    for (int index = 0; index < commandPackage.dataLength; index++)
      push_FrameEscaped(commandPackage.data[index]); // data...
    transmit();
  }

  void ECBCommunicator::dispatch_XbeeCommandResponse(std::vector<uint8> receiveBuffer) {
    uint8 msgLength = (receiveBuffer[1] & 0xFF) << 8 | (receiveBuffer[2] & 0xFF);

    std::string Command;
    Command.push_back((char) receiveBuffer[5]);
    Command.push_back((char) receiveBuffer[6]);

    if (Command.compare("HV") == 0) {
      uint16 HardwareVersionNumber = 0;

      HardwareVersionNumber += ((uint16) receiveBuffer[8] & 0xFF) << 1 * 8;
      HardwareVersionNumber += ((uint16) receiveBuffer[9] & 0xFF) << 0 * 8;

      switch (HardwareVersionNumber) { // TODO: extend for each XBee/ZigBee device: hardware version number
        case 0x0000:
          ecbTransmitModeType = Cable;
          cout << "Running in cable mode." << endl;
          break;
        case 0x180B:
          ecbTransmitModeType = XBee;
          cout << "Running in XBee series 1 mode." << endl;
          break;
        case 0x1942:
          ecbTransmitModeType = XBeeS2;
          cout << "Running in XBee series 2 mode." << endl;
          break;
        default:
          ecbTransmitModeType = Undefined;
          cout << "Running in unknown mode." << endl;
          break;
      }
      if (ecbTransmitModeType != Undefined) {
        timerThread.stopTimer(); // stop TransmitTimer
        if (ecbTransmitModeType != Cable) { // send discover on XBee nodes
          // start timer: wait for answer from all reachabled nodes
          currentCommState = STATE_DISCOVER_XBEE_NODES;
          timerThread.setTimer(globalData->discoverNodesTimeout, false);
          send_XBeeATND();
        } else {
          // in cable mode we don't need to discover the xbee nodes
          // there should be only one ecb, register this ECB at the Mediator
          if (globalData->agents.size() == 1) {
            ECBRobot* robot = globalData->agents.front()->getRobot();
            if (robot->getECBlist().size() == 1) {
              currentCommState = STATE_READY_FOR_STEP_OVER_AGENTS;
            } else {
              cerr << "ERROR: One ECBAgent and ECBRobot defined, but number of ECBs != 1 in cable mode! (is: "
                  << robot->getECBlist().size() << ")" << endl;
              currentCommState = STATE_STOPPED;
            }
          } else {
            cerr << "ERROR: Number of ECBAgents and ECBRobots != 1 in cable mode! (is: " << globalData->agents.size()
                << ")" << endl;
            currentCommState = STATE_STOPPED;
          }
        }
      }
      return;
    }

    if (Command.compare("ND") == 0) // NodeIdentifier-Response
    {
      if ((int) receiveBuffer[7] != 0) // Status OK?
      {
        if (globalData->debug)
          cout << "Error occured while identifing nodes." << endl;
        return;
      }

      std::string nodeId;
      uint16 ECB_XBeeAddress16_tmp = 0;
      uint64 ECB_XBeeAddress64_tmp = 0;

      // Das XBee sendet als antwort auch ein Paket mit Länge 5,
      // jedoch ohne nützliche Informationen
      if (msgLength <= 5)
        return;

      ECB_XBeeAddress16_tmp += ((uint8) receiveBuffer[8] & 0xFF) << 1 * 8;
      ECB_XBeeAddress16_tmp += ((uint8) receiveBuffer[9] & 0xFF) << 0 * 8;

      ECB_XBeeAddress64_tmp += ((uint64) receiveBuffer[10] & 0xFF) << 7 * 8;
      ECB_XBeeAddress64_tmp += ((uint64) receiveBuffer[11] & 0xFF) << 6 * 8;
      ECB_XBeeAddress64_tmp += ((uint64) receiveBuffer[12] & 0xFF) << 5 * 8;
      ECB_XBeeAddress64_tmp += ((uint64) receiveBuffer[13] & 0xFF) << 4 * 8;
      ECB_XBeeAddress64_tmp += ((uint64) receiveBuffer[14] & 0xFF) << 3 * 8;
      ECB_XBeeAddress64_tmp += ((uint64) receiveBuffer[15] & 0xFF) << 2 * 8;
      ECB_XBeeAddress64_tmp += ((uint64) receiveBuffer[16] & 0xFF) << 1 * 8;
      ECB_XBeeAddress64_tmp += ((uint64) receiveBuffer[17] & 0xFF) << 0 * 8;

      // Lese den NodeIdentifier-String aus.
      switch (ecbTransmitModeType) {
        case Cable:
          break;
        case XBee: {
          for (int i = 0; i < msgLength - 17; i++)
            nodeId.push_back((char) receiveBuffer[19 + i]);
          break;
        }
        case XBeeS2: {
          for (int i = 18; i < msgLength - 6; i++)
            nodeId.push_back((char) receiveBuffer[i]);
          break;
        }
        default:
          assert(false);
          break;
      }//end switch

      if (globalData->debug)
        cout << "XBee/ECB with dnsName found: \"" << nodeId << "\"";
      // get ECBs and set address
      // ..and check, if all addresses are known
      bool found = false;
      bool allAddressesSet = true;
      for (unsigned int index = 0; index < getNumberOfMediatorCollegues(); index++) {
        ECB* ecb = static_cast<ECB*> (getMediatorCollegue(index));
        if (nodeId.compare(ecb->getDNSName()) == 0) {
          if (globalData->debug)
            cout << " match for ECBRobot " << endl;
          ecb->setAddresses(ECB_XBeeAddress16_tmp, ECB_XBeeAddress64_tmp);
          found = true;
          break;
        } else if (!ecb->isAddressesSet())
          allAddressesSet = false;
      }
      // info: If ECB/XBee was not found, just ignore this XBee!
      if (!found && globalData->debug)
        cout << " --- but no Robot found for it (ignoring)" << endl;
      if (allAddressesSet && currentCommState == STATE_DISCOVER_XBEE_NODES) {
        timerThread.stopTimer(); // stop TransmitTimer
        timerThread.setTimer(globalData->serialReadTimeout, false);
        currentCommState = STATE_READY_FOR_STEP_OVER_AGENTS;
      }
    }
  }

  void ECBCommunicator::dispatchPackageCommand(ECBCommunicationEvent* event) {
    if (globalData->debug)
      cout << "dispatching received command package" << endl;
    CommunicationData commData = event->commPackage;
    switch (commData.command) {
      // In both cases the event is mediated to the ECB.
      // The ECB decides itself if it must be initialised (then COMMAND_DIMENSION
      // is returned).
      case COMMAND_DIMENSION: // 00000010 Dimension data: number of sensors/motors
        timerThread.stopTimer(); // stop TransmitTimer
        event->type = ECBCommunicationEvent::EVENT_PACKAGE_DIMENSION_RECEIVED;
        mediate(currentECBIndex, event);
        currentECBIndex++;
        currentCommState = STATE_READY_FOR_SENDING_PACKAGE_MOTORS;
        break;
      case COMMAND_SENSORS: // 00000011 Sensor data values
        timerThread.stopTimer(); // stop TransmitTimer
        event->type = ECBCommunicationEvent::EVENT_PACKAGE_SENSORS_RECEIVED;
        mediate(currentECBIndex, event);
        currentECBIndex++;
        currentCommState = STATE_READY_FOR_SENDING_PACKAGE_MOTORS;
        break;
      case COMMAND_MESSAGE: // 00001001 Message data.
        cout << "Message received: ";
        for (int i = 0; i < commData.dataLength; i++)
          cout << (char) commData.data[i];
        cout << endl;
        delete event;
        break;
      case COMMAND_PONG: // COMMAND_ommunication Test between PCOMMAND_ and ATmega
        // timerThread.stopTimer(); // stop TransmitTimer?
        cout << "PONG." << endl;
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
        cerr << "Unidentified command in package or command which is not known how to be handled: " << commData.command
            << endl;
        delete event;
        break;
    }
  }

  void ECBCommunicator::newDataReceived(std::vector<uint8> receiveBuffer) {
    if (globalData->debug)
      printBuffer(receiveBuffer);
    // Eine Nachricht vom Microcontroller wurde empfangen
    // --------------------------------------------------
    //  0 uint8 StartDelimiter;
    //  1 uint8 Length_MSB;
    //  2 uint8 Length_LSB;
    //  3 uint8 API_ID;
    //  4 ...

    uint16 msgLength = ((uint16) receiveBuffer[1]) << 8 | ((uint16) receiveBuffer[2] & 0xFF);
    int msgApi_Id = ((uint8) receiveBuffer[3] & 0xFF);

    switch (msgApi_Id) {
      case API_XBee_AT_Command_Response:
        // stopTimer is handled in dispatch_XbeeCommandResponse(receiveBuffer)
        dispatch_XbeeCommandResponse(receiveBuffer);
        break;
      case API_Cable_TransmitReceive: {
        // +----+----+-----+-----+----------+---------+------+----+
        // | 7E |  Length  | API | msgGroup | command | data | CS |
        // +----+----+-----+-----+----------+---------+------+----+
        if (receiveBuffer[4] != Application_MessageGroupCode) // msgGroup
          return; // message is from bootloader, ignore for application
        ECBCommunicationEvent* event = new ECBCommunicationEvent();
        event->commPackage.command = (uint8) receiveBuffer[5];
        event->commPackage.dataLength = (uint8) (msgLength - 3);
        for (int i = 0; i < event->commPackage.dataLength; i++) { // copy data
          event->commPackage.data[i] = receiveBuffer[6 + i];
        }
        // stopTimer is handled in dispatchPackageCommand(event)
        dispatchPackageCommand(event);
      }
        break;
      case API_XBee_Receive_Packet_16Bit: {
        // +----+----+-----+-----+--------+---------+------+---------+----------+---------+------+----+
        // | 7E |  Length  | API | SourceAddr_16Bit | RSSI | Options | msgGroup | command | data | CS |
        // +----+----+-----+-----+--------+---------+------+---------+----------+---------+------+----+
        if (receiveBuffer[8] != Application_MessageGroupCode) // msgGroup
          return; // message is from bootloader, ignore for application
        ECBCommunicationEvent* event = new ECBCommunicationEvent();
        event->commPackage.command = (uint8) receiveBuffer[9];
        event->commPackage.dataLength = (uint8) (msgLength - 7);
        for (int i = 0; i < event->commPackage.dataLength; i++) { // copy data
          event->commPackage.data[i] = receiveBuffer[10 + i];
        }
        // stopTimer is handled in dispatchPackageCommand(event)
        dispatchPackageCommand(event);
      }
        break;
      case API_XBeeS2_ZigBee_Receive_Packet: {
        // +----+----+-----+-----+---+---+---+---+---+---+---+---+-------+-------+---------+----------+---------+------+----+
        // | 7E |  Length  | API |         64Bit-Address         | 16Bit_Address | Options | msgGroup | command | data | CS |
        // +----+----+-----+-----+---+---+---+---+---+---+---+---+-------+-------+---------+----------+---------+------+----+
        if (receiveBuffer[15] != Application_MessageGroupCode) // msgGroup
          return; // message is from bootloader, ignore for application
        ECBCommunicationEvent* event = new ECBCommunicationEvent();
        event->commPackage.command = (uint8) receiveBuffer[16];
        event->commPackage.dataLength = (uint8) (msgLength - 14);
        for (int i = 0; i < event->commPackage.dataLength; i++) { // copy data
          event->commPackage.data[i] = receiveBuffer[17 + i];
        }
        // stopTimer is handled in dispatchPackageCommand(event)
        dispatchPackageCommand(event);
      }
        break;
      default: {
        if (globalData->debug)
          printBuffer(receiveBuffer);
        // don't stop timer!
        return;
      }
    } //end switch api
    // in general the data is forwarded to the currently registered ECB
    // The ECB instances are alternating.
  }

  void ECBCommunicator::doOnCallBack(BackCaller* source, BackCaller::CallbackableType type /* =
   SerialPortThread::NEW_DATA_RECEIVED*/) {
    switch (type) {
      case SerialPortThread::NEW_DATA_RECEIVED:
        // stopTimer is handled in newDataReceived (determined by received package)
        if (globalData->debug)
          cout << "newData received: answer took " << (timeOfDayinMS() - currentTime) << "ms, package: ";
        newDataReceived(serialPortThread->getData());
        break;
      case SerialPortThread::DATA_CHECKSUM_ERROR:
        timerThread.stopTimer(); // stop TransmitTimer
      case TimerThread::TIMER_EXPIRED: {
        switch (currentCommState) {
          case STATE_NOT_INITIALISED:
            cerr << "ERROR: should never be reached!" << endl;
            break;
          case STATE_DISCOVER_XBEE_HW_VERSION:
            cerr << "ERROR: XBee not responding or no cable connected! retry..." << endl;
            // just retry after 1 sec
            //sleep(1);
            send_XBeeATHV();
            break;
          case STATE_DISCOVER_XBEE_NODES:
            cerr << "ERROR: XBee not responding (no nodes in range, check Xbee hardware version)! retry..." << endl;
            // just retry after 1 sec
            // sleep(1);
            send_XBeeATND();
            break;
          case STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS:
            mediate(currentECBIndex, new ECBCommunicationEvent(
                ECBCommunicationEvent::EVENT_COMMUNICATION_ANSWER_TIMEOUT));
            // got to next ECB
            currentECBIndex++;
            currentCommState = STATE_READY_FOR_SENDING_PACKAGE_MOTORS;
            break;
          default:
            cerr << "WARNING: timer expired but state not handled (" << currentCommState << ")" << endl;
            break;
        }
      }
        break;
      default: // e.g. SerialPortThread::DEFAULT_CALLBACKABLE_TYPE
        break;
    }
  }

  void ECBCommunicator::printBuffer(std::vector<uint8>& buffer) {
    //std::stringstream line;


    for (unsigned int i = 0; i < buffer.size(); i++) {
      cout << std::hex << ((buffer[i] >> 4) & 0x0F);
      cout << std::hex << ((buffer[i] >> 0) & 0x0F);
      cout << " ";
    }
    cout << std::dec << std::endl;

    // TODO: append to logfile
    //cout << line;
  }

  void ECBCommunicator::mediatorInformed(MediatorCollegue* source, MediatorEvent* event) {
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

} // namespace lpzrobots
