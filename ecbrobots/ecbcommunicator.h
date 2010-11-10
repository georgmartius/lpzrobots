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
 *   Revision 1.8  2010-11-10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.7  2009/08/11 19:28:07  guettler
 *   stop/paused threads while paused
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
 *   - QGlobalData, ECBCommunicator is now configurable
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
#ifndef __ECBCOMMUNICATOR_H
#define __ECBCOMMUNICATOR_H

//#include <vector>
#include "CThread.h"
#include "SerialPortThread.h"
#include "TimerThread.h"
#include <selforg/callbackable.h>
#include <selforg/configurable.h>
#include <selforg/mediator.h>
#include "constants.h"
#include <selforg/inspectable.h>

#include "QGlobalData.h"

namespace lpzrobots
{

  class ECB;

  /**
   * TODO: überarbeiten
   This class is the robot and the communicator in one.
   This might be a bit confusing, because it contains also the agent and so on.
   The main flow is as follows:
   loop:
   over serial we send motor values to the Xbees
   we wait for sensor values from Xbees
   then we invoke the agent
   the agent asks the robot (us) about sensors (which we saved in a local variable)
   the agent calls the controller and then sets the motor values with setMotors (also in this class)
   endloop
   */

  typedef struct
  {
      uint8 command; // command byte
      uint8 dataLength; // data length of data pointer or array
      uint8 data[254]; // data byte pointer or array
  } CommunicationData;



  class ECBCommunicationEvent : public MediatorEvent, public Inspectable
  {
    public:
      enum CommunicationEventType
      {
          EVENT_PACKAGE_SENSORS_RECEIVED, //!< indicates that a package with sensor information was received: ECBCommunicator --> ECB
          EVENT_PACKAGE_DIMENSION_RECEIVED, //!< indicates that a package with dimension (and description info) was received: ECBCommunicator --> ECB
          EVENT_REQUEST_SEND_MOTOR_PACKAGE, //!< indicates that a package with new motor values (or reset command) is requested from ECBCommunicator from a ECB: ECBCommunicator --> ECB
          EVENT_COMMUNICATION_ANSWER_TIMEOUT, //!< indicates that an awaited package was not received: ECBCommunicator --> ECB
          EVENT_REQUEST_SEND_COMMAND_PACKAGE //!< indicates that the ECB has a package to send out (motor, reset, motorstop, beep,...): ECB --> ECBCommunicator
      };

      CommunicationEventType type;
      CommunicationData commPackage;

      ECBCommunicationEvent() {}
      ECBCommunicationEvent(CommunicationEventType type) : type(type) {}
      ECBCommunicationEvent(CommunicationData commPackage) : commPackage(commPackage) {}
      ECBCommunicationEvent(CommunicationEventType type, CommunicationData commPackage) : type(type), commPackage(commPackage) {}
  };

  class ECBCommunicator : public CThread, public Callbackable, public Mediator
  {

    public:

      enum CommuncationState
      {
        STATE_NOT_INITIALISED, //!< port not opened, SerialPortThread is not running
        STATE_DISCOVER_XBEE_HW_VERSION, //!< awaiting package info with hw version (XBee series1, XBee series2, cable)
        STATE_DISCOVER_XBEE_NODES, //!< awaiting package info which XBee nodes are in range
        STATE_READY_FOR_STEP_OVER_AGENTS, //!< indicates that all ECBs are updated and the loop over the agents can begin
        STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS, //!< awaiting package with current sensor informations of current handled ECB
        STATE_READY_FOR_SENDING_PACKAGE_MOTORS, //!< indicates that the thread is ready to send new motor values to an ECB
        STATE_STOPPED //!< state which indicates that all actions should be stopped, quitted and leaved. Bye bye.
      };





      ECBCommunicator(QGlobalData& globalData);

      virtual ~ECBCommunicator();

      /**
       * Flushes the input buffer (removes all packets which are in the buffer)
       * @param time the time to wait for flushing
       */
   /*   virtual void flushInputBuffer(int time = -1);*/

      /**
       * defines a test mode callback function, which is called,
       * if globalData.testMode is true
       * @return false if you would like to stop the test mode (ECBCommunicator will stop).
       */
      virtual bool testModeCallback()
      {
        return false;
      }
      ;

      /**
       * sets the config according to given globalData
       */
      virtual void setConfig(QGlobalData& globalData);

      virtual void doOnCallBack(BackCaller* source, BackCaller::CallbackableType type =
          SerialPortThread::NEW_DATA_RECEIVED);

      using CThread::stopandwait;
      using CThread::stop;
      using CThread::start;


    private:

      virtual void newDataReceived(std::vector<uint8> dataPacket);

      virtual void dispatchPackageCommand(ECBCommunicationEvent* event);

      virtual void mediatorInformed(MediatorCollegue* source, MediatorEvent* event);

      virtual void printBuffer(std::vector<uint8>& buffer);

      virtual bool initialise();

      /** This function is called by CSerialThread every step
       * @return true if loop() should be recalled, otherwise false (thread stops)
       */
      virtual bool loop();

      long realtimeoffset;
      QGlobalData* globalData;
      long lastBenchmarkTime;


      void printMsg(int xbee, int addr, uint8* data, int len)
      {
        data[len] = 0;
        fprintf(stdout, "Message from slave (%i (addr: %i)): %s\n", xbee, addr, data);
      }

      long timeOfDayinMS();

      void resetSyncTimer();

      void loopCallback();

      // serial command functions

      // basics
      void push_Frame(uint8 c);
      void push_FrameEscaped(uint8 c);
      bool transmit();

      /**
       * send discover node for all XBees/ECBs to get network address (dynamic name resolving)
       */
      void send_XBeeATND();

      /**
       * send packet for identifying hardware revision (XBee Series1, XBee Series2)
       */
      void send_XBeeATHV();

      void send_CommandPackage(CommunicationData commData, ECB* sourceECB);

      /**
       * Is called when the SerialPortThread knows that the port is opened (via doOnCallback).
       */
      void portOpened();

      /**
       * handles the response of data packets coming from the XBee (API command)
       * (called via doOnCallback)
       * @param receiveBuffer
       */
      void dispatch_XbeeCommandResponse(std::vector<uint8> receiveBuffer);

      std::vector<uint8> transmitBuffer;
      uint8 transmitBufferCheckSum;

      SerialPortThread* serialPortThread;
      TimerThread timerThread;

      transmitMode ecbTransmitModeType;

      CommuncationState currentCommState;

      unsigned int currentECBIndex;

      unsigned long currentTime;
      unsigned long answerTime;

      bool threadsStoppedWhilePaused;
  };

}

#endif
