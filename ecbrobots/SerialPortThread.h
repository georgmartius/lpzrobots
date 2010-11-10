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
 *  Diese Klasse stellt den Zugriff zu einer seriellen Schnittstelle       *
 *  bereit. Alle verfügbaren (nicht verwendeten) seriellen Verbindungen    *
 *  können abgefragt werden. Eingehende Daten werden mit Hilfe eines       *
 *  eigenständigen Threads verarbeitet.                                    *
 *  Die Klasse sendet drei Signale: ein Signal über das Öffnen des Ports,  *
 *  ein signal über Status/Fehler-Mitteilungen und ein Signal über den     *
 *  Empfang neuer Daten vom Port                                           *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2010-11-10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.3  2009/08/11 19:30:23  guettler
 *   use CThread paused functionality
 *
 *   Revision 1.2  2009/08/11 18:26:47  guettler
 *   BUGFIX: stopTimer if SerialPortThread calls back
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
 *                                                *
 *                                                                         *
 ***************************************************************************/

#ifndef SERIALPORTTHREAD_H_
#define SERIALPORTTHREAD_H_

#if defined(WIN32) || defined(_WIN32) || defined (__WIN32) || defined(__WIN32__) \
        || defined (_WIN64) || defined(__CYGWIN__) || defined(__MINGW32__)
#define IS_WINDOWS
#elif defined(__APPLE__)
#define IS_APPLE
#elif defined(unix) || defined(__unix) || defined(__unix__)
#define IS_LINUX
#else
#error This development environment does not support pthreads or windows threads
#endif

#if defined(IS_WINDOWS)
#include <windows.h>            // Standard Windows include for system defines
#include <winbase.h>            // Has WIN32 Comm library and structure DEFS
#include <stdio.h>
#include <stdlib.h>             // Standard library include
#include <string.h>
#include <conio.h>
#include <process.h>
#endif
#if defined(IS_LINUX)
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#endif

#include <string>
#include "QGlobalData.h"
#include <selforg/backcaller.h>
#include <selforg/callbackable.h>
#include "CThread.h"
#include "constants.h"
#include "TimerThread.h"

namespace lpzrobots
{



  /**
   * Entwurfsmuster: Fassade (Verbirgt die systemspezifische Implementierung der seriellen Schnittstelle.
   *                 Anfragen und Kommandos werden an dedizierte Funktionen übergeben.)
   * Besonderheiten: Betriebssystem-Unabhängige Bereitstellung der Seriellen Schnittstelle.
   *                 Akkumulierung aller verwendbaren (noch nicht in Benutzung) seriellen Ports.
   * Verwendete Klassen: BackCaller, für die Verteilung eigener Nachrichten.
   *                     CThread, damit der serielle Port nebenläufig überwacht wird
   */
  class SerialPortThread : public BackCaller, protected CThread, protected Callbackable
  {

    public:

      /**
       * Zusaetzliche Typen
       */
      static const CallbackableType NEW_DATA_RECEIVED = 654; //!< NEW_DATA_RECEIVED
      static const CallbackableType DATA_CHECKSUM_ERROR = 655; //!< DATA_CHECKSUM_ERROR

      /**
       * Constructs a new thread. The thread does not begin executing until start() is called.
       */
      SerialPortThread(std::string portName, unsigned int baudrate, bool debug = false);
      /**
       * First stopps the then destroys the thread.
       */
      virtual ~SerialPortThread();

      /**
       * The starting point for the thread.
       * The method controls the incoming Databytes and accumulates them into one packet. If the packet was
       * received correct (that means the checksum has been valid), a callBack(NEW_DATA_RECEIVED) is called.
       */
      virtual bool loop();

      /**
       * Sets the Name of the serial Port.
       * @param port std::string, new name of the port.
       */
      void setPortName(std::string port)
      {
        portName = port;
      }
      ;
      /**
       * Returns name of the serial port.
       * @return a std::string containing the serialportname.
       */
      std::string getPortName()
      {
        return portName;
      }
      ;
      /**
       * Sets the speed of the connection.
       * @param baud the new speed-value. Notice that only standard values will make the port working.
       */
      void setBaudrate(int baud)
      {
        baudrate = baud;
      }
      ;
      /**
       * Returns the speed of the connection.
       * @return an int-value of the baudrate.
       */
      int getBaudrate()
      {
        return baudrate;
      }
      ;
      /**
       * Returns a boolean whether the serial port is opened or not. The serial port is opened
       * if data can read from and/or written to.
       * @return true if the connection is opened, otherwise returns false.
       */
      bool isOpened()
      {
        return opened;
      }

      /**
       * Sends binary data out of the serial port if it was opend before.
       * @return true if succeded, otherwise returns false.
       */
      bool writePort(std::vector<uint8> msg);

      /**
       * First validate the given connection-speed and if correct then open the serial port.
       * After there sets the specific parameter and then starts the listener.
       */
      void openPort();
      /**
       * First stops the listener and then closes the serial port. The listener is a
       * concurrent process that handles the incoming data bytes.
       */
      void closePort();
      /**
       *  Controls the 'Data Terminal Ready'-Line on the serial Connection.
       *  @param bDTR if true sets the DTR-Line (physical voltage low),
       *              otherwise resets the DTR-Line (physical voltage high).
       */
      void setDTR(bool bDTR);
      /**
       * Checks the availability of the given port.
       * @return true if the given serial Portname can be opened, otherwise false.
       */
      bool isPortAvailable(std::string pName);
      /**
       * Accumulates all available ports.
       * @return a std::vector<std::string> with all available portnames that can be opened.
       */
      std::vector<std::string> getAvailablePorts();

      /**
       * Returns the currently received data packet
       * @return
       */
      std::vector<uint8> getData()
      {
        return currentData;
      }

      virtual bool initialise();

      using CThread::pause;
      using CThread::resume;

    private:

      /**
       * This signal is emitted if a new data-packet was received correct from the serial port.
       * The data-packet is passed.
       * This is done by calling doOnCallback() for each callbackable.
       */
      void newData();

#if defined(IS_WINDOWS)
      HANDLE Win_Handle; ///< The handle for the port.
      DCB Win_CommConfig; ///< The data to control the port.
#endif

#if defined(IS_LINUX)
      int fd; ///< Handle for the port.
      struct termios oldtio; ///< Holds the system-data of the control-block until restore the setting on closePort().
#endif

      std::vector<uint8> bBuffer; ///< The buffer for accumulating the incoming data
      volatile bool opened; ///< If is set serial port is opended.

      std::vector<uint8> currentData;

      int iReadMode;
      int iCheckSum;
      unsigned int iPacketLength;

      std::string portName; ///< Holds the name of the serial port.
      unsigned int baudrate; ///< Holds the speed setting of the serial port.

      TimerThread wasteThread; // workaround to avoid that SerialPortThread does not receive anything

      void doOnCallBack(BackCaller* source, CallbackableType type) {
     //   wasteThread.restartTimer();
      }
  };

} // end namespace lpzrobots


#endif /* SERIALPORTTHREAD_H_ */
