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
 *   Revision 1.2  2009-08-11 18:26:47  guettler
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
 *   - GlobalData, ECBCommunicator is now configurable
 *   - ECBAgent rewritten: new PlotOptionEngine support, adapted to new WiredController structure
 *   - ECBRobot is now Inspectables (uses new infoLines functionality)
 *   - ECB now supports dnsNames and new communication protocol via Mediator
 *   - Better describing command definitions
 *   - SphericalRobotECB: adapted to new ECB structure, to be tested
 *                                              *
 *                                                                         *
 ***************************************************************************/

#include "SerialPortThread.h"
#include <selforg/stl_adds.h>

#include <assert.h>

using namespace std;

namespace lpzrobots
{

  SerialPortThread::SerialPortThread(std::string portName, unsigned int baudrate, bool debug) :
    CThread("SerialPortThread", debug), iReadMode(0), iCheckSum(0), iPacketLength(0), portName(portName), baudrate(
        baudrate)
  {
#if defined(IS_WINDOWS)
    portName="";
    Win_Handle = INVALID_HANDLE_VALUE;
#endif
#if defined(IS_LINUX)
    portName = "";
    fd = -1;
#endif
    baudrate = 0;
    opened = false;
    wasteThread.addCallbackable(this, TimerThread::TIMER_EXPIRED);
    wasteThread.setTimer(10,false);
  }
  SerialPortThread::~SerialPortThread()
  {
    closePort();
  }

  void SerialPortThread::openPort()
  {
    int baud;

    if (portName.length() == 0)
      return;

#if defined(IS_WINDOWS)
    switch(baudrate)
    {
      case 1200: baud=1200; break;
      case 2400: baud=2400; break;
      case 9600: baud=9600; break;
      case 19200: baud=19200; break;
      case 38400: baud=38400; break;
      case 57600: baud=57600; break;
      case 115200: baud=115200; break;
      case 230400: baud=230400; break;
      case 460800: baud=460800; break;
      case 921600: baud=921600; break;
      default: return;
    }
    Win_Handle = CreateFileA(std::string("\\\\.\\"+portName),
        GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    // Do we have a valid handle? (If not, the driver probably isn't loaded)
    if (Win_Handle == INVALID_HANDLE_VALUE)
    {
      std::cout << "Cannot open SerialPort '" << portName << "'!" << std::endl;
      return;
    }
    else
    {
      // The SetupComm() function establishes the Transmit and Receive
      // buffer sizes
      SetupComm (Win_Handle, 1024, 1024);
      // Obtain the current DCB structure. this can be saved away for restore after
      // the application is done using the Comport
      GetCommState (Win_Handle, &Win_CommConfig);
      // Fill in the DCB structure with our own settings.
      Win_CommConfig.BaudRate = baud;
      Win_CommConfig.fParity = 0;
      Win_CommConfig.fOutxCtsFlow = 0;
      Win_CommConfig.fOutxDsrFlow = 0;
      Win_CommConfig.fDtrControl = DTR_CONTROL_DISABLE;
      Win_CommConfig.fDsrSensitivity = FALSE;
      Win_CommConfig.fTXContinueOnXoff = 0;
      Win_CommConfig.fOutX = 0;
      Win_CommConfig.fInX = 0;
      Win_CommConfig.fErrorChar = 0;
      Win_CommConfig.fNull = 0;
      Win_CommConfig.fRtsControl = RTS_CONTROL_DISABLE;
      Win_CommConfig.fAbortOnError = 0;
      Win_CommConfig.ByteSize = 8;
      Win_CommConfig.Parity = NOPARITY;
      Win_CommConfig.StopBits = ONESTOPBIT;
      // Configure the comport with our new DCB
      SetCommState (Win_Handle, &Win_CommConfig);
    }
#endif
#if defined(IS_LINUX)

    switch (baudrate)
    {
      case 1200:
        baud = B1200;
        break;
      case 2400:
        baud = B2400;
        break;
      case 9600:
        baud = B9600;
        break;
      case 19200:
        baud = B19200;
        break;
      case 38400:
        baud = B38400;
        break;
      case 57600:
        baud = B57600;
        break;
      case 115200:
        baud = B115200;
        break;
      case 230400:
        baud = B230400;
        break;
      case 460800:
        baud = B460800;
        break;
      case 921600:
        baud = B921600;
        break;
      default:
        return;
    }
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); // open port

    if (fd == -1)
    {
      cout << "Cannot open SerialPort '" << portName << "'!" << endl;
      return;
    }
    // don't block serial read
    fcntl(fd, F_SETFL, FNDELAY);
    // sichere alte Einstellungen des Terminal, zur Restaurierung vor dem Schließen des Port!
    ioctl(fd, TCGETS, &oldtio);
    // Exclusiven Zugriff auf den Port anfordern
    ioctl(fd, TIOCEXCL);
    // set interface parameters
    struct termios newtio;
    newtio.c_cflag = baud | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
      cout << "Setting interface parameter failed." << endl;
    // Lösche den Eingangspuffer des geöffnente Ports
    if (tcflush(fd, TCIFLUSH) == -1)
      cout << "Flushing inputbuffer failed." << endl;
    // setzte die DTR-Leitung, ist mit der Reset-Steuerung der Hardware verbunden!
    setDTR(false);
#endif

    opened = true;
    start();

    cout << "SerialPort '" << portName << "' opened with " << baudrate << "bd." << endl;

  }

  bool SerialPortThread::isPortAvailable(std::string pName)
  {
#if defined(IS_WINDOWS)
    // Standard-Serielle Ports Windows (RS232)
    try
    {
      HANDLE h = CreateFileA(std::string("\\\\.\\").append(pName)),
      GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL );

      if( h != INVALID_HANDLE_VALUE )
      { CloseHandle(h); return true;}
    } catch (...)
    {}
    return false;
#endif
#if defined(IS_LINUX)
    try
    {
      int d = open(pName.c_str(), O_RDWR | O_NONBLOCK); // open port
      if (d != -1)
      {
        close(d);
        return true;
      }
    } catch (...)
    {
    }
    return false;
#endif
  }
  std::vector<std::string> SerialPortThread::getAvailablePorts()
  {
    std::vector<std::string> stringListSerialPortNames;

#if defined(IS_WINDOWS)
    // Suche nach vorhandenen Seriellen Anschluessen
    // Standard-Serielle Ports Windows (RS232)
    for(int i=0; i<30; i++)
    {
      std::string portname = "\\\\.\\COM"+i;
      if(isPortAvailable(portname))
      stringListSerialPortNames.push_back(portname);
    }
#endif
#if defined(IS_LINUX)
    // Standard-Serielle Ports linux (RS232)
    for (int i = 0; i < 30; i++)
    {
      std::string portname = "/dev/tty" + i;
      if (isPortAvailable(portname))
        stringListSerialPortNames.push_back(portname);
    }
    // USB-to-SerialPort-Adapter linux
    for (int i = 0; i < 30; i++)
    {
      std::string portname = "/dev/ttyUSB" + i;
      if (isPortAvailable(portname))
        stringListSerialPortNames.push_back(portname);
    }
#endif
    return stringListSerialPortNames;
  }

  void SerialPortThread::setDTR(bool bDTR)
  {
#if defined(IS_WINDOWS)
    DCB dcb;

    GetCommState (Win_Handle, &dcb);
    if(bDTR) dcb.fDtrControl = DTR_CONTROL_ENABLE; else dcb.fDtrControl = DTR_CONTROL_DISABLE;
    SetCommState (Win_Handle, &dcb);
#endif
#if defined(IS_LINUX)
    if (bDTR)
    {
      // DTR an
      int line_bits;
      ioctl(fd, TIOCMGET, &line_bits);
      line_bits |= TIOCM_DTR;
      ioctl(fd, TIOCMSET, &line_bits);
    } else
    {
      /* DTR aus */
      int line_bits;
      ioctl(fd, TIOCMGET, &line_bits);
      line_bits &= ~TIOCM_DTR;
      ioctl(fd, TIOCMSET, &line_bits);
    }
#endif
  }

  void SerialPortThread::closePort()
  {
#if defined(IS_WINDOWS)
    CloseHandle( Win_Handle );
    Win_Handle = INVALID_HANDLE_VALUE;
#endif

#if defined(IS_LINUX)
    if (fd != -1)
    {
      // Setze Port auf Original-Einstellungen zurück
      tcsetattr(fd, TCSANOW, &oldtio);
      close(fd);
      fd = -1;
    }
#endif

    stop();
    opened = false;
  }

  bool SerialPortThread::writePort(std::vector<uint8> msg)
  {
    if (!opened)
    {
      std::string s = "Can't write on SerialPort! Port is not open.";
      cout << s << endl;
    } else
    {
#if defined(IS_WINDOWS)
      // Transmit data to Serial Port
      DWORD dwBytesWritten = 0;
      WriteFile (Win_Handle, msg.data(), msg.size(), &dwBytesWritten, 0);
      if(dwBytesWritten==(DWORD)msg.size()) return true;
#endif

#if defined(IS_LINUX)
      // Transmit data to Serial Port
      int i = write(fd, msg.data(), msg.size());
      if (i == (int) msg.size())
        return true;
#endif
    }
    return false;
  }

  bool SerialPortThread::loop()
  {
    usleep(1);
    if (!opened)
    {
      std::string s = "Can't read from SerialPort! Port is not open.";
      cout << s << endl;
      return false;
    }
    unsigned char c = 0;
    unsigned int i = 0;

#if defined(IS_WINDOWS)
    COMSTAT Win_Comstat;
    DWORD Com_Errors;

    //  get one character from port
    while(i<1 && !isTerminated())
    {
      ClearCommError (Win_Handle, &Com_Errors, &Win_Comstat);
      if(Win_Comstat.cbInQue > 0) ReadFile(Win_Handle, &c, 1, (LPDWORD)&i, NULL); else usleep(10);
    }
#endif

#if defined(IS_LINUX)
    //  get one character from port
    do
    {
      i = read(fd, &c, 1);
      if (i == 1 || isTerminated())
        break;
      else
        return true;
    } while (i != 1);
#endif

    // Startzeichen der Uebertragung?
    if (c == 0x7E)
    {
      bBuffer.clear();
      iReadMode = 0;
      iCheckSum = 0;
      bBuffer.push_back(c);
    } else
    {
      if (c == 0x7D)
      {
        // Escape-Behandlung
#if defined(IS_WINDOWS)
        i=0;
        while(i<1 && !isTerminated())
        {
          ClearCommError (Win_Handle, &Com_Errors, &Win_Comstat);
          if(Win_Comstat.cbInQue > 0) ReadFile(Win_Handle, &c, 1, (LPDWORD)&i, NULL); else usleep(10);
        }
#endif

#if defined(IS_LINUX)
        // Escape-Behandlung
        do
        {
          i = read(fd, &c, 1); //  get one character from port fd
          if (i == 1 || isTerminated())
            break;
          else
            return true;
        } while (i != 1);
#endif

        c ^= 0x20;
      }

      switch (iReadMode)
      {
        case 0:
          // lese das Langenfeld (16Bit)
          // Bits[15:8]
          iPacketLength = (c << 8);
          bBuffer.push_back(c);
          iReadMode++;
          break;

        case 1:
          // Lese das Langenfeld (16Bit)
          // Bits[7:0]
          iPacketLength += (c << 0);
          bBuffer.push_back(c);
          // Wenn die Lange des Packetes groesser als 250 ist, dann nicht Empfangen!
          if (900 < iPacketLength)
            iReadMode = 0;
          iReadMode++;
          break;

        case 2:
          // Lese das Packet bis zum Ende ein
          bBuffer.push_back(c);
          iCheckSum += c;
          if (bBuffer.size() >= iPacketLength + 3)
            iReadMode = 900;
          break;

        case 900:
          // Lese die Pruefsumme ein
          iCheckSum += c;
          iCheckSum += 1;
          iCheckSum = iCheckSum % 256;

          if (iCheckSum == 0)
          {
            // inform all callbackables (ECBCommunicator)
            currentData = vector<uint8> (bBuffer);
            callBack(SerialPortThread::NEW_DATA_RECEIVED);
          } else {
            if (debug)
              cout << "checksum error!" << endl;
             callBack(SerialPortThread::DATA_CHECKSUM_ERROR);
          }
          iReadMode++;
          break;
      } // switch
    } // if
    return true;
  }

  bool SerialPortThread::initialise()
  {
    //wasteThread.restartTimer();
    return true;
  }

}

