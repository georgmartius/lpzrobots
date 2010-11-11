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
 *   Revision 1.8  2010-11-11 15:34:59  wrabe
 *   - some extensions for QMessageClient (e.g. quitServer())
 *   - fixed some includes
 *
 *   Revision 1.7  2010/11/10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.6  2009/08/18 14:49:37  guettler
 *   implemented COMMAND_MOTOR_MAX_CURRENT
 *
 *   Revision 1.5  2009/08/11 15:49:05  guettler
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
 *                                                                         *
 ***************************************************************************/
#ifndef __COMMANDDEFS_H
#define __COMMANDDEFS_H

#include "constants.h"
#include "types.h"

namespace lpzrobots
{

  /// types of different commands
  static const uint8 COMMAND_DUMMY = 0x0; //!< 00000000 Dummy command, do not use this!
  static const uint8 COMMAND_RESET = 0x1; //!<  00000001 Reset command
  static const uint8 COMMAND_DIMENSION = 0x2; //!<  00000010 Dimension data: number of sensors/motors
  static const uint8 COMMAND_SENSORS = 0x3; //!<  00000011 Sensor data values
  static const uint8 COMMAND_MOTORS = 0x4; //!<  00000100 Motor data values
  static const uint8 COMMAND_BEEP = 0x8; //!<  00001000 Make a beep
  static const uint8 COMMAND_MESSAGE = 0x9; //!<  00001001 Message data.
  static const uint8 COMMAND_TESTI2C = 0x0A; //!<  00001010 Test i2c communication.
  static const uint8 COMMAND_PING = 0x0B; //!<  COMMAND_ommunication Test between PCOMMAND_ and ATmega
  static const uint8 COMMAND_PONG = 0x0C; //!<  COMMAND_ommunication Test between PCOMMAND_ and ATmega
  static const uint8 COMMAND_MOTOR_STOP = 0x0D; //!<  stop motors of the md23-motorboard
  static const uint8 COMMAND_MOTOR_START = 0x0E; //!<  start motors of the md23-motorboard
  static const uint8 COMMAND_OSCI = 0x0F; //!<  define the oscillation of the SphericalRobots-Weights
  static const uint8 COMMAND_MOTOR_CURRENT_LIMIT = 0x10; //!< Sets the maximum current for the motorboards (from Wolfgang Rabe, Version 1.0)

}

#endif
