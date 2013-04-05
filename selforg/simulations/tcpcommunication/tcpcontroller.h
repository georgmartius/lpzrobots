/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
#ifndef __TCPCONTROLLER_H
#define __TCPCONTROLLER_H

#include <stdio.h>
#include <selforg/abstractcontroller.h>
#include "Socket.h"

/**
 * class for robot control via a remote tcp controller
 *
 * Todo add protocol description
 */
class TcpController : public AbstractController {
public:
  enum CommandID{MOTORS=1,SENSORS,STATUS,QUIT,OBSERVE,CONFIGURATION,RESET};

  typedef std::map<std::string, CommandID> CommandList;


  /**
     @param port Port number to listen for controller
     @param robotname name of robot to send to controller
   */
  TcpController(const string& robotname, int port = 4000, AbstractController* teacher = 0);

  virtual ~TcpController();

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  /** @return Number of sensors the controller was initialised
      with or 0 if not initialised */
  virtual int getSensorNumber() const {return number_sensors;}


  /** @return Number of motors the controller was initialised
      with or 0 if not initialised */
  virtual int getMotorNumber() const {return number_motors;}

  /** performs one step (does communication with remote controller)
      and waits for motor values to be send to the robot.
  */
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber);

  /** performs one step without learning (Not implemented
      for remote controller! we use step instead)
      @see step
  */
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    Configurable::parse(f);
    return true;
  }

protected:
  void getMotorCommands(motor* motors, int motornumber);
  void sendSensorValues(const sensor* sensors, int sensornumber);
  void sendMotorValues(const motor* motors, int motornumber);
  void configuration();

protected:
  int number_sensors;
  int number_motors;
  paramint port;
  std::string robotname;
  bool quit;
  AbstractController* teacher;

  Socket      socket;
  CommandList commands;
};

#endif
