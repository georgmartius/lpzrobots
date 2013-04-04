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

#include <stdio.h>
#include <cmath>
#include <assert.h>
#include "tcpcontroller.h"
#include <sstream>

using namespace std;

TcpController::TcpController(const string& robotname, int port, AbstractController* teacher)
  : AbstractController("tcpcontroller (" + robotname + ")", "0.1"),
    port(port), robotname(robotname),teacher(teacher) {
  addParameter("port", &port,"Port to listen for TCP connections (immutable after initialisation)");

  commands["MOTORS"       ] = MOTORS;
  commands["SENSORS"      ] = SENSORS;
  commands["STATUS"       ] = STATUS;
  commands["OBSERVE"      ] = OBSERVE;
  commands["QUIT"         ] = QUIT;
  commands["CONFIGURATION"] = CONFIGURATION;
  commands["RESET"        ] = RESET;

  number_sensors=0;
  number_motors=0;
  quit=false;
};

TcpController::~TcpController(){
  socket.close();
}

void TcpController::init(int sensornumber, int motornumber, RandGen* randGen){
  number_sensors=sensornumber;
  number_motors=motornumber;
  socket.accept(port);
  if(teacher) teacher->init(sensornumber,motornumber,randGen);
};


void TcpController::step(const sensor* sensors, int sensornumber,
                          motor* motors, int motornumber) {
  if(quit){
    cout << "TcpController::step() remote controller quit!" << endl;
    return;
  }

  string s;
  bool gotMotors=false;
  bool observing=false;
  while(!gotMotors && !quit){
    socket >> s;
    cout << "TCP: got " << s << endl;
    switch(commands[s]) {
    case MOTORS:
      getMotorCommands(motors, motornumber);
      gotMotors=true;
      break;
    case SENSORS:
      sendSensorValues(sensors, sensornumber);
      break;
    case OBSERVE:
      if(teacher){
        teacher->step(sensors,sensornumber, motors, motornumber);
        sendSensorValues(sensors, sensornumber);
        sendMotorValues(motors, motornumber);
      }else{
        cout << "TcpController::step() Observing needs teacher controller!" << endl;
        socket << "Error" << "TcpController::step() Observing needs teacher controller!";
      }
      observing=true;
      gotMotors=true;
      break;
    case STATUS:
      if(quit)
        socket << "QUIT";
      else
        socket << "OK";
      break;
    case RESET: // we have not Reset
      break;
    case QUIT:
      socket.close();
      quit=true;
      break;
    case CONFIGURATION:
      configuration();
      break;
    default:
      cout << "TcpController::step: unknown command \"" << s << "\"" << endl;
    }
  }
  if(!observing && teacher) // teacher sees sensor inputs to fill buffers
    teacher->stepNoLearning(sensors,sensornumber,motors,motornumber);

};

void TcpController::stepNoLearning(const sensor* sensors, int number_sensors,
                                    motor* motors, int number_motors) {

  step(sensors, number_sensors, motors, number_motors);
};



void TcpController::getMotorCommands(motor* motors, int motornumber){
  vector<double> ms;
  socket >> ms;
  int i=0;
  FOREACHC(vector<double>, ms, m){
    if(i<motornumber){
      motors[i]=*m;
    }
    i++;
  }
  if(i!=motornumber) cout << "TcpController::motorcommands() did not get enough motor values!"
                          << "Expect " << motornumber << " but got " << i << endl;
}

void TcpController::sendSensorValues(const sensor* sensors, int sensornumber){
  vector<double> ss(sensors,sensors+sensornumber);
  socket << ss;
}

void TcpController::sendMotorValues(const motor* motors, int motornumber){
  vector<double> ms(motors,motors+motornumber);
  socket << ms;
}


void TcpController::configuration(){
  stringstream oss;
  oss << "INTEGER " << sizeof(int) << " bytes, little endian";
  string s_integer = oss.str();
  oss.str("");
  oss << "DOUBLE " << sizeof(double) << " bytes, little endian";
  string s_double = oss.str();
  socket << "BEGIN CONFIGURATION";
  socket << "BEGIN DATA TYPES";
  socket << s_integer;
  socket << s_double;
  socket << "END DATA TYPES";
  socket << "BEGIN ROBOT DATA";
  oss.str("");
  oss << "NAME " << robotname;
  socket << robotname;
  for(int i = 0; i < number_sensors; i++)
  {
    socket << "BEGIN SENSOR";
    oss.str("");
    oss << "NAME " << "x[" << i << "]";
    socket << oss.str();
    oss.str("");
    oss << "DIMENSION 1";
    socket << oss.str();
    oss.str("");
    oss << "RAW DOMAIN " << -1.0 << " " << 1.0;
    socket << oss.str();
    oss.str("");
    oss << "MAPPED DOMAIN " << -1.0 << " " << 1.0;
    socket << oss.str();
    socket << "END SENSOR";
  }
  for(int i = 0; i < number_motors; i++) {
    socket << "BEGIN MOTOR";
    oss.str("");
    oss << "NAME " << "y[" << i << "]";
    socket << oss.str();
    oss.str("");
    oss << "DIMENSION 1";
    socket << oss.str();
    oss.str("");
    oss << "RAW DOMAIN " << -1.0 << " " << 1.0;
    socket << oss.str();
    oss.str("");
    oss << "MAPPED DOMAIN " << -1.0 << " " << 1.0;
    socket << oss.str();
    socket << "END MOTOR";
  }

  socket << "END ROBOT DATA";
  socket << "END CONFIGURATION";
}
