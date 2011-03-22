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
 *   Revision 1.18  2011-03-22 16:36:29  guettler
 *   - ECB is now inspectable
 *
 *   Revision 1.17  2011/02/04 12:59:05  wrabe
 *   - convert function for short values added
 *
 *   Revision 1.16  2011/01/24 14:16:25  guettler
 *   - removed deprecated comments
 *   - setMotors now uses currentNumberMotors instead of maximum value
 *
 *   Revision 1.15  2010/11/26 12:22:37  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *   Revision 1.14  2010/11/11 15:34:59  wrabe
 *   - some extensions for QMessageClient (e.g. quitServer())
 *   - fixed some includes
 *
 *   Revision 1.13  2010/11/10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.12  2009/08/18 14:49:37  guettler
 *   implemented COMMAND_MOTOR_MAX_CURRENT
 *
 *   Revision 1.11  2009/08/11 15:49:05  guettler
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
 *   Revision 1.10  2009/03/25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.9  2008/08/15 13:16:58  robot1
 *   add PORT PIN configuration for ECBs
 *
 *   Revision 1.8  2008/08/12 11:45:20  guettler
 *   plug and play update, added some features for the ECBRobotGUI
 *
 *   Revision 1.7  2008/07/16 14:37:17  robot1
 *   -simple getc included
 *   -extended config on reset
 *   -minor changes
 *
 *   Revision 1.6  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.5  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.4  2008/04/11 06:15:48  guettler
 *   Inserted convertion from byte to double and backwards for motor and sensor values
 *
 *   Revision 1.3  2008/04/08 10:11:03  guettler
 *   alpha testing
 *
 *   Revision 1.2  2008/04/08 09:09:09  martius
 *   fixed globaldata to pointer in classes
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#include "ecb.h"
#include "commanddefs.h"
#include "QECBCommunicator.h"
#include "ECBCommunicationData.h"
#include "QGlobalData.h"

#include <iostream>
#include <sstream>
#include <assert.h>

using namespace std;

namespace lpzrobots {
  ECB::ECB(QString dnsName, QGlobalData& globalData, ECBConfig& ecbConfig) :
    Configurable(dnsName.toStdString(), "$ID$"), Inspectable(dnsName.toStdString()), MediatorCollegue(globalData.comm), globalData(&globalData), ecbConfig(ecbConfig), dnsName(dnsName) {
    failureCounter = 0;
    initialised = false;
  }

  ECB::~ECB() {
  }
  
  void ECB::sendMotorValuesPackage() {
    // at first start of ECB, it must be initialized with a reset-command
    assert(initialised && failureCounter <= globalData->maxFailures);

    globalData->textLog("ECB(" + dnsName + "): sendMotorPackage()!");

    // prepare the communication-protocol
    ECBCommunicationEvent* event = new ECBCommunicationEvent(ECBCommunicationEvent::EVENT_REQUEST_SEND_COMMAND_PACKAGE);

    event->commPackage.command = COMMAND_MOTORS;
    event->commPackage.dataLength = currentNumberMotors;
    // set motor-data
    int i = 0;
    // motorList was update by ECBAgent->ECBRobot:setMotors()->(to all ECBs)-> ECB:setMotors()
    FOREACH (list<motor>,motorList,m) {
      // Agent and Controller process with double-values
      // The ECB(hardware) has to work with byte-values
      event->commPackage.data[i++] = convertToByte((*m));
    }
    informMediator(event);
  }
  
  void ECB::sendMaxMotorCurrent(uint8 maxCurrent, uint8 motorboardIndex) {
    // at first start of ECB, it must be initialized with a reset-command
    assert(initialised && failureCounter <= globalData->maxFailures);

    globalData->textLog("ECB(" + dnsName + "): sendMaxMotorCurrent(" + maxCurrent + ", " + motorboardIndex + ")!");

    // prepare the communication-protocol
    ECBCommunicationEvent* event = new ECBCommunicationEvent(ECBCommunicationEvent::EVENT_REQUEST_SEND_COMMAND_PACKAGE);

    event->commPackage.command = COMMAND_MOTOR_CURRENT_LIMIT;
    event->commPackage.dataLength = 2;

    event->commPackage.data[0] = motorboardIndex;
    event->commPackage.data[1] = maxCurrent;
    informMediator(event);
  }

  /**
   * Send reset command to the ecb and
   * receive the number of motors and sensors
   * @return
   */
  void ECB::sendResetECB() {

    //if (globalData->debugOutput)
    globalData->textLog("ECB: resetECB!(" + dnsName + ")");

    // globalData->comm->flushInputBuffer();

    initialised = false;

    // TODO: send specific config settings to ECB!

    generateAndFireEventForCommand(COMMAND_RESET);
  }

  void ECB::makeBeepECB() {
    generateAndFireEventForCommand(COMMAND_BEEP);
  }
  
  void ECB::stopMotors() {
    generateAndFireEventForCommand(COMMAND_MOTOR_STOP);
  }
  
  void setMaxMotorCurrent(uint8 maxCurrent, uint8 motorboardIndex = 0) {

  }

  void ECB::startMotors() {
    generateAndFireEventForCommand(COMMAND_MOTOR_START);
  }
  
  /**
   * The ECBRobot holds a list of motor-values. To distribute this values
   * to all connected ECBs is done by following: Every ECB get the hole motor-value-array.
   * Additional the beginIndex for start-position (first ECB have the start-pos.=0).
   * The ECB-Function setMotors will add the values into motor-list itself, until the
   * number of his motorNumber. After that, this function will return the number of
   * added values. This number is equal to the start-position of the next ECB motorList.
   *
   * motorArray - hold the new motor-values from controller
   * beginIndex - the start-pos of motor-array that will be add into ECB-motorList
   * maxIndex   - size of motorArray
   *
   * return     - the number of adding motor-values
   */
  int ECB::setMotors(const motor* motorArray, int beginIndex, int maxIndex) {
    motorList.clear();
    int i = 0;
    for (i = beginIndex; i < currentNumberMotors; i++) {
      motorList.push_back(motorArray[i]);
      if (i == maxIndex)
        break;
    }
    return (i - beginIndex);
  }
  
  int ECB::getNumberMotors() {
    return (currentNumberMotors > ecbConfig.maxNumberMotors ? ecbConfig.maxNumberMotors : currentNumberMotors);
  }
  
  int ECB::getNumberSensors() {
    return (currentNumberSensors > ecbConfig.maxNumberSensors ? ecbConfig.maxNumberSensors : currentNumberSensors);
  }
  
  int ECB::getMaxNumberMotors() {
    return (ecbConfig.maxNumberMotors);
  }
  
  int ECB::getMaxNumberSensors() {
    return (ecbConfig.maxNumberSensors);
  }

  /// helper functions

  double ECB::convertToDouble(uint8 byteVal) {
    // 0 <= byteVal <= 255
    if (byteVal >= 255)
      byteVal = 255;
    if (byteVal <= 0)
      byteVal = 0;

    //globalData->textLog("Converted" + (unsigned int)byteVal + "to doubleval = " + (((double) (byteVal - 128)) / 128.));
    return (((double) (byteVal - 128)) / 128.);

  }

  uint8 ECB::convertToByte(double doubleVal) {
    // insert check byteVal<255
    // -1 <= doubleVal <= 1
    if (doubleVal >= 1)
      doubleVal = 1;
    if (doubleVal <= -1)
      doubleVal = -1;

    int byteVal = (int) ((doubleVal + 1.) * 128.0);
    if (byteVal > 255)
      byteVal = 255;
    if (byteVal < 0)
      byteVal = 0;
    //globalData->textLog("Converted" + doubleVal + "to byteval = " + (unsigned int)byteVal);
    return (uint8) byteVal;
  }

  double ECB::convertToDouble(short shortVal, short minBound, short maxBound) {
    assert(minBound <= maxBound);
    assert(minBound !=0 && maxBound!=0);
    if (shortVal > maxBound)
      shortVal = maxBound;
    if (shortVal < minBound)
      shortVal = minBound;
    // shift zero point!
    shortVal += (maxBound + minBound)/2;
    if (shortVal > 0) {
      return (shortVal / (double)maxBound);
    } else if (shortVal < 0) {
      return -(shortVal / (double)minBound);
    } else {
      return 0;
    }
  }

  short ECB::convertToShort(double doubleVal, short minBound, short maxBound) {
    if (doubleVal >= 1)
      doubleVal = 1;
    if (doubleVal <= -1)
      doubleVal = -1;

    int zero = (maxBound + minBound)/2;
    int range = maxBound - minBound;
    short shortVal = (short) (doubleVal * 0.5 * range + zero);
    return shortVal;
  }

  /// STORABLE INTERFACE

  /** stores the object to the given file stream (binary). */
  bool ECB::store(FILE* f) const {
    // viel Spaß!
    return true;
  }
  
  /** loads the object from the given file stream (binary). */
  bool ECB::restore(FILE* f) {
    // xml parser anwerfen
    // viel Spaß!
    return true;
  }

  std::string ECB::getChannelDescription() {
    if (!initialised)
      return "";

    /*stringstream ss;
     std::string s="";
     for (int i=0;i<ecbConfig.md23_sensors;i++) {
     ss + "mot(" + address + ")";
     }
     for (int i=0;i<ecbConfig.pcf_sensors;i++) {
     ss + "pcf(" + address + ")";
     }
     for (int i=0;i<ecbConfig.adc_sensors;i++) {
     ss + "adc(" + address + ")";
     }
     s = ss.str(); */

    globalData->textLog("ECB: getChannelDescription():[" + QString(descriptionLine.c_str()) + "]");

    return descriptionLine;
  }

  void ECB::commandDimensionReceived(ECBCommunicationEvent* event) {
    globalData->textLog("dimension package received: ");
    /*********************************************************************
     // GET the new description of connected motors
     // and sensors at ECB(hardware)
     // Each sensor-name should be unique to identify the sensors as well
     // The names are separated by a space-sign
     *********************************************************************/
    ECBCommunicationData result = event->commPackage;

    // set number of motors and sensors that are in use (current)
    currentNumberMotors = result.data[0];
    currentNumberSensors = result.data[1];

    globalData->textLog("[" + dnsName + "] found motors: " + QString::number(currentNumberMotors) + ", sensors: " + QString::number(currentNumberSensors));

    // modify the data-values with the ECB-DnsName
    stringstream ss;
    QString description;
    globalData->textLog("[" + dnsName + "] descriptionLine:");

    for (int i = 2; i < result.dataLength; i++) {
      if (result.data[i] == ' ') {
        ss << "(" << dnsName.toStdString() << ")";
        globalData->textLog("   - " + description);
        description.clear();
      } else {
        ss << (uchar) result.data[i];
        description.append(result.data[i]);
      }
    }
    // add the last dnsName to stringstream if its not empty
    if (result.dataLength > 0)
      ss << "(" + dnsName.toStdString() << ")";

    if (currentNumberSensors < ecbConfig.maxNumberSensors) {
      for (int i = currentNumberSensors; i < ecbConfig.maxNumberSensors; i++) {
        ss << " -";
      }
    }

    // complete description as a string-line
    descriptionLine = ss.str();

    if (currentNumberMotors > ecbConfig.maxNumberMotors) {
      globalData->textLog("Warning: ECB " + dnsName + " reported more motors than permitted and configured respectively!");
    }

    if (currentNumberSensors > ecbConfig.maxNumberSensors) {
      globalData->textLog("Warning: ECB " + dnsName + " reported more sensors than permitted and configured respectively!");
    }

    //globalData->textLog("[" + dnsName + "] descriptionLine:[" + QString(descriptionLine.c_str()) + "]");

    initialised = true;
    failureCounter = 0;

    // set max motor current if deviating from default value
    if (ecbConfig.maxMotorCurrent != DEFAULT_MAX_MOTOR_CURRENT)
      sendMaxMotorCurrent(ecbConfig.maxMotorCurrent, 0);
  }

  void ECB::commandSensorsReceived(ECBCommunicationEvent* event) {
    globalData->textLog("sensor package received");
    ECBCommunicationData result = event->commPackage;

    /// fill sensorList which will be the input for agent or controller
    sensorList.clear();
    for (int i = 0; i < result.dataLength; i++) {
      sensorList.push_back(convertToDouble(result.data[i]));
    }

    // reset the counter, because communication was successful
    failureCounter = 0;
  }

  /**
   * Is called when the mediator informs this collegue that an event
   * has to be performed by this collegue instance.
   */
  void ECB::doOnMediatorCallBack(MediatorEvent* event) {
    ECBCommunicationEvent* commEvent = static_cast<ECBCommunicationEvent*> (event);
    switch (commEvent->type) {
      case ECBCommunicationEvent::EVENT_PACKAGE_SENSORS_RECEIVED:
        commandSensorsReceived(commEvent);
        break;
      case ECBCommunicationEvent::EVENT_PACKAGE_DIMENSION_RECEIVED:
        commandDimensionReceived(commEvent);
        break;
      case ECBCommunicationEvent::EVENT_REQUEST_SEND_MOTOR_PACKAGE:
        if (initialised)
          sendMotorValuesPackage();
        else
          sendResetECB();
        break;
      case ECBCommunicationEvent::EVENT_REQUEST_SEND_MOTOR_STOP_PACKAGE:
        if (initialised)
          stopMotors();
        break;
      case ECBCommunicationEvent::EVENT_COMMUNICATION_ANSWER_TIMEOUT:
        globalData->textLog("ECB(" + dnsName + ") did not answer: ");
        if (initialised) {
          if (failureCounter >= globalData->maxFailures) { // try to send reset next time
            globalData->textLog(" failure count=" + QString::number(failureCounter + 1) + " reached maximum, reset to initial state.");
            initialised = false;
            failureCounter = 0;
          } else {
            failureCounter++;
            globalData->textLog(" failure count=" + failureCounter);
          }
        } else
          // do nothing, try later again
          globalData->textLog(" no initial response to reset command.");
        break;
      default:
        break;
    }

  }

  void ECB::generateAndFireEventForCommand(uint8 command) {
    ECBCommunicationEvent* event = new ECBCommunicationEvent(ECBCommunicationEvent::EVENT_REQUEST_SEND_COMMAND_PACKAGE);

    event->commPackage.command = command;
    event->commPackage.dataLength = 0;

    informMediator(event);
  }

} // end namespace lpzrobots
