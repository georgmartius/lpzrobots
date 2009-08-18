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
 *   Revision 1.12  2009-08-18 14:49:37  guettler
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
 *   - GlobalData, ECBCommunicator is now configurable
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
#include "ecbcommunicator.h"
#include "globaldata.h"

#include <iostream>
#include <sstream>
#include <assert.h>

using namespace std;

namespace lpzrobots {
  ECB::ECB(string dnsName, GlobalData& globalData, ECBConfig& ecbConfig) :
    Configurable("ECB", "$ID$"), MediatorCollegue(globalData.comm), globalData(&globalData), ecbConfig(ecbConfig),
        dnsName(dnsName), networkAddress(0), serialNumber(0) {
    failureCounter = 0;
    initialised = false;

    //PORTG of ATmega128 has only 4 pins for use
    //   ecbConfig.PORTG.max_pinnumber = 4;
    //set pin 0 of PORTA as output
    //   ecbConfig.PORTA.pin[0] = OUTPUT;

    motorList.push_back(0.);
    motorList.push_back(0.);
  }

  ECB::~ECB() {
  }
  
  void ECB::sendMotorValuesPackage() {
    // at first start of ECB, it must be initialized with a reset-command
    assert(initialised && failureCounter <= globalData->maxFailures);

    if (globalData->debug)
      std::cout << "ECB(" << dnsName << "): sendMotorPackage()!" << endl;

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

    if (globalData->debug)
      std::cout << "ECB(" << dnsName << "): sendMaxMotorCurrent("<< maxCurrent << ", " << motorboardIndex << ")!" << endl;

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

    //if (globalData->debug)
      std::cout << "ECB: resetECB!(" << dnsName << ")" << endl;

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
  
  void setMaxMotorCurrent(uint8 maxCurrent, uint8 motorboardIndex=0) {

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
    int numberMotors = motorList.size();
    motorList.clear();
    int i = 0;
    for (i = beginIndex; i < numberMotors; i++) {
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

    //cout << "Converted" << (unsigned int)byteVal << "to doubleval = " << (((double) (byteVal - 128)) / 128.) << endl;
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
    //cout << "Converted" << doubleVal << "to byteval = " << (unsigned int)byteVal << endl;
    return (uint8) byteVal;
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
     ss << "mot(" << address << ")";
     }
     for (int i=0;i<ecbConfig.pcf_sensors;i++) {
     ss << "pcf(" << address << ")";
     }
     for (int i=0;i<ecbConfig.adc_sensors;i++) {
     ss << "adc(" << address << ")";
     }
     s = ss.str(); */

    std::cout << "ECB: getChannelDescription():[" << descriptionLine << "]" << std::endl;

    return descriptionLine;
  }

  void ECB::setAddresses(uint16 shortAddress, uint64 longAddress) {
    this->networkAddress = shortAddress;
    this->serialNumber = longAddress;
  }

  bool ECB::isAddressesSet() {
    if (this->serialNumber != 0)
      return true;
    return false;
  }

  void ECB::commandDimensionReceived(ECBCommunicationEvent* event) {
    if (globalData->debug)
      cout << "dimension package received: ";
    /*********************************************************************
     // GET the new description of connected motors
     // and sensors at ECB(hardware)
     // Each sensor-name should be unique to identify the sensors as well
     // The names are separated by a space-sign
     *********************************************************************/
    CommunicationData result = event->commPackage;

    // set number of motors and sensors that are in use (current)
    currentNumberMotors = result.data[0];
    currentNumberSensors = result.data[1];

    // modify the data-values with the ECB-DnsName
    stringstream ss;

    for (int i = 2; i < result.dataLength; i++) {
      if (result.data[i] == ' ')
        ss << "(" << dnsName << ")";
      ss << result.data[i];
    }

    // add the last dnsName to stringstream if its not empty
    if (result.dataLength > 0)
      ss << "(" << dnsName << ")";

    if (currentNumberSensors < ecbConfig.maxNumberSensors) {
      for (int i = currentNumberSensors; i < ecbConfig.maxNumberSensors; i++) {
        ss << " -";
      }
    }

    // complete description as a string-line
    descriptionLine = ss.str();

    if (currentNumberMotors > ecbConfig.maxNumberMotors) {
      cout << "Warning: ECB " << dnsName << " reported more motors than permitted and configured respectively!";
    }

    if (currentNumberSensors > ecbConfig.maxNumberSensors) {
      cout << "Warning: ECB " << dnsName << " reported more sensors than permitted and configured respectively!";
    }

    if (globalData->debug) {
      printf("ECB(%s) found motors: %d sensors: %d\r\n", dnsName.c_str(), currentNumberMotors, currentNumberSensors);
      std::cout << "ECB(" << dnsName << ") descriptionLine:[" << descriptionLine << "]" << std::endl;
    }

    initialised = true;
    failureCounter = 0;
    if (globalData->debug)
      cout << currentNumberMotors << " motors, " << currentNumberSensors << " sensors: " << descriptionLine << endl;

    // set max motor current if deviating from default value
    if (ecbConfig.maxMotorCurrent!=DEFAULT_MAX_MOTOR_CURRENT)
      sendMaxMotorCurrent(ecbConfig.maxMotorCurrent,0);
  }

  void ECB::commandSensorsReceived(ECBCommunicationEvent* event) {
    if (globalData->debug)
      cout << "sensor package received" << endl;
    CommunicationData result = event->commPackage;

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
      case ECBCommunicationEvent::EVENT_COMMUNICATION_ANSWER_TIMEOUT:
        if (globalData->debug)
          cout << "ECB(" << dnsName << ") did not answer: ";
        if (initialised) {
          if (failureCounter >= globalData->maxFailures) { // try to send reset next time
            if (globalData->debug)
              cout << " failure count=" << failureCounter + 1 << " reached maximum, reset to initial state." << endl;
            initialised = false;
            failureCounter = 0;
          } else {
            failureCounter++;
            if (globalData->debug)
              cout << " failure count=" << failureCounter << endl;
          }
        } else if (globalData->debug) // do nothing, try later again
          cout << " no initial response to reset command." << endl;
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

  uint16 ECB::getNetworkAddress() {
    return networkAddress;
  }

  uint64 ECB::getSerialNumber() {
    return serialNumber;
  }

} // end namespace lpzrobots
