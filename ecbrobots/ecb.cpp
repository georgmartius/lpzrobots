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
 *   Revision 1.10  2009-03-25 11:06:55  robot1
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

#include <iostream>
#include <sstream>

using namespace std;

namespace lpzrobots {

ECB::ECB ( short address,GlobalData& globalData,ECBConfig& ecbConfig ) : Configurable("ECB","$ID$"), globalData ( &globalData ), ecbConfig ( ecbConfig ),address ( address ) {
  failureCounter=0;
  initialised=false;
  
  //PORTG of ATmega128 has only 4 pins for use
//   ecbConfig.PORTG.max_pinnumber = 4;
  //set pin 0 of PORTA as output
//   ecbConfig.PORTA.pin[0] = OUTPUT;
  
  motorList.push_back (0.);
  motorList.push_back (0.);
}

ECB::~ECB() {}

bool ECB::writeMotors_readSensors() {
  if (globalData->debug)
    std::cout << "ECB: writeMotors_readSensors! / " << initialised << std::endl;
  
  // at first start of ECB, it must be initialized with a reset-command
  if ( ( !initialised ) || ( failureCounter > globalData->maxFailures ) ) {
    resetECB();
  }
  if ( !initialised )
    return false;
  
  // if initialized, new motor-data will send to ECB (hardware)
  
  // prepare the communication-protocol 
  commData motorComm;
  motorComm.destinationAddress = address;
  motorComm.command = CMOT;
  motorComm.dataLength = currentNumberMotors;
  // set motor-data
  int i=0;
  // motorList was update by ECBAgent->ECBRobot:setMotors()->(to all ECBs)-> ECB:setMotors()
  FOREACH (list<motor>,motorList,m) {
    // Agent and Controller process with double-values
    // The ECB(hardware) have to work with byte-values
    motorComm.data[i++]=convertToByte((*m));
  }
  if (!globalData->comm->sendData(motorComm)) {
    cerr << "Error while sending motor values for ECB " << address << "." << endl;
    failureCounter++;
    return false;
  }

  commData result = globalData->comm->receiveData();
  if ((!result.commSuccess) || ( result.sourceAddress != address ) || ( result.command != CSEN ) || ( result.dataLength!=currentNumberSensors) ) {
    failureCounter++;
    return false;
  }

  /// fill sensorList which will be input for agent or controller   
  sensorList.clear();
  for(int i=0;i<result.dataLength;i++) {
    sensorList.push_back(convertToDouble(result.data[i]));
  }
 
 
  // reset the counter, because communication was successful
  failureCounter=0;
  return true;
}


/**
 * Send reset command to the ecb and
 * receive the number of motors and sensors
 * @return
 */
bool ECB::resetECB() {
  
  if (globalData->debug)
    std::cout << "ECB: resetECB!(" << address << ")" << std::endl;

  //    flush input buffer
  
//   stopMotors();
  
  globalData->comm->flushInputBuffer();

  initialised=false;
  
  // TODO: send specific config settings to ECB!
  
  /***********************************************
  // SEND the new motor-values to ECB(hardware)
  ***********************************************/
  commData reset;
  reset.destinationAddress = address;
  reset.sourceAddress = 0;
  reset.command = CRES;
  reset.dataLength = 1;
  reset.data[0]=0;
  
  if ( !globalData->comm->sendData ( reset )) {
    cerr << "Error while sending reset for ECB (" << address << ")" << ".\n";
    return false;
  }

  /*********************************************************************
  // GET the new description of connected motors
  // and sensors at ECB(hardware)
  // Each sensor-name should be unique to identify the sensors as well
  // The names are separate by a space-sign
  *********************************************************************/
  commData result = globalData->comm->receiveData();
  // data are verify to communication-protokoll ?
  if ((!result.commSuccess) || ( result.sourceAddress != address ) || ( result.command != CDIM ) )
    return false;

// set number of motors and sensors that are in use (current)
  currentNumberMotors = result.data[0];
  currentNumberSensors = result.data[1];
  
  // modify the data-values with the ECB-address
  stringstream ss;

  for (int i=2;i<result.dataLength;i++) {
    if (result.data[i]==' ') 
      ss << "(" << address << ")";
    ss << result.data[i];
  }
  
 // add the last address to stringstream if it not empty
  if (result.dataLength>0)
    ss << "(" << address << ")";
  
  if (currentNumberSensors < ecbConfig.maxNumberSensors) {
    for (int i=currentNumberSensors;i<ecbConfig.maxNumberSensors;i++) {
      ss << " -";
    }
  }
  
  // complete description as a string-line
  descriptionLine = ss.str();
  
  if ( currentNumberMotors>ecbConfig.maxNumberMotors ) {
    cout << "Warning: ECB " << address << " reported more motors than permitted and configured respectively!";
  }

  if ( currentNumberSensors>ecbConfig.maxNumberSensors ) {
    cout << "Warning: ECB " << address << " reported more sensors than permitted and configured respectively!";
  }

  if (globalData->debug) {
    printf("ECB(%d) found motors: %d sensors: %d\r\n",address,currentNumberMotors, currentNumberSensors);
    std::cout << "ECB("<< address << ") descriptionLine:[" << descriptionLine << "]" << std::endl;
  }
  
  initialised=true;

  failureCounter=0;
  
//   startMotors();
  
  return true;
}

bool ECB::makeBeepECB () {
  commData mbeep;
  mbeep.destinationAddress = address;
  mbeep.command = CBEEP;
  mbeep.dataLength = 0;

  if ( !globalData->comm->sendData ( mbeep )) {
    cerr << "Error while sending make-beep for ECB " << address << ".\n";
    failureCounter++;
    return false;
  }

  return true;
}

bool ECB::stopMotors() {
  
  commData mstop;
  mstop.destinationAddress = address;
  mstop.command = CMSTOP;
  mstop.dataLength = 0;
  if ( !globalData->comm->sendData ( mstop )) {
    cerr << "Error while sending make-stop for ECB " << address << ".\n";
    failureCounter++;
    return false;
  }
  return true;
}

bool ECB::startMotors() {
  
  commData mstart;
  mstart.destinationAddress = address;
  mstart.command = CMSTART;
  mstart.dataLength = 0;
  if ( !globalData->comm->sendData ( mstart )) {
    cerr << "Error while sending make-start for ECB " << address << ".\n";
    failureCounter++;
    return false;
  }
  return true;
}


/**
* The ECBRobot hold a list of motor-values. To distribute this values
* to all connected ECBs is do by follow: Every ECB get the hole motor-value-array.
* Additional the beginIndex for start-position (first ECB have the start-pos.=0).
* The ECB-Function setMotors will add the values into motor-list itself, until the
* number of his motorNumber. After that, this function will return the number of
* adding values. This number is equal to the start-position of the next ECB motorList.
*
* motorArray - hold the new motor-values from controller
* beginIndex - the start-pos of motor-array that will be add into ECB-motorList
* maxIndex   - size of motorArray
*
* return     - the number of adding motor-values
*/
int ECB::setMotors (const motor* motorArray,int beginIndex,int maxIndex ) {
  int numberMotors = motorList.size();
  motorList.clear();
  int i=0;
  for ( i=beginIndex;i<numberMotors;i++ ) {
    motorList.push_back ( motorArray[i] );
    if ( i==maxIndex )
      break;
  }
  return ( i-beginIndex );
}

int ECB::getNumberMotors() {
  return ( currentNumberMotors>ecbConfig.maxNumberMotors?ecbConfig.maxNumberMotors:currentNumberMotors );
}

int ECB::getNumberSensors() {
  return ( currentNumberSensors>ecbConfig.maxNumberSensors?ecbConfig.maxNumberSensors:currentNumberSensors );
}

int ECB::getMaxNumberMotors() {
  return ( ecbConfig.maxNumberMotors);
}

int ECB::getMaxNumberSensors() {
  return ( ecbConfig.maxNumberSensors);
}

/// helper functions

double ECB::convertToDouble(int byteVal) {
  // 0 <= byteVal <= 255
  if (byteVal >= 255) byteVal=255;
  if (byteVal <= 0) byteVal=0;
  
  return (((double)(byteVal-127))/128.);
  
//   return (((double)(byteVal-127))/255.);
}

int ECB::convertToByte(double doubleVal) {
  // insert check byteVal<255
  // -1 <= doubleVal <= 1
  if (doubleVal >= 1) doubleVal=1;
  if (doubleVal <= -1) doubleVal=-1;
  
  int byteVal =(int)((doubleVal+1.)*128.0);
  //(byteVal<255?byteVal:254);
  if (byteVal>255) byteVal=254;
//   printf("double=%1.3E to byte=%d\r\n",doubleVal, byteVal);
  return byteVal;
}



/// STORABLE INTERFACE

/** stores the object to the given file stream (binary). */
bool ECB::store ( FILE* f ) const {
  // viel Spaß!
  return true;
}

/** loads the object from the given file stream (binary). */
bool ECB::restore ( FILE* f ) {
  // xml parser anwerfen
  // viel Spaß!
  return true;
}



}

std::string lpzrobots::ECB::getChannelDescription()
{
  // check if initialised (normally not)
  if ( ( !initialised ) || ( failureCounter > globalData->maxFailures ) ) {
    resetECB();
  }
  
  if ( !initialised )
    return "";
  
  /*
  
  stringstream ss;
  
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
  
  s = ss.str();
  */
  std::cout << "ECB: getChannelDescription():[" << descriptionLine << "]" << std::endl;
  
  
  return descriptionLine;
}

