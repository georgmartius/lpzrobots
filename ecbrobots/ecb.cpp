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
 *   Revision 1.4  2008-04-11 06:15:48  guettler
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

using namespace std;

ECB::ECB ( short address,GlobalData& globalData,ECBConfig& ecbConfig ) : Configurable("ECB","$ID$"), globalData ( &globalData ), ecbConfig ( ecbConfig ),address ( address ) {
  failureCounter=0;
  initialised=false;
}

ECB::~ECB() {}

bool ECB::writeMotors_readSensors() {
  if (globalData->debug)
    std::cout << "ECB: writeMotors_readSensors!" << std::endl;
  if ( ( !initialised ) || ( failureCounter > globalData->maxFailures ) ) {
    resetECB();
  }

  if ( !initialised )
    return false;
  commData motorComm;
  motorComm.destinationAddress = address;
  motorComm.command = CMOT;
  motorComm.dataLength = currentNumberMotors;
  int i=0;
  // TODO: convert double
  FOREACH (list<motor>,motorList,m) {
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

  sensorList.clear();
  for(int i=0;i<result.dataLength;i++) {
    sensorList.push_back(convertToDouble(result.data[i]));
  }

  // reset the counter, because communication was successful
  failureCounter=0;
  return true;
}


/**
 * Send reset command to the ecb
 * @return
 */
bool ECB::resetECB() {
  if (globalData->verbose)
    std::cout << "ECB: resetECB!" << std::endl;

  //    flush input buffer
  globalData->comm->flushInputBuffer();

  initialised=false;
  // TODO: send specific config settings to ECB!
  commData reset;
  reset.destinationAddress = address;
  reset.command = CRES;
  reset.dataLength = 0;

  if ( !globalData->comm->sendData ( reset )) {
    cerr << "Error while sending reset.\n";
    return false;
  }

  commData result = globalData->comm->receiveData();

  if ((!result.commSuccess) || ( result.sourceAddress != address ) || ( result.command != CDIM ) || ( result.dataLength!=2 ) )
    return false;

  // set number of motors and sensors
  // the first data describe the number of motors
  currentNumberMotors=result.data[0];

  // the second data describe the number of sensors
  currentNumberSensors=result.data[1];

  if ( currentNumberMotors>ecbConfig.maxNumberMotors ) {
    cout << "Warning: ECB reported more motors than permitted and configured respectively!";
  }

  if ( currentNumberSensors>ecbConfig.maxNumberSensors ) {
    cout << "Warning: ECB reported more sensors than permitted and configured respectively!";
  }

  initialised=true;

  failureCounter=0;
  return true;
}


bool ECB::makeBeepECB () {
  commData mbeep;
  mbeep.destinationAddress = address;
  mbeep.command = CBEEP;
  mbeep.dataLength = 0;

  if ( !globalData->comm->sendData ( mbeep )) {
    cerr << "Error while sending make-beep.\n";
    failureCounter++;
    return false;
  }

  return true;
}


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
  return (((double)(byteVal-127))/255.);
}

int ECB::convertToByte(double doubleVal) {
  // insert check byteVal<255
  int byteVal =(int)((doubleVal+1.)*128.0);
  return (byteVal<255?byteVal:254);
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
