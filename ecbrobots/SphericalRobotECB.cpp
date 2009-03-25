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
 *   Revision 1.1  2009-03-25 11:16:49  robot1
 *   neue Version
 *
 *                                                                         *
 ***************************************************************************/

#include "SphericalRobotECB.h"

#include "commanddefs.h"
#include "ecbcommunicator.h"
#include <iostream>

namespace lpzrobots {

bool SphericalRobotECB::writeMotors_readSensors()
{
  if (globalData->debug)
    std::cout << "SphericalRobotECB: writeMotors_readSensors!" << std::endl;
  
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
  motorComm.command = COSCI;
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
  if ((!result.commSuccess) || ( result.sourceAddress != address ) || ( result.command != CSEN )) {// || ( result.dataLength!=currentNumberSensors) ) {
    failureCounter++;
    return false;
  }
  
//   cout << "ECB: received result.data: [";
//   for (int i=0;i<result.dataLength;i++) {
//     printf("%d ",result.data[i]);
//   }
//   cout << "]" << endl;
//   
  sensorList.clear();
  for(int i=0;i<result.dataLength;i++) {
    sensorList.push_back(convertToDouble(result.data[i]));
  }
  
  
  // reset the counter, because communication was successful
  failureCounter=0;
  return true;
}

void SphericalRobotECB::setWeightsOscillator(double x, double y)
{
}

SphericalRobotECB::~ SphericalRobotECB()
{
}

}
