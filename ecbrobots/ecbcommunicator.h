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
 *   Revision 1.3  2008-04-11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.2  2008/04/08 09:09:09  martius
 *   fixed globaldata to pointer in classes
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __ECBCOMMUNICATOR_H
#define __ECBCOMMUNICATOR_H

#include "cserialthread.h"
#include <selforg/configurable.h>

#include "globaldata.h"

namespace lpzrobots {


/**
 * TODO: überarbeiten
   This class is the robot and the communicator in one.
   This might be a bit confusing, because it contains also the agent and so on.
   The main flow is as follows:
     loop:
      over serial we send motor values to the Xbees
      we wait for sensor values from Xbees
      then we invoke the agent
      the agent asks the robot (us) about sensors (which we saved in a local variable)
      the agent calls the controller and then sets the motor values with setMotors (also in this class)
     endloop
*/

typedef struct  {
  bool commSuccess;  // when receiving data, this var indicates if comm protocol was correct
  uint8_t destinationAddress;  // destination address, 0xFF is forbidden!
  uint8_t sourceAddress;  // source address, 0xFF is forbidden!
  uint8_t command;  // command byte, 0xFF is forbidden!
  uint8_t dataLength;   // data length of data pointer or array, 0xFF is forbidden!
  uint8_t data[254];    // data byte pointer or array, 0xFF is forbidden!
} commData;



class ECBCommunicator : public CSerialThread {

public:


  ECBCommunicator(GlobalData& globalData);

  virtual ~ECBCommunicator() {};

  /**
   * Receives data from any ECB, all neccessary values are then stored in commData
   * @return commData contains sourceAddress, dataLength, data,...
   */
  virtual commData receiveData();


  /**
   * Sends data to a specific ECB, all neccessary values are defined in commData
   * @param commData contains destinationAddress, dataLength, data,...
   * @return true if communication was successful, otherwise false
   */
  virtual bool sendData(commData& commData);

  /**
   * Flushes the input buffer (removes all packets which are in the buffer)
   * @param time the time to wait for flushing
   */
  virtual void flushInputBuffer(int time=-1);


protected:

  virtual void initialise();

  /** This function is called by CSerialThread every step
   * @return true if loop() should be recalled, otherwise false (thread stops)
   */
  virtual bool loop();


private:
  long realtimeoffset;
  GlobalData* globalData;
  long lastBenchmarkTime;

  void printMsg(int xbee, int addr, uint8* data, int len) {
    data[len]=0;
    fprintf(stdout, "Message from slave (%i (addr: %i)): %s\n", xbee, addr, data);
  }

  long timeOfDayinMS();

  void resetSyncTimer();

  void loopCallback();



// void onTermination(){
//   cmd_end_input();
//   fprintf(stderr,"Try to stop serial thread\n");
//   if(communication){
//     communication->stop();
//   }
// }



};


}

/// special wiring for the spherical Robot
// class OurWiring : public AbstractWiring{
// public:
//   OurWiring(NoiseGenerator* noise)
//     : AbstractWiring(noise){
//       lastmotors = 0;
//       numMotorSensors=2;
//     }
//   virtual ~OurWiring() {
//     if(lastmotors) free(lastmotors);
//   }
//
//   virtual bool init(int robotsensornumber, int robotmotornumber){
//     rsensornumber = robotsensornumber;
//     rmotornumber  = robotmotornumber;
//     // csensornumber = rsensornumber;
//     csensornumber = numMotorSensors + (rsensornumber-numMotorSensors)*2;    // for IR sensors create two sensors each
//     cmotornumber  = rmotornumber;
//     if(!noiseGenerator) return false;
//     noiseGenerator->init(csensornumber); // initialised with number of sensors ( see add() below)
//     lastmotors = (motor*)malloc(sizeof(motor)* rmotornumber);
//     return true;
//   }
//
//   virtual bool wireSensors(const sensor* rsensors, int rsensornumber,
//                            sensor* csensors, int csensornumber,
//                            double noise){
//     // copy motor sensors
//                              for(int i=0; i< numMotorSensors; i++){
//                                csensors[i] = rsensors[i];
//                              }
//     // for IR Sensors
//                              int j = numMotorSensors;
//                              for(int i=numMotorSensors; i< rsensornumber; i++){
//                                csensors[j]   = rsensors[i] * lastmotors[0];
//                                csensors[j+1] = rsensors[i] * lastmotors[1];
//                                j+=2;
//                              }
//                              return true;
//                            }
//
//   virtual bool wireMotors(motor* rmotors, int rmotornumber,
//                           const motor* cmotors, int cmotornumber){
//                             memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
//                             memcpy(lastmotors, cmotors, sizeof(motor)*rmotornumber);
//                             return true;
//                           }
//
//
// private:
//   motor* lastmotors;
//   int numMotorSensors;
// };

#endif
