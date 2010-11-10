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
 *   Revision 1.3  2010-11-10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.2  2009/08/11 15:49:06  guettler
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
 *   Revision 1.1  2009/03/25 11:16:49  robot1
 *   neue Version
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __SPHERICALROBOTECB_H
#define __SPHERICALROBOTECB_H


#include "QGlobalData.h"
#include "ecb.h"

namespace lpzrobots {

class SphericalRobotECB : public ECB {
  
public:
  
  SphericalRobotECB(std::string address, QGlobalData& globalData, ECBConfig& ecbConfig) : ECB(address, globalData, ecbConfig), x_osci(107), y_osci(107) {}
    
  virtual ~SphericalRobotECB();
  
  /**
  * set the weights-position of the sphericalRobot 
  * are x and y could become 0 ... 127 ... 255
  * for the top-view of the robot there are following possible values
  *
  *               255 (forward)
  *                |
  *                |
  *             127|
  *(left) 255--------------0 (right)
  *                |127
  *                |
  *                |
  *                0 (backward)
  *
  * So if x=127 and y=127 the weights are in middle of the robot and no move will expected
  * with x=255 and y=127 the robot will move straightforward
  */
  virtual void setWeightsOscillator(double x, double y);
  
  /**
  * this is an overwrited function in order to proceed the weight-oscillation and not the motorspeeds
  */
  virtual void sendMotorValuesPackage();


protected:
  
private:
  
  double x_osci;
  double y_osci;
  
};

}

#endif

