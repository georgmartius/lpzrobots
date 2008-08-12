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
 *   Revision 1.6  2008-08-12 11:45:50  guettler
 *   plug and play update, added some features for the ECBRobotGUI
 *
 *   Revision 1.5  2008/07/30 07:42:47  robot3
 *   wolfgang: - reverted to version 1.3
 *
 *   Revision 1.4  2008/07/30 07:39:23  robot3
 *   wolfgang: - added function to set...
 *   - added other things
 *
 *   Revision 1.3  2008/04/11 06:31:16  guettler
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
#ifndef __ECBROBOT_H
#define __ECBROBOT_H

#include <selforg/abstractrobot.h>
#include <selforg/inspectable.h>

#include <list>
#include "ecb.h"

#include <stdio.h>

#include "globaldata.h"

namespace lpzrobots {


class ECBRobot : public AbstractRobot, public Inspectable {
public:

  ECBRobot(GlobalData& globalData);

  virtual ~ECBRobot();


  /// ABSTRACTROBOT INTERFACE

  /** returns actual sensorvalues
  @param sensors sensors scaled to [-1,1]
  @param sensornumber length of the sensor array
  @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber);

  /** sets actual motorcommands
  @param motors motors scaled to [-1,1]
  @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber);

  /** returns number of sensors */
  virtual int getSensorNumber();

  /** returns number of motors */
  virtual int getMotorNumber();

  /** returns max number of sensors */
  virtual int getMaxSensorNumber();

  /** returns maxnumber of motors */
  virtual int getMaxMotorNumber();

  /// TRACKABLE INTERFACE

  virtual Position getPosition() const;
  virtual Position getSpeed() const;
  virtual Position getAngularSpeed() const;
  virtual matrix::Matrix getOrientation() const;


  /// CONFIGURABLE INTERFACE

  virtual paramval getParam(const paramkey& key) const;

  virtual bool setParam(const paramkey& key, paramval val);

  virtual paramlist getParamList() const;


  /// new methods for the communicator

  /**
   * This method is for the user for adding new ECBss in the start function
   * @param slaveAddress the address of the ECB
   * @param ecbConfig The config of the ECB, @see ecb.h
   */
  virtual void addECB(int slaveAddress, ECBConfig& ecbConfig);


  /**
   * For the communicator, accessing all ECBs of a robot
   * @return
   */
  virtual std::list< ECB * > getECBlist() const { return ECBlist; }

  virtual void writeMotors_readSensors();

  /**
   * Returns specific ECBRobot infos to the ECBAgent, who pipes this infos out (PlotOptions)
   * Something like that:
   * #ECB M y[0] y[1]
   * #ECB IR x[0] x[1]
   * #ECB ADC x[2] x[3]
   * #ECB ME x[4] x[5]
   * Strom, Spannung usw. (konfigurationsabhängige Parameter vom ECB)
   */
  virtual std::string getGUIInformation();


private:
  std::list<ECB*> ECBlist;
  GlobalData* globalData;
  Inspectable::paramval* speed; // example

};

}

#endif
