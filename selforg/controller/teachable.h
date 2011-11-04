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
#ifndef __TEACHABLE_H
#define __TEACHABLE_H

#include <selforg/matrix.h>

/**
   Interface for teachable controller. 
*/
class Teachable {
public:
  
  virtual ~Teachable() {}

  /** The given motor teaching signal is used for this timestep. 
      It is used as a feed forward teaching signal for the controller.
      Please note, that the teaching signal has to be given each timestep 
       for a continuous teaching process.
     @param teaching: matrix with dimensions (motornumber,1)
   */
  virtual void setMotorTeaching(const matrix::Matrix& teaching) = 0;

  /** The given sensor teaching signal (distal learning) is used for this timestep. 
      The belonging motor teachung signal is calculated by the inverse model.
      See setMotorTeaching
     @param teaching: matrix with dimensions (motorsensors,1)
   */
  virtual void setSensorTeaching(const matrix::Matrix& teaching) = 0;

  /// returns the last motor values (useful for cross motor coupling)
  virtual matrix::Matrix getLastMotorValues() = 0;

  /// returns the last sensor values (useful for cross sensor coupling)
  virtual matrix::Matrix getLastSensorValues() = 0;
};

  
#endif
