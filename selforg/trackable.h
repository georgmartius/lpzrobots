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
#ifndef __TRACKABLE_H
#define __TRACKABLE_H

#include <selforg/position.h>
//#include <selforg/matrix.h>

namespace matrix {
  class Matrix;
}

/**
 * Abstract class (interface) for trackable objects (used for robots)
 *
 *
 */
class Trackable{
public:

  /**
   * Constructor
   */
  Trackable(){
  };

  virtual ~Trackable() {};

  /** returns name of trackable
   */
  virtual std::string getTrackableName() const =0;

  /** returns position of the object
      @return vector of position (x,y,z)
   */
  virtual Position getPosition() const =0;

  /** returns linear speed vector of the object
      @return vector  (vx,vy,vz)
   */
  virtual Position getSpeed() const =0;

  /** returns angular velocity vector of the object
      @return vector  (wx,wy,wz)
   */
  virtual Position getAngularSpeed() const =0;

  /** returns the orientation of the object
      @return 3x3 rotation matrix
   */
  virtual matrix::Matrix getOrientation() const =0;

};

#endif

