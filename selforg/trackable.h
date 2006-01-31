/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   Revision 1.1.2.4  2006-01-31 15:47:37  martius
 *   virtual destructor
 *
 *   Revision 1.1.2.3  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.2  2005/11/16 11:23:30  martius
 *   const
 *
 *   Revision 1.1.2.1  2005/11/14 17:37:56  martius
 *   moved to selforg
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __TRACKABLE_H
#define __TRACKABLE_H
 
#include "position.h"
#include <matrix.h>

/**
 * Abstract class (interface) for trackable objects (used for robots)
 * 
 * 
 */
class Trackable{
public:

  /**
   * Constructor
   * @param name name of the robot
   */
  Trackable(){
  };
  
  virtual ~Trackable() {};

  /** returns position of the object
      @return vector of position (x,y,z)
   */
  virtual Position getPosition() const =0;

  /** returns linear speed vector of the object
      @return vector  (vx,vy,vz)
   */
  virtual Position getSpeed() const =0;

  /** returns the orientation of the object
      @return 3x3 rotation matrix
   */
  virtual matrix::Matrix getOrientation() const =0;

};

#endif

