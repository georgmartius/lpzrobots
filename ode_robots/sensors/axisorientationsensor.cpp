/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2006-12-21 11:42:10  martius
 *   sensors have dimension to sense
 *   axissensors have finer settings
 *
 *   Revision 1.2  2006/08/11 15:45:38  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2006/08/08 17:03:27  martius
 *   new sensors model
 *
 *
 *                                                                 *
 ***************************************************************************/

#include <selforg/matrix.h>

#include "primitive.h"
#include "axisorientationsensor.h"
#include "mathutils.h"

namespace lpzrobots {

  AxisOrientationSensor::AxisOrientationSensor(Mode mode, short dimensions) 
    : mode(mode), dimensions(dimensions) {
    own = 0;
  }

  void AxisOrientationSensor::init(Primitive* own){
    this->own = own;
  }

  int AxisOrientationSensor::getSensorNumber() const{
    
    short n = ((dimensions & X) != 0) + ((dimensions & Y) != 0) + ((dimensions & Z) != 0);
    switch (mode) {
    case OnlyZAxis: 
    case ZProjection: 
      return n;
      break;
    case Axis:
      return 3*n;
      break;
    }      
    return 0;
  }
  
  bool AxisOrientationSensor::sense(const GlobalData& globaldata) { return true; }

  std::list<sensor> AxisOrientationSensor::get() const {
    assert(own);
    matrix::Matrix A = odeRto3x3RotationMatrix ( dBodyGetRotation ( own->getBody() ) );
    
    switch (mode) {
    case OnlyZAxis: 
      if(dimensions == (X | Y | Z)) return A.column(2).convertToList();
      else return selectrows(A.column(2),dimensions); break;
    case ZProjection: 
      if(dimensions == (X | Y | Z)) return A.row(2).convertToList();
      else return selectrows(A.row(2)^(matrix::T),dimensions);  break;
    case Axis:
      if(dimensions == (X | Y | Z)) return A.convertToList();
      else return selectrows(A,dimensions); break;
    }    
    return std::list<sensor>();
  }

  int AxisOrientationSensor::get(sensor* sensors, int length) const{
    assert(own); 
    matrix::Matrix A = odeRto3x3RotationMatrix ( dBodyGetRotation ( own->getBody() ) );
    switch (mode) {
    case OnlyZAxis: 
      if(dimensions == (X | Y | Z)) return A.column(2).convertToBuffer(sensors, length);
      else return selectrows(sensors, length, A.column(2),dimensions); break;
    case ZProjection: 
      if(dimensions == (X | Y | Z)) return A.row(2).convertToBuffer(sensors, length);
      else return selectrows(sensors, length, A.row(2)^(matrix::T),dimensions);  break;
    case Axis:
      if(dimensions == (X | Y | Z)) return A.convertToBuffer(sensors, length);
      else return selectrows(sensors, length, A,dimensions); break;
    }    
    return 0;
  }

}

