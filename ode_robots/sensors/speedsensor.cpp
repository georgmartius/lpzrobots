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
 *   Revision 1.4  2007-11-07 13:22:18  martius
 *   scale sensor values
 *
 *   Revision 1.3  2007/08/22 08:27:15  martius
 *   removed bug with relative modes where 4 values are returned not 3
 *
 *   Revision 1.2  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.1  2006/12/21 11:41:06  martius
 *   sensor from measuring speed
 *
 *
 *                                                                 *
 ***************************************************************************/

#include <assert.h>

#include "primitive.h"
#include "speedsensor.h"
#include "mathutils.h"

using namespace matrix;

namespace lpzrobots {

  SpeedSensor::SpeedSensor(double maxSpeed, Mode mode /* = Translational */,
			   short dimensions /* = X | Y | Z */ )
    : maxSpeed(maxSpeed), mode(mode), dimensions (dimensions) {
    own=0;
  }
  
  void SpeedSensor::init(Primitive* own){
    this->own = own;
  }

  int SpeedSensor::getSensorNumber() const{
    return (dimensions & X) + ((dimensions & Y) >> 1)  + ((dimensions & Z) >> 2);    
  }
  
  bool SpeedSensor::sense(const GlobalData& globaldata) { return true; }

  std::list<sensor> SpeedSensor::get() const {
    const Matrix& m = getSenseMatrix()*(1.0/maxSpeed);
    return selectrows(m,dimensions);
  }

  int SpeedSensor::get(sensor* sensors, int length) const{
    const Matrix& m = getSenseMatrix()*(1.0/maxSpeed);
    if(dimensions == (X | Y | Z)) 
      return m.convertToBuffer(sensors, length); 
    else{
      return selectrows(sensors, length, m, dimensions);
    }
  }

  Matrix SpeedSensor::getSenseMatrix() const {
    assert(own);
    assert(own->getBody());
    Matrix local;
    Matrix m;
    switch(mode){
    case Translational:
      m.set(3,1, dBodyGetLinearVel(own->getBody()));
      break;
    case TranslationalRel:      
      local = osgMatrix2Matrixlib(own->getPose());
      m.set(4,1, dBodyGetLinearVel(own->getBody()));
      m.val(3,0)=0; // we have a vector and not a point (homogeneous coordinates)
      m=local*m;
      m.reshape(3,1);
      break;
    case Rotational:
      m.set(3,1, dBodyGetAngularVel(own->getBody()));
      break;      
    case RotationalRel:
      local = osgMatrix2Matrixlib(own->getPose());
      m.set(4,1, dBodyGetAngularVel(own->getBody()));
      m.val(3,0)=0; // we have a vector and not a point (homogeneous coordinates)
      m=local*m;  // this is m^T= m^T*local^T, ode matrix multiplications are the other way around (left sided) 
      m.reshape(3,1);
      break;
    }
    return m;
  }
  
}
