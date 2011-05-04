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
 *   Revision 1.4  2011-05-04 10:59:23  fhesse
 *   constructor has additional bool to transform output of getSensors to local
 *   coordinates; default=false to be compatible
 *
 *   Revision 1.3  2007/07/03 13:13:57  martius
 *   *** empty log message ***
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

#include <assert.h>

#include "primitive.h"
#include "relativepositionsensor.h"
#include "mathutils.h"

namespace lpzrobots {

  RelativePositionSensor::RelativePositionSensor(double maxDistance, double exponent, 
						 short dimensions /* = X | Y | Z */ , bool local_coordinates /*= false*/)
    : maxDistance(maxDistance), exponent(exponent), dimensions (dimensions), local_coords(local_coordinates){
    own=0;
    ref=0;
  }
  
  void RelativePositionSensor::init(Primitive* own){
    this->own = own;
  }
  void RelativePositionSensor::setReference(Primitive* ref){
    this->ref = ref;
  }

  int RelativePositionSensor::getSensorNumber() const{
    return (dimensions & X) + ((dimensions & Y) >> 1)  + ((dimensions & Z) >> 2);
  }
  
  bool RelativePositionSensor::sense(const GlobalData& globaldata) { return true; }

  std::list<sensor> RelativePositionSensor::get() const {
    assert(ref);    assert(own);
    std::list<sensor> s;
    osg::Vec3 v = ref->getPosition() - own->getPosition();

    if (local_coords){
    	// start local coordinates -----------------------------------
    	matrix::Matrix local = osgMatrix2Matrixlib(own->getPose());
    	matrix::Matrix m;
    	m.set(4,1, 0);
    	for (int i=0; i<3; i++){
    		m.val(i,0)=v[i];
    	}
    	m.val(3,0)=0; // we have a vector and not a point (homogeneous coordinates)
    	m=local*m;
    	for (int i=0; i<3; i++){
    		v[i]=m.val(i,0);
    	}
        // end local coordinates   -----------------------------------
    }

    double scale = pow(v.length() / maxDistance, exponent);
    v *= (1/maxDistance)*scale;

    if (dimensions & X) s.push_back(v.x());
    if (dimensions & Y) s.push_back(v.y());
    if (dimensions & Z) s.push_back(v.z());
    return s;    
  }

  int RelativePositionSensor::get(sensor* sensors, int length) const{
    assert(ref);    assert(own); 
    int i = 0;
    assert ( length >= getSensorNumber() );
    osg::Vec3 v = ref->getPosition() - own->getPosition();
    if (local_coords){
    	// start local coordinates -----------------------------------
    	matrix::Matrix local = osgMatrix2Matrixlib(own->getPose());
    	matrix::Matrix m;
    	m.set(4,1, 0);
    	for (int i=0; i<3; i++){
    		m.val(i,0)=v[i];
    	}
    	m.val(3,0)=0; // we have a vector and not a point (homogeneous coordinates)
    	m=local*m;
    	for (int i=0; i<3; i++){
    		v[i]=m.val(i,0);
    	}
        // end local coordinates   -----------------------------------
    }

    double scale = pow(v.length() / maxDistance, exponent);
    v *= (1/maxDistance)*scale;
    if (dimensions & X) sensors[i++] = v.x();
    if (dimensions & Y) sensors[i++] = v.y();
    if (dimensions & Z) sensors[i++] = v.z();
    return i;    
  }
  
}
