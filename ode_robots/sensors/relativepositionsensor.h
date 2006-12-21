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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2006-12-21 11:42:10  martius
 *   sensors have dimension to sense
 *   axissensors have finer settings
 *
 *   Revision 1.1  2006/08/08 17:03:27  martius
 *   new sensors model
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __RELATIVEPOSITIONSENSOR_H
#define __RELATIVEPOSITIONSENSOR_H

#include "sensor.h"

namespace lpzrobots {

  /** Class for relative position sensing. 
      The sensor values are the normalised relative position to some given object ( setReference() )
  */
  class RelativePositionSensor : public Sensor {
  public:  
    /**
       @param maxDistance maximal distance that is expected used for normalisation of sensor value
       @param exponent exponent of the sensor characteritic (default: 1 (linear))
       @param dimensions bit mask for the dimensions to sense. Default: X | Y | Z (all dimensions)
       @see Dimensions
     */
    RelativePositionSensor(double maxDistance, double exponent, short dimensions = X | Y | Z );
    virtual ~RelativePositionSensor() {}
  
    virtual void init(Primitive* own);  
    virtual int getSensorNumber() const;
  
    virtual bool sense(const GlobalData& globaldata);
    virtual std::list<sensor> get() const;
    virtual int get(sensor* sensors, int length) const;

    /**
       Sets the reference object we use for relative position measureing.
       This can be another robot an obstacle (light source) and such like
       This must be called before first sense() or get() call.
    */
    virtual void setReference(Primitive* ref);

  private:
    double maxDistance;
    double exponent;
    short dimensions;
    Primitive* own;
    Primitive* ref;    
  };


}

#endif
