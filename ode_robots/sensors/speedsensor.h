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
 *   Revision 1.2  2007-06-08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.1  2006/12/21 11:41:12  martius
 *   sensor from measuring speed
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __SPEEDSENSOR_H
#define __SPEEDSENSOR_H

#include "sensor.h"

namespace lpzrobots {

  /** Class for speed sensing of robots. 
      The sensor values are normalised speeds, either translational or rotational
  */
  class SpeedSensor : public Sensor {
  public:  

    /// Sensor mode 
    enum Mode { Translational,     ///< measures translational speed in world coordinates (Linear velocity)
		TranslationalRel,  ///< measures translational speed in body coordinates (Linear velocity)
		Rotational,        ///< measures roational velocity around the world axis
		RotationalRel      ///< measures roational velocity around the body axis
    };

    /**
       @param maxSpeed maximal speed that is expected used for normalisation of sensor value
       @param dimensions bit mask for the dimensions to sense. Default: X | Y | Z (all dimensions)
       @see Dimensions
     */
    SpeedSensor(double maxSpeed, Mode mode = Translational, short dimensions = X | Y | Z );
    virtual ~SpeedSensor() {}
  
    virtual void init(Primitive* own);  
    virtual int getSensorNumber() const;
  
    virtual bool sense(const GlobalData& globaldata);
    virtual std::list<sensor> get() const;
    virtual int get(sensor* sensors, int length) const;
  protected:
    matrix::Matrix getSenseMatrix() const;

  protected:
    double maxSpeed;
    Mode mode;
    short dimensions;
    Primitive* own;
  };


}

#endif
