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
 *   Revision 1.1  2007-09-06 18:49:40  martius
 *   Sphere reinforcement
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __IRSENSOR_WALL_H
#define __IRSENSOR_WALL_H

#include <ode_robots/irsensor.h>

namespace lpzrobots {


  /** Class for IR sensors, that not collides with the ground.
  */
  class IRSensorWall : public IRSensor {
  public:
    /**
       @param exponent exponent of the sensor characteritic (default: 1 (linear))
    */
    IRSensorWall(float exponent, const std::list<dGeomID>& avoids);

    virtual ~IRSensorWall(){};

    virtual RaySensor* clone() const;

    virtual void init(const OdeHandle& odeHandle,
                      const OsgHandle& osgHandle,
                      Primitive* body,
                      const osg::Matrix pose, float range,
                      rayDrawMode drawMode = drawSensor);
  public:
    std::list<dGeomID> avoids;
  };

}

#endif
