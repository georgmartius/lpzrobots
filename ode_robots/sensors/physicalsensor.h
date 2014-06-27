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
#ifndef __PHYSICALSENSOR_H
#define __PHYSICALSENSOR_H

#include "sensor.h"
#include "odehandle.h"
#include "osghandle.h"

namespace lpzrobots {

  /** Abstract class for sensors that have a physical representation
  */
  class PhysicalSensor : public Sensor {
  public:

    PhysicalSensor() : isInitDataSet(false) {}
    virtual ~PhysicalSensor() {}

    /** sets the initial data structures
        @param pose position and orientation of sensor (e.g. camera) wrt.
        the primitive that is given at init()
     */
    virtual void setInitData(const OdeHandle& odeHandle,
                             const OsgHandle& osgHandle,
                             const osg::Matrix& pose) {
      this->odeHandle = odeHandle;
      this->osgHandle = osgHandle;
      this->pose      = pose;
      isInitDataSet   = true;
    }

    /// changes the relative pose of the sensor
    virtual void setPose(const osg::Matrix& pose) { this->pose= pose; };

    /// relative pose of the sensor
    virtual osg::Matrix getPose() { return pose; };


  protected:
    OdeHandle odeHandle;
    OsgHandle osgHandle;
    osg::Matrix pose;
    bool isInitDataSet;
  };
}

#endif
