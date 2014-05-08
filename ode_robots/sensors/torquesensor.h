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
#ifndef __TORQUESENSOR_H
#define __TORQUESENSOR_H

#include "sensor.h"
#include <ode-dbl/ode.h>

namespace lpzrobots {

  class Joint;

  /** Class for sensing the torque that are applied to the joint by a motor.
      The sensor value can be interpreted as a motor current.
  */
  class TorqueSensor : public Sensor {
  public:

    /**
       @param maxtorque at this torque the sensor value is 1.
       @param avg number of averaging steps (def 1) (very noisy for universal joint)
     */
    TorqueSensor(double maxtorque = 1.0, int avg = 1);
    virtual ~TorqueSensor();

    /** the primitive is not required here, set it to NULL
        @param joint the joint on which to measure the torques.
    */
    virtual void init(Primitive* own, Joint* joint = 0);
    virtual int getSensorNumber() const;

    virtual bool sense(const GlobalData& globaldata);
    virtual std::list<sensor> getList() const;
    virtual int get(sensor* sensors, int length) const; // we implement this one because easier with averaging

  private:
    Joint* joint;
    double maxtorque;
    std::vector<sensor> values;
    double tau; // for averaging
  };


}

#endif
