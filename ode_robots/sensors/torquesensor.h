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
 ***************************************************************************/
#ifndef __TORQUESENSOR_H
#define __TORQUESENSOR_H

#include "sensor.h"
#include <ode-dbl/ode.h>

namespace lpzrobots {

  class Joint;

  /** Class for sensing the torque that are applied the joint by a motor.
      The sensor value can be interpretedas a motor current.
  */
  class TorqueSensor : public Sensor {
  public:  

    /**
       @param joint the joint on which to measure the torques.
       @param maxtorque at this torque the sensor value is 1.       
     */
    TorqueSensor(Joint* joint, double maxtorque = 1.0);
    virtual ~TorqueSensor();
    
    /// the primitive is not required here, set it to NULL 
    virtual void init(Primitive* own);
    virtual int getSensorNumber() const;
  
    virtual bool sense(const GlobalData& globaldata);
    virtual std::list<sensor> get() const;

  private:
    Joint* joint;
    double maxtorque;
    dJointFeedback* feedback;
    bool allocatedfb;
  };


}

#endif
