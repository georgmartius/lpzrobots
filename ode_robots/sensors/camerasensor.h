/***************************************************************************
 *   Copyright (C) 2007 by Robot Group Leipzig                             *
 *    georg@nld.ds.mpg.de                                                  *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 ** Started on  Mon Oct 15 16:48:03 2007 Georg Martius *
 ** Last update Mon Oct 15 16:48:03 2007 Georg Martius *
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
#ifndef __CAMERASENSOR_H
#define __CAMERASENSOR_H

#include "camera.h"
#include "sensor.h"

namespace lpzrobots {


  /** This to connect a Camera as a sensor to a robot. 
      Essentially it implements the conversion from the 2D image to a list of double sensor values. 
   */
  class CameraSensor : public Sensor {
  public:  

    /** Creates a camera sensor from a camera. The camera will be initialized in init().
        @param pose position and orientation of camera wrt the primitive that is given at init()
     */ 
    CameraSensor(Camera* camera, const OdeHandle& odeHandle, 
		 const OsgHandle& osgHandle, const osg::Matrix& pose);


    virtual ~CameraSensor();

    /// this function initialized the camera (no need to overload)
    virtual void init(Primitive* own);

    /** overload this function to initialized you data structures.
        Use camera->getImage() to get the image from the camera
    */
    virtual void init_sensor() = 0;

    /// overload this function an process the image (use camera->getImage())
    virtual bool sense(const GlobalData& globaldata) = 0;

    /// overload this function and return the number of sensor values
    virtual int getSensorNumber() const = 0;

    /// overload this function and return the sensor values
    virtual int get(sensor* sensors, int length) const = 0;
    
    /// we update the camera's visual appearance
    virtual void update();

    /// this is implemented based on get(sensor*,int)
    virtual std::list<sensor> get() const;

  protected:
    Camera* camera;    
    OdeHandle odeHandle;
    OsgHandle osgHandle;
    osg::Matrix pose;
  };  

}

#endif
