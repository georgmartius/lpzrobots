/***************************************************************************
 *   Copyright (C) 2007 by Robot Group Leipzig                             *
 *    georg@nld.ds.mpg.de                                                  *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *  A camerasensor that calculates the optical flow of the image           *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#ifndef __OPTICALFLOW_H
#define __OPTICALFLOW_H

#include "camerasensor.h"
#include <selforg/controller_misc.h>

namespace lpzrobots {

  /** This CameraSensor calculates the optical flow at few points of the image
      based on a box matching technique.       
      This can be applied directly to the camera image.
   */
  class OpticalFlow : public CameraSensor {
  public:  

    struct Field {
      int x;     // middle position x
      int y;     // middle position y
      int size;  // size of field
    };
    struct Shift {
      Shift() : x(0), y(0) {}
      int x;
      int y;
    };

    /** @see CameraSensor for further parameter explanation.
        @param points number of points to calculate the flow (horizontally placed)
        @param dims dimensions to return the flow (X means horizonal, Y vertical)
     */
    OpticalFlow(Camera* camera, const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
		const osg::Matrix& pose, int points, Dimensions dims = XY );

    virtual ~OpticalFlow();

    virtual void init_sensor();
    
    /// Performs the calculations
    virtual bool sense(const GlobalData& globaldata);

    /**
       compares a small part of two given images 
       and returns the average absolute difference.
       Field center, size and shift have to be choosen, 
       so that no clipping is required
       
       \param field Field specifies position(center) and size of subimage 
       \param d_x shift in x direction
       \param d_y shift in y direction   
    */    
    double compareSubImg(unsigned char* const I1, unsigned char* const I2, 
			 const Field* field, int width, int height, int bytesPerPixel,
			 int d_x,int d_y);

    /* calculates the optimal transformation for one field in RGB 
     *   using all three color channels
     */
    Shift calcFieldTransRGB(const Field* field);
     
    virtual int getSensorNumber() const {
      return num;
    };

    virtual int get(sensor* sensors, int length) const;

  protected:
    int num;
    int points;
    Dimensions dims;
    sensor* data;
    osg::Image* last;
  }; 


}

#endif
