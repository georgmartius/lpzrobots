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

#ifndef __OPTICALFLOW_H
#define __OPTICALFLOW_H

#include "camerasensor.h"
#include <selforg/controller_misc.h>

namespace lpzrobots {

  /// configuration object for OpticalFlow
  struct OpticalFlowConf {
    /// dimensions to return the flow (X means horizonal, Y vertical)
    Sensor::Dimensions dims;
    /** points to measure optical flow in normalized coordinates [-1,1]
        however the points are placed sufficiently away from the border to have no
        boundary effects.*/
    std::list<Pos> points;
    /** maximum fraction of the image dimension to consider for 
        a possible flow. Larger values increase the computational demand and move the
        points more to the center (since we don't want edge effects)*/
    double maxFlow; 
    /** size (edge length) of the measurement field (block) in pixel 
        (if 0 then 1/12th of width) */
    int fieldSize; 
    /** verbosity level 
        (0: quite, 1: initialization values, 2: warnings, 3: info, 4: debug) */
    int verbose; 
  };

  /** This CameraSensor calculates the optical flow at few points of the image
      based on a box matching technique.       
      This can be applied directly to the camera image.
   */
  class OpticalFlow : public CameraSensor {
  public:  

    struct Vec2i {
      Vec2i() : x(0), y(0) {}
      Vec2i(int x, int y) : x(x), y(y) {}
      int x;
      int y;
      Vec2i operator + (const Vec2i& v) const;
      Vec2i operator * (int i) const;
      Vec2i operator / (int i) const;
    };

    typedef std::list< std::pair<Vec2i,int> > FlowDelList;

    /** @see CameraSensor for further parameter explanation.
     */
    OpticalFlow(OpticalFlowConf conf = getDefaultConf());

    virtual ~OpticalFlow();

    /** calculates default positions for optical flow detection.
        The points are in aranged horizontally in a line at the vertical center.
        For num 2 the points are at the border, 
        3 points there is additioanlly one is the center and so on.      
     */
    static std::list<Pos> getDefaultPoints(int num);

    /// the default config has 2 points in and calculates the flow in X and Y
    static OpticalFlowConf getDefaultConf(){
      OpticalFlowConf c;
      c.dims    = XY;
      c.points  = getDefaultPoints(2);
      c.fieldSize = 24; 
      c.maxFlow = 0.15;
      c.verbose = 1;
      return c;
    }

    virtual void intern_init();
    
    /// Performs the calculations
    virtual bool sense(const GlobalData& globaldata);
    
    virtual int getSensorNumber() const {
      return num;
    };

    virtual int get(sensor* sensors, int length) const;


  protected:
    /**
       compares a small part of two given images 
       and returns the average absolute difference.
       Field center, size and shift have to be choosen, 
       so that no clipping is required!
       \param field specifies position(center) of subimage to use for comparison
       \param size specifies the size of the field edged in pixels
       \param d_x shift in x direction
       \param d_y shift in y direction   
    */    
    static double compareSubImg(const unsigned char* const I1, 
                                const unsigned char* const I2, 
                                const Vec2i& field, int size, int width, int height, 
                                int bytesPerPixel, int d_x,int d_y);

    /** calculates the optimal transformation for one field in RGB 
     *   using all three color channels
     * @param minerror (is to return the minimum error achieved during the matching)
     */
    Vec2i calcFieldTransRGB(const Vec2i& field, const osg::Image* current, 
                            const osg::Image* last, double& minerror) const;

  protected:
    OpticalFlowConf conf;
    int num;
    std::list<Vec2i> fields; // fields in image coordinates
    sensor* data;
    osg::Image* lasts[4];
    std::vector<Vec2i> oldFlows;
    int maxShiftX;
    int maxShiftY;
    int width;
    int height;
    int cnt;
    double avgerror; // average minimum matching error
  }; 


}

#endif
