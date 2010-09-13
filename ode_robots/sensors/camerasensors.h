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
 *  A collection of typical camerasensor implementations.                  *
 *                                                                         *
 ***************************************************************************/

#ifndef __CAMERASENSORS_H
#define __CAMERASENSORS_H

#include "camerasensor.h"
#include <selforg/controller_misc.h>

namespace lpzrobots {


  /** This CameraSensor implements a direct conversion from pixels to sensors.
      Probably you want to use an image processor like LineImgProc before.      
   */
  class DirectCameraSensor : public CameraSensor {
  public:  

    /** the camera image should be black and white (e.g. @see BWImageProcessor)
        @see CameraSensor for further parameter explanation.
        @param minValue pixel value that corresponds to -1
        @param maxValue pixel value that corresponds to 1
        (for minValue =-256 and maxValue=256 the sensor values are in [0,1)
     */
    DirectCameraSensor(int minValue=-256, int maxValue=256)
      : minValue(minValue), maxValue(maxValue) {
    }


    virtual ~DirectCameraSensor(){
      delete[] data;
    }

    virtual void intern_init(){
      assert(camera->isInitialized());
      const osg::Image* img = camera->getImage();
      assert(img && img->getPixelFormat()==GL_LUMINANCE  && img->getDataType()==GL_UNSIGNED_BYTE);
      num = img->s() * img->t();
      data = new sensor[num]; 
    };

    
    /// Performs the calculations
    virtual bool sense(const GlobalData& globaldata){
      const osg::Image* img = camera->getImage();
      const unsigned char* pixel = img->data();
      if(img->s() * img->t() < num) return false;
      int center = (maxValue+minValue)/2;
      for(int k=0; k< num; k++){
        data[k]=(pixel[k]-center)*2.0/double(maxValue-minValue);        
}
      return true;
    }
     
    virtual int getSensorNumber() const {
      return num;
    };

    /// overload this function and return the sensor values
    virtual int get(sensor* sensors, int length) const {
      assert(length>=num);
      memcpy(sensors, data, num * sizeof(sensor));
      return num;
    }
    

  protected:
    int num;
    int minValue;
    int maxValue;
    sensor* data;
  };  


  /** This CameraSensor calculates the position of the visible object(s) that
      is essentially the center of gravity of the image.      
      The position in normalized to -1 to 1.
      Probably you want to use an image processor like ColorFilterImgProc before.      
   */
  class PositionCameraSensor : public CameraSensor {
  public:  
    /** additional sensor values. Size is the size of the object (only one value,
        independent of the dimensions */
    enum ValueTypes { None = 0, Position = 1, Size = 2, SizeChange = 4 };
    typedef short Values; ///< combination of ValueTypes
    
    /** The camera image should be black and white 
	(e.g. @see BWImageProcessor or ColorFilterImgProc)
        @see CameraSensor for further parameter explanation.
        @param values sensor values to compute (@see PositionCameraSensor::ValueTypes)
        @param dims dimensions to return the position (X means horizonal, Y vertical)
     */
    PositionCameraSensor(Values values = Position, Dimensions dims = XY )
      : dims(dims), values(values), oldsize(0) {
      num = (bool(dims & X) + bool(dims & Y))*bool(values & Position) +
        bool(values & Size) + bool(values & SizeChange);
      data = new sensor[num]; 
      memset(data,0,sizeof(sensor)*num);
    }

    virtual ~PositionCameraSensor(){
      if(data) delete[] data;
    }

    virtual void intern_init(){
      assert(camera->isInitialized());
      const osg::Image* img = camera->getImage();
      img=img; // to avoid unused variable in NDEBUG mode
      assert(img && img->getPixelFormat()==GL_LUMINANCE  && 
             img->getDataType()==GL_UNSIGNED_BYTE);
    };

    
    /// Performs the calculations
    virtual bool sense(const GlobalData& globaldata){
      const osg::Image* img = camera->getImage();
      double x;
      double y;      
      double size;      
      calcImgCOG(img, x, y, size);
      int k=0;
      if(values & Position){
        if(dims & X) data[k++] = x;
        if(dims & Y) data[k++] = y;
      }
      if(values & Size) data[k++] = size;
      if(values & SizeChange) data[k++] = (size - oldsize)*10;
      oldsize = size;
      return true;
    }

    /** calculates the Center of Gravity (normalized to -1 to 1) of an image. 
        As a bonus it also calculates the sum of all pixels (normalizes to 0-2.5) in size
	@return false if image is too dark
    */
    static bool calcImgCOG(const osg::Image* img, double& x, double& y, double& size, 
                           int threshold = 1){
      int w = img->s();
      int h = img->t();
      double centerX = w/2.0;
      double centerY = h/2.0;
      if(threshold<1) threshold = 1;
      x = y = 0;
      double sum= 0;
      for(int r=0; r< h; r++){
	const unsigned char* row = img->data(0, r);
	double pY = ((double)r-centerY);
	for(int c=0; c < w; c++){
	  sum += row[c];
	  x  += row[c] * ((double)c-centerX);
	  y  += row[c] * pY;
	}
      }
      if(sum<threshold){
	x = y = size = 0;
	return false;
      }else{
	x /= sum * centerX; // normalize to -1 to 1 
	y /= sum * centerY; // normalize to -1 to 1 
        // the /255 would be correct, but then the values is so small
        size = double(sum) / (w*h) / 128; 
	return true;
      }
    }

     
    virtual int getSensorNumber() const {
      return num;
    };

    /// overload this function and return the sensor values
    virtual int get(sensor* sensors, int length) const {
      assert(length>=num);
      memcpy(sensors, data, num * sizeof(sensor));
      return num;
    }
  protected:
    int num;
    Dimensions dims;
    Values values;
    sensor* data;
    double oldsize;
  }; 

  /** This CameraSensor calculates the global optical flow of the camera image
      using the center of gravity method. It requires objects to be bright on black ground.
      The velocity is by default windowed to avoid step-like artefacts at the border.
      Probably you want to use an image processor like ColorFilterImgProc before. 
   */
  class MotionCameraSensor : public PositionCameraSensor {
  public:  

    /** The camera image should be black and white 
	(e.g. @see BWImageProcessor or ColorFilterImgProc)
        @see CameraSensor for further parameter explanation.
        @param dims dimensions to return the position (X means horizonal, Y vertical)
        @param values additional sensor values, @see PositionCameraSensor::Values
	@param factor factor for measured velocity (velocity is in framesize/frame
	@param avg averaging time window (1: no averaging)
	@param window whether to apply a windowing function to avoid edge effects
	@param clipsize value at which the values are clipped, e.g. [-1.5,1.5] 
     */
    MotionCameraSensor(int avg = 2, Values values=None, Dimensions dims = X, 
                       double factor = 5.0, bool window = true, double clipsize = 1.5)
      : PositionCameraSensor(values, dims), factor(factor), 
	window(true), clipsize(clipsize), last(false), lastX(0), lastY(0)
    {
      lambda = 1/(double)avg;
      num += bool(dims & X) + bool(dims & Y);
      delete[] data;
      data = new sensor[num]; 
      memset(data,0,sizeof(sensor)*num); 
    }

    virtual ~MotionCameraSensor(){
    }
    
    /// Performs the calculations
    virtual bool sense(const GlobalData& globaldata){
      const osg::Image* img = camera->getImage();
      double x;
      double y;      
      double size;      
      bool success = calcImgCOG(img, x, y, size);
      int k=0;
      if(last && success){
        // check if the apparent shift is infeasible, then leave the old sensor value.
        if(fabs(x - lastX) < 0.4 && fabs(y - lastY) < 0.4) {
          if(dims & X) { 
            data[k] = lambda*(x - lastX)*factor* (window ? windowfunc(x) : 1) 
	      + (1- lambda)*data[k]; k++; 
          }
          if(dims & Y) { 
            data[k] = lambda*(y - lastY)*factor* (window ? windowfunc(y) : 1)
	      + (1- lambda)*data[k]; k++; 
          }
        }
      }else{
	if(dims & X) data[k++] = 0;
	if(dims & Y) data[k++] = 0;	
      }
      if(values & Position){
        if(dims & X) data[k++] = x;
        if(dims & Y) data[k++] = y;
      }
      if(values & Size) data[k++] = size;	
      if(values & SizeChange) data[k++] = (size - oldsize)*10;
        
      if(clipsize!=0){
	for(int i=0; i<num; i++){
	  data[i] = clip(data[i],-clipsize,clipsize);
	}
      }
	
      lastX = x;
      lastY = y;
      last  = success;
      oldsize = size;
      return true;
    }
    
    /// window function for the interval -1 to 1, with ramps from 0.5 off center
    double windowfunc(double x){
      if(x>-0.5 && x<0.5) return 1.0;
      if(x<= -0.5) return 2+ 2*x;
      else return 2- 2*x; // (x>0.5)
    }

  protected:
    double factor;
    double lambda;
    bool window;
    double clipsize;
    bool   last;  ///< whether last image had a valid position 
    double lastX;
    double lastY;

  };  

}

#endif
