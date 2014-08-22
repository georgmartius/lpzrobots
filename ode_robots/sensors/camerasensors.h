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

  struct PositionCameraSensorConf {
    typedef short Values; ///< combination of PositionCameraSensor::ValueTypes

    /// values additional sensor values, @see PositionCameraSensor::Values
    Values     values        ;
    /// dims dimensions to return the position (X means horizonal, Y vertical)
    Sensor::Dimensions dims        ;
    /** exponent for the measured size. A sqrt (0.5) make small sizes larger, so
        that changes in the distance can be measured better */
    double     sizeExponent;
    /// factor for measured size change (velocity is in framesize/frame)
    double     factorSizeChange;
    /// clipsize value at which the values are clipped, e.g. [-1.5,1.5]
    double     clipsize        ;
    /** if >0 then the size and sizechange are zero if position is that far (border) away from the image border @see PositionCameraSensor */
    double     border   ;
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

    /** The camera image should be black and white
        (e.g. @see BWImageProcessor or ColorFilterImgProc)
        @see CameraSensor for further parameter explanation.
        @param values sensor values to compute (@see PositionCameraSensor::ValueTypes)
        @param dims dimensions to return the position (X means horizonal, Y vertical)
        @param border if >0 then the size and sizechange are zero if position is that far (border) away from
        image border
     */
    PositionCameraSensor(PositionCameraSensorConf conf = getDefaultConf())
      : conf(conf), oldsize(0) {
      num = (bool(conf.dims & X) + bool(conf.dims & Y))* bool(conf.values & Position) +
        bool(conf.values & Size) + bool(conf.values & SizeChange);
      std::vector<std::string> names;
      setNamesIntern(names);
    }

    static PositionCameraSensorConf getDefaultConf(){
      PositionCameraSensorConf c;
      c.values           = Position;
      c.dims             = XY;
      c.factorSizeChange = 10.0;
      c.sizeExponent     = 1;
      c.clipsize         = 1.5;
      c.border           = 0;
      return c;
    }

    /// sets the names of the sensors and starts with the given names (for subclasses)
    virtual void setNamesIntern(std::vector<std::string>& names){
      setBaseInfo(SensorMotorInfo("CamAvg: ").changequantity(SensorMotorInfo::Other));
      if(conf.values & Position) {
        if(conf.dims & X) names.push_back("PosH");
        if(conf.dims & Y) names.push_back("PosV");
      }
      if(conf.values & Size) names.push_back("Size");
      if(conf.values & SizeChange) names.push_back("Size Change");
      setNames(names);
    }

    virtual ~PositionCameraSensor(){
      if(data) delete[] data;
    }

    virtual void intern_init(){
      assert(camera->isInitialized());

      data = new sensor[num];
      memset(data,0,sizeof(sensor)*num);

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
      return processAndFillData(x, y, size, k );
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
    /// process sensor information and fills
    //   the data buffer starting at intex k
    virtual bool processAndFillData(double& x, double& y, double& size, int& k){
      int kstart = k;
      if(conf.sizeExponent!=1)
        size = pow(size,conf.sizeExponent);

      if(conf.values & Position){
        if(conf.dims & X) data[k++] = x;
        if(conf.dims & Y) data[k++] = y;
      }
      double sizeChange = (size - oldsize)*conf.factorSizeChange;
      oldsize = size;

      // check border effects
      if(conf.border>0){
        if((x==0 && y==0) || ((conf.dims & X) && (fabs(x) > (1-conf.border))) ||
           ((conf.dims & Y) && (fabs(y) > (1-conf.border))) ){
          size=0;
          sizeChange=0;
        }
      }
      if(conf.values & Size)       data[k++] = size;
      if(conf.values & SizeChange) data[k++] = sizeChange;

      // clip values
      if(conf.clipsize!=0){
        for(int i=kstart; i<k; i++){
          data[i] = clip(data[i],-conf.clipsize,conf.clipsize);
        }
      }
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


    protected:
    PositionCameraSensorConf conf;
    int num;
    sensor* data;
    double oldsize;
  };


  struct MotionCameraSensorConf : public PositionCameraSensorConf{
    /// averaging time window (1: no averaging)
    int        avg      ;
    /// factor for measured velocity (velocity is in framesize/frame)
    double     factorMotion ;
    /// window whether to apply a windowing function to motion data to avoid edge effects
    bool       window        ;

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
        @param mconf configuration object @see MotionCameraSensorConf
         and @see PositionCameraSensorConf
     */
    MotionCameraSensor(const MotionCameraSensorConf& mconf = getDefaultConf())
      : PositionCameraSensor(mconf), mconf(mconf),
        last(false), lastX(0), lastY(0)
    {
      if(this->mconf.avg<1) this->mconf.avg=1;
      lambda = 1/(double)this->mconf.avg;
      num   += bool(this->mconf.dims & X) + bool(this->mconf.dims & Y);
      std::vector<std::string> names;
      if(mconf.dims & X) names.push_back("MotionH");
      if(mconf.dims & Y) names.push_back("MotionV");
      setNamesIntern(names);
    }

    static MotionCameraSensorConf getDefaultConf(){
      MotionCameraSensorConf c;
      c.avg              = 2;
      c.values           = None;
      c.dims             = X;
      c.factorSizeChange = 10.0;
      c.sizeExponent     = 1;
      c.clipsize         = 1.5;
      c.border           = 0;
      c.factorMotion     = 5.0;
      c.window           = true;
      return c;
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
      // check if the apparent shift is feasible, otherwise set to no motion.
      if(last && success && fabs(x - lastX) < 0.4 && fabs(y - lastY) < 0.4){
        if(mconf.dims & X) {
          data[k] = lambda*(x - lastX)*mconf.factorMotion* (mconf.window ? windowfunc(x) : 1)
            + (1- lambda)*data[k];
          k++;
        }
        if(mconf.dims & Y) {
          data[k] = lambda*(y - lastY)*mconf.factorMotion* (mconf.window ? windowfunc(y) : 1)
            + (1- lambda)*data[k]; k++;
        }
        // clip values
        if(conf.clipsize!=0){
          for(int i=0; i<k; i++){
            data[i] = clip(data[i],-mconf.clipsize,mconf.clipsize);
          }
        }
      }else{
        if(mconf.dims & X) data[k++]=0;
        if(mconf.dims & Y) data[k++]=0;
      }
      lastX = x;
      lastY = y;
      last  = success;
      // add all other sensor values
      return processAndFillData(x,y,size,k);
    }

    /// window function for the interval -1 to 1, with ramps from 0.5 off center
    double windowfunc(double x){
      if(x>-0.5 && x<0.5) return 1.0;
      if(x<= -0.5) return 2+ 2*x;
      else return 2- 2*x; // (x>0.5)
    }

  protected:
    MotionCameraSensorConf mconf;
    double lambda;
    bool   last;  ///< whether last image had a valid position
    double lastX;
    double lastY;

  };

}

#endif
