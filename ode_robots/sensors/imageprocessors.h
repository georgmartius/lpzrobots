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
#ifndef __IMAGEPROCESSORS
#define __IMAGEPROCESSORS

#include "imageprocessor.h"

#include <selforg/stl_adds.h>

#define MIN3(x,y,z) x<y ? (x<z ? x : z) : (y<z ? y : z)
#define MAX3(x,y,z) x>y ? (x>z ? x : z) : (y>z ? y : z)

namespace lpzrobots {

  /** Standard image processor - convenience class for 1 to 1 image processing.
      The last image of the stack is the source (output of last processor).
      Simpler to implement than ImageProcessor.
      @param show whether do display the resulting image on the screen
      @param scale how to scale the display
      @see ImageProcessor
   */
  struct StdImageProcessor : public ImageProcessor {
    StdImageProcessor(bool show, float scale) {
      _dest.show  = show;
      _dest.scale = scale;
    };
    virtual ~StdImageProcessor() {
    };

    /// overload this function and initialise the dest.img and the dest.name
    virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src) = 0;

    /// overload this function and do processing here
    virtual void process(const osg::Image* src, osg::Image* dest) = 0;

    /*   substituded generic interface */
    virtual Camera::CameraImage init(const Camera::CameraImages& imgs){
      assert(imgs.size()>0);
      _src = imgs.back();
      // printf("source picture: %s, %i x %i\n",src.name.c_str(), src.img->s(), src.img->t());
      _dest.img = new osg::Image;
      initDestImage(_dest, _src);
      return _dest;
    }

    virtual void process(){
      process(_src.img, _dest.img);
      _dest.img->dirty();
    }

  protected:
    Camera::CameraImage _dest;
    Camera::CameraImage _src;
  };



  /// black and white image @see StdImageProcessor
  struct BWImageProcessor : public StdImageProcessor {
    enum ChannelMask {Red = 1, Green = 2, Blue = 4, Hue = 1, Saturation = 2, Value = 4};

    /// @param channelmask which channels to consider, @see BWImageProcessor::ChannelMask
    BWImageProcessor(bool show, float scale, char channelmask = 7)
      : StdImageProcessor(show,scale), channelmask(channelmask) {}

    virtual ~BWImageProcessor() {}

    virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src){
      dest.img->allocateImage(src.img->s(), src.img->t(), 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);
      dest.name  = "bw(" + src.name + ")";
      red   = (channelmask & 1);
      green = (channelmask & 2);
      blue  = (channelmask & 4);
      numchannels = red+green+blue;
      printf("BWImageProcessor: Select: Red %i, Green %i, Blue %i : numchannels %i\n",
             red, green, blue,numchannels);
      if(numchannels==0) numchannels=1; // avoid division by 0
    }

    virtual void process(const osg::Image* src, osg::Image* dest){
      assert(src && src->getPixelFormat()==GL_RGB && src->getDataType()==GL_UNSIGNED_BYTE);
      for(int r=0; r < src->t(); ++r) {
        const unsigned char* sdata = src->data(0, r);
        unsigned char* ddata = dest->data(0, r);
        for(int c=0; c < src->s(); ++c) {
          (*ddata) =  (*(sdata)*red + *(sdata+1)*green + *(sdata+2)*blue)/numchannels;
          sdata+=3;
          ddata++;
        }
      }
    }
    bool red, green, blue;
    char numchannels;
    char channelmask;
  };

  /** converts the image to a HSV coded image @see StdImageProcessor.
      If this image is shown the h,s,v is displayed by r,g,b!
      The h (hue) values are given by HSVImgProc::Colors
  */

  struct HSVImgProc : public StdImageProcessor {
    enum Colors {Red=0, Yellow=30, Green=60, Cyan=90,
                 Blue=120, Magenta=150, Red2=180, Gray=255, Span=30};

    HSVImgProc(bool show, float scale)
      : StdImageProcessor(show,scale) {
    }

    virtual ~HSVImgProc() {}

    virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src){
      dest.img->allocateImage(src.img->s(), src.img->t(), 1, GL_RGB, GL_UNSIGNED_BYTE);
      dest.name  = "hsv(" + src.name + ")";
    }

    virtual void process(const osg::Image* src, osg::Image* dest){
      assert(src && src->getPixelFormat()==GL_RGB && src->getDataType()==GL_UNSIGNED_BYTE);
      for(int r=0; r < src->t(); ++r) {
        const unsigned char* sdata = src->data(0, r);
        unsigned char* ddata = dest->data(0, r);
        for(int c=0; c < src->s(); ++c) {
          RGBtoHSV(*(sdata),*(sdata+1),*(sdata+2),
                   (*ddata), *(ddata+1), *(ddata+2));
          sdata+=3;
          ddata+=3;
        }
      }
    }

    /** converts RGB to HSV color model;
        r,g,b values are from 0 to 255;
        h = [0,180]+255, s = [0,255], v = [0,255];
        h is the standard hue value/2 (since 360 cannot be represented)
        and 255 if undefined (gray)
    */
    void RGBtoHSV( unsigned char r, unsigned char g, unsigned char b,
                   unsigned char& h, unsigned char& s, unsigned char& v ) {
      unsigned char min, max;
      float delta;
      float hue;
      min = MIN3( r, g, b );
      max = MAX3( r, g, b );
      v = max;                               // v
      delta = max - min;
      if( max != 0 ){
        s = (unsigned char)(255.0*delta / max); // s
      }
      if( max == 0 || delta == 0){
        // r = g = b                             // s = 0, h is undefined
        s = 0;
        h = 255;
        return;
      }

      if( r == max )
        hue = float( g - b ) / delta;        // between yellow & magenta
      else if( g == max )
        hue = 2.0 + float( b - r ) / delta;     // between cyan & yellow
      else
        hue = 4.0 + float( r - g ) / delta;     // between magenta & cyan
      hue *=30; // this is 60 in full range
      if( hue < 0 )
        hue += 180;
      h = int(hue);
    }
  };


  /** filters for a specific color (requires HSV, so use HSVImgProc before)
      @param minhue minimal hue value to pass through @see HSVImgProc::Colors
      @param maxhue maximal hue value to pass through @see HSVImgProc::Colors
      @param satThreshold minimal saturation required to be considered as a color
      @param valThreshold minimal "value" required to be considered as a color
      @see HSVImgProc
      @see StdImageProcessor
  */
  struct ColorFilterImgProc : public StdImageProcessor {
    ColorFilterImgProc(bool show, float scale, int minhue, int maxhue,
                       int sat_threshold=100, int val_threshold=50)
      : StdImageProcessor(show,scale),
        minhue(minhue), maxhue(maxhue), sat_threshold(sat_threshold), val_threshold(val_threshold) {
    }

    virtual ~ColorFilterImgProc() {}

    virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src){
      dest.img->allocateImage(src.img->s(), src.img->t(), 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);
          //      dest.img->allocateImage(16, 1, 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);
      dest.name  = "spots(" + src.name + ")";
    }

    virtual void process(const osg::Image* src, osg::Image* dest){
      // actually we need HSV but there is no coding for it
      assert(src && src->getPixelFormat()==GL_RGB && src->getDataType()==GL_UNSIGNED_BYTE);
      for(int r=0; r < src->t(); ++r) {
        const unsigned char* sdata = src->data(0, r);
        unsigned char* ddata = dest->data(0, r);
        for(int c=0; c < src->s(); ++c) {
          if(*(sdata) >= minhue && *(sdata) < maxhue
             && *(sdata+1) > sat_threshold && *(sdata+2) > val_threshold){
            (*ddata) = *(sdata+2);
          } else{
            (*ddata) = 0;
          }
          sdata+=3;
          ddata++;
        }
      }
    }
    int minhue;
    int maxhue;
    int sat_threshold;
    int val_threshold;
  };



  /** creates a lightsensitive sensorline. It requires a black and white source,
      e.g. provided by BWImageProcessor, ColorFilterImgProc
      @param num number of segments of the sensor line
      @param factor factor for average pixel value (rescaling)
      @see StdImageProcessor
  */
  struct LineImgProc : public StdImageProcessor {
    LineImgProc(bool show, float scale, int num, double factor = 20.0)
      : StdImageProcessor(show,scale), num(num), factor(factor) {
    }

    virtual ~LineImgProc() {}

    virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src){
      dest.img->allocateImage(num, 1, 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);
      dest.name  = "line(" + src.name + ")";
    }

    virtual void process(const osg::Image* src, osg::Image* dest){
      // actually we need HSV but there is no coding for it
      assert(src && src->getPixelFormat()==GL_LUMINANCE  && src->getDataType()==GL_UNSIGNED_BYTE);

      int w = src->s();
      int h = src->t();
      int size = w/num; // size of one segment
      int numpixel_per_segm = size*h;
      int segmentvalue;
      unsigned char* destdata = dest->data();
      for(int k=0; k<num; k++){
        int sum = 0;
        for(int j=0; j<h; j++){
          const unsigned char* pixel = src->data(k*size, j);
          for(int i=0; i< size; i++){
            sum += *pixel;
            pixel++;
          }
        }
        segmentvalue = int((double)sum*factor/(double)numpixel_per_segm);
        destdata[k] = std::min(segmentvalue,255);
      }
    }
    int num;
    double factor;
  };

  /** time average of image @see StdImageProcessor.
  */

  struct AvgImgProc : public StdImageProcessor {

    /// @param time length of averageing (time=1: no averaging)
    AvgImgProc(bool show, float scale, int time)
      : StdImageProcessor(show,scale), time(time) {
      if(time<1) time =1;
      factor = 1.0/(float)time;
    }

    virtual ~AvgImgProc() {}

    virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src){
      assert(src.img && src.img->getDataType()==GL_UNSIGNED_BYTE);
      dest.img->allocateImage(src.img->s(), src.img->t(), 1, src.img->getPixelFormat(),
                              GL_UNSIGNED_BYTE);
      dest.name  = "avg(" + std::itos(time) +  "," + src.name + ")";
    }

    virtual void process(const osg::Image* src, osg::Image* dest){
      for(int r=0; r < src->t(); ++r) {
        const unsigned char* sdata = src->data(0, r);
        unsigned char* ddata = dest->data(0, r);
        for(unsigned int c=0; c < src->getRowSizeInBytes(); ++c) {
          *ddata = (unsigned char)(((float)*sdata)*factor + ((float)*ddata)*(1-factor));
          sdata++;
          ddata++;
        }
      }
    }

    int time;
    float factor;
  };


//   /** Testing code
//   */
//   struct TestLineImgProc : public StdImageProcessor {
//     TestLineImgProc(bool show, float scale, int num)
//       : StdImageProcessor(show,scale), num(num) {
//     }

//     virtual ~TestLineImgProc() {}

//     virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src){
//       dest.img->allocateImage(src.img->s(), src.img->t(), 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);
//       dest.name  = "testline(" + src.name + ")";
//     }

//     virtual void process(const osg::Image* src, osg::Image* dest){
//       // actually we need HSV but there is no coding for it
//       assert(src && src->getPixelFormat()==GL_LUMINANCE  && src->getDataType()==GL_UNSIGNED_BYTE);

//       int w = src->s();
//       int h = src->t();
//       int size = w/num; // size of one segment
//       int numpixel_per_segm = size*h;
//       for(int k=0; k<num; k++){
//         int sum = 0;
//         for(int j=0; j<h; j++){
//           const unsigned char* pixel = src->data(k*size, j);
//           unsigned char* destdata = dest->data(k*size, j);
//           for(int i=0; i< size; i++){
//             *destdata= (k*111+*pixel)%256;
//             pixel++;
//             destdata++;
//           }
//         }
//       }
//     }
//     int num;
//   };




}

#endif
