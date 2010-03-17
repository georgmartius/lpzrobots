/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
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
 *
 *******************************************`********************************/
#ifndef __IMAGEPROCESSORS
#define __IMAGEPROCESSORS

#include "imageprocessor.h"

#define MIN3(x,y,z) x<y ? (x<z ? x : z) : (y<z ? y : z)
#define MAX3(x,y,z) x>y ? (x>z ? x : z) : (y>z ? y : z)

namespace lpzrobots {

  /** Standard image processor - convenience class for 1 to 1 image processing. 
      The last image of the stack is the source (output of last processor).
      Simpler to implement than ImageProcessor.
      @see ImageProcessor
   */
  struct StdImageProcessor : public ImageProcessor {
    StdImageProcessor(bool show, float scale) {
      _dest.show  = show;
      _dest.scale = scale;
    };
    virtual ~StdImageProcessor() {};

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
    BWImageProcessor(bool show, float scale)
      : StdImageProcessor(show,scale) {}

    virtual ~BWImageProcessor() {}
    
    virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src){
      dest.img->allocateImage(src.img->s(), src.img->t(), 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);    
      dest.name  = src.name + "_bw"; 
    }
    
    virtual void process(const osg::Image* src, osg::Image* dest){      
      assert(src && src->getPixelFormat()==GL_RGB && src->getDataType()==GL_UNSIGNED_BYTE);
      for(int r=0; r < src->t(); ++r) {
        const unsigned char* sdata = src->data(0, r);
        unsigned char* ddata = dest->data(0, r);
        for(int c=0; c < src->s(); ++c) {
          (*ddata) =  (*(sdata) + *(sdata+1) + *(sdata+2))/3;
          sdata+=3;
          ddata++;
        }
      }
    }
  };

  /// converts the image to a HSV coded image @see StdImageProcessor
  struct HSVImgProc : public StdImageProcessor {

    HSVImgProc(bool show, float scale)
      : StdImageProcessor(show,scale) {
      sat_Threshold = val_Threshold = 50;
    }

    virtual ~HSVImgProc() {}
    
    virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src){
      dest.img->allocateImage(src.img->s(), src.img->t(), 1, GL_RGB, GL_UNSIGNED_BYTE);    
      dest.name  = src.name + "_hsv"; 
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

    // Todo: make hue continuous
    /** converts RGB to HSV color model
        r,g,b values are from 0 to 255
        h = [0,6], s = [0,255], v = [0,255]
        0-6 -> colors 6: gray
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
      if( max != 0 ) {
        s = int(float(delta) / float(max)*255);               // s
        if (s<sat_Threshold){
          h = 6; //gray
          return;
        }
      } else {
        // r = g = b = 0                // s = 0, v is undefined
        s = 0;
        h = 6; // gray
        return;
      }
      if (v<val_Threshold) {
        h = 6; // gray
        return;
      }
      if( r == max )
        hue = float( g - b ) / delta;        // between yellow & magenta
      else if( g == max )
        hue = 2.0 + float( b - r ) / delta;     // between cyan & yellow
      else
        hue = 4.0 + float( r - g ) / delta;     // between magenta & cyan
      if( hue < 0 )
        hue += 6;
      h= char(hue+0.5);
    }

    int sat_Threshold;
    int val_Threshold;

  };


  /** filters for a specific color (requires HSV, so use HSVImgProc before) 
      @see HSVImgProc
      @see StdImageProcessor
  */
  struct ColorFilterImgProc : public StdImageProcessor {
    ColorFilterImgProc(bool show, float scale, int hue)
      : StdImageProcessor(show,scale), hue(hue) {
    }
    
    virtual ~ColorFilterImgProc() {}
    
    virtual void initDestImage(Camera::CameraImage& dest, const Camera::CameraImage& src){
      dest.img->allocateImage(src.img->s(), src.img->t(), 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);    
          //      dest.img->allocateImage(16, 1, 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);    
      dest.name  = src.name + "_spots"; 
    }
    
    virtual void process(const osg::Image* src, osg::Image* dest){      
      // actually we need HSV but there is no coding for it
      assert(src && src->getPixelFormat()==GL_RGB && src->getDataType()==GL_UNSIGNED_BYTE);
      for(int r=0; r < src->t(); ++r) {
        const unsigned char* sdata = src->data(0, r);
        unsigned char* ddata = dest->data(0, r);
        for(int c=0; c < src->s(); ++c) {
          if(*(sdata) == hue){
            (*ddata) = *(sdata+2);
          } else{
            (*ddata) = 0;
          }
          sdata+=3;
          ddata++;
        }
      }
    }
    int hue;
  };

}

#endif
