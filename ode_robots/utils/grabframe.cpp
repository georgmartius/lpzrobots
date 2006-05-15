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
 *                                                                         *
 *   $Log$
 *   Revision 1.3.4.2  2006-05-15 13:14:10  robot3
 *   STRG-R now makes screenshots in jpg-format
 *   STRG-F now toggles the file logging (controller stuff) on/off
 *   STRG-G now restarts the GuiLogger
 *
 *   Revision 1.3.4.1  2006/02/22 15:27:22  martius
 *   class-ified and osg-ified
 *
 *   Revision 1.3  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <assert.h>
#include <string.h>
#include <osgDB/WriteFile>
#include "grabframe.h"

namespace lpzrobots {

  // bool getGLFrameBuffer( unsigned char *buf, int w, int h);

  void VideoStream::open(const char* _filename){
    assert (_filename);
    filename = new char[strlen(_filename) + 1];
    strcpy(filename, _filename);
    counter  = 0;
    opened = true;    
  }

  void VideoStream::close(){
    if(filename) delete[] filename;
    filename = 0;
    opened   = false;
  }

  bool VideoStream::grabAndWriteFrame(const Producer::Camera& camera){
    if(!opened) return false;
    char name[128];
    osg::ref_ptr<osg::Image>image = new osg::Image; 
    int x, y; 
    camera.getProjectionRectangle(x, y, w, h); 
    image->allocateImage( w, h, 1, GL_RGB, GL_UNSIGNED_BYTE); 
    
    image->readPixels( 0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE); 
    sprintf(name,"%s_%06ld.jpg", filename, counter);
    if(!osgDB::writeImageFile( *(image.get()), name )){
      fprintf(stderr, "VideoStream: Cannot write to file %s\n", name);
      return false;
    }
    counter++;
    return true;
  }

} 

// bool getGLFrameBuffer( unsigned char *buf, int w, int h){    
//   if (!buf)
//     return false;
//   glReadPixels(0,0,w,h,GL_RGB,GL_UNSIGNED_BYTE,(GLvoid*)buf);
//   return true;
// } 

// > the best way to capture frames within OSG is to use osg::Image. One 
// > suggestion is to use a Producer::Camera postDrawCallback. Each frame, 
// > then, (to quote a current project): 
// > 
// > osg::ref_ptr<osg::Image>image = new osg::Image; 
// > int x, y; 
// > unsigned int w, h; 
// > camera.getProjectionRectangle(x,y,w,h); 
// > image->allocateImage( w, h, 1, GL_RGB, GL_UNSIGNED_BYTE); 

// > image->readPixels( 0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE); 
// > 
// > 
// > char filename[128]; 
// > sprintf( filename, "ScreenCapture/%04d.bmp", _screenCaptureSequence); 
// > osgDB::writeImageFile( *(image.get()), filename ); 
// > _screenCaptureSequence++;



//  And finally, to create mpegs, (I made mpeg1), you simply
//  use the mpeg_encode with this cfg file encode.stats:
 
//  PATTERN         ibbpbbpbbpbbpbb
 
//  SLICES_PER_FRAME        1
 
//  OUTPUT nb.mpg
 
//  GOP_SIZE        50
 
//  INPUT_DIR .
 
 
//  INPUT_CONVERT   *
 
//  INPUT
//  nb*.ppm [0000-3181]
//  END_INPUT
 
//  BASE_FILE_FORMAT PPM
 
//  ERROR           MAD
 
 
//  PIXEL HALF
 
//  # means +/- this many pixels
//  RANGE           10
 
//  PSEARCH_ALG     EXHAUSTIVE
//  BSEARCH_ALG     CROSS2
//  #IQSCALE         8
//  #PQSCALE         10
//  #BQSCALE         25
 
//  IQSCALE         26
//  PQSCALE         26
//  BQSCALE         26
 
//  REFERENCE_FRAME ORIGINAL
 
