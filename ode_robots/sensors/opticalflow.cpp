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
 **************************************************************************/
 
#include "opticalflow.h"

namespace lpzrobots {


  OpticalFlow::OpticalFlow(Camera* camera, const OdeHandle& odeHandle, 
			   const OsgHandle& osgHandle, 
			   const osg::Matrix& pose, int points, Dimensions dims)
    : CameraSensor(camera, odeHandle, osgHandle, pose), dims(dims), data(0), last(0) {
    assert(camera);
    num = points * (bool(dims & X) + bool(dims & Y));
    data = new sensor[num]; 
    memset(data,0,sizeof(sensor)*num);
  }

  OpticalFlow::~OpticalFlow(){
    delete[] data;
  }

  void OpticalFlow::init_sensor(){
    assert(camera->isInitialized());
    const osg::Image* img = camera->getImage();
    assert(img && img->getPixelFormat()==GL_RGB  && img->getDataType()==GL_UNSIGNED_BYTE);
  };

    
  /// Performs the calculations
  bool OpticalFlow::sense(const GlobalData& globaldata){
    const osg::Image* img = camera->getImage();
    if(!last){ 
      last = new osg::Image(*img, osg::CopyOp::DEEP_COPY_IMAGES);
      return true;
    }
      
      
    int k=0;
    if(dims & X) data[k++] = 1;
    if(dims & Y) data[k++] = 1;      
    // copy image to memory
    memcpy(last->data(), img->data(), img->getImageSizeInBytes());
    return true;
  }

  int OpticalFlow::get(sensor* sensors, int length) const {
    assert(length>=num);
    memcpy(sensors, data, num * sizeof(sensor));
    return num;
  }


 
  double OpticalFlow::compareSubImg(unsigned char* const I1, unsigned char* const I2, 
				    const Field* field, 
				    int width, int height, int bytesPerPixel, int d_x, int d_y)
  {
    int k, j;
    unsigned char* p1 = NULL;
    unsigned char* p2 = NULL;
    int s2 = field->size / 2;
    double sum = 0;
    
    p1=I1 + ((field->x - s2) + (field->y - s2)*width)*bytesPerPixel;
    p2=I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y)*width)*bytesPerPixel;
    // TODO: use some mmx or sse stuff here
    for (j = 0; j < field->size; j++){
        for (k = 0; k < field->size * bytesPerPixel; k++) {
	  sum += abs((int)*p1 - (int)*p2);
	  p1++;
	  p2++;     
        }
        p1 += (width - field->size) * bytesPerPixel;
        p2 += (width - field->size) * bytesPerPixel;
    }
    return sum/((double) field->size *field->size* bytesPerPixel);
  }
  
  OpticalFlow::Shift OpticalFlow::calcFieldTransRGB(const Field* field){
    Shift t;
//     uint8_t *I_c = sd->curr, *I_p = sd->prev;
//     int i, j;
  
//     double minerror = 1e20;  
//     for (i = -sd->maxshift; i <= sd->maxshift; i += 2) {
//         for (j=-sd->maxshift; j <= sd->maxshift; j += 2) {      
//             double error = compareSubImg(I_c, I_p, field, 
//                                          sd->width, sd->height, 3, i, j);
//             if (error < minerror) {
//                 minerror = error;
//                 t.x = i;
//                 t.y = j;
//             }	
//         }
//     }
//     for (i = t.x - 1; i <= t.x + 1; i += 2) {
//         for (j = -t.y - 1; j <= t.y + 1; j += 2) {
//             double error = compareSubImg(I_c, I_p, field, 
//                                          sd->width, sd->height, 3, i, j);
//             if (error < minerror) {
//                 minerror = error;
//                 t.x = i;
//                 t.y = j;
//             }	
//         }
//     }
//     if (!sd->allowmax && fabs(t.x) == sd->maxshift) {
//         t.x = 0;
//     }
//     if (!sd->allowmax && fabs(t.y) == sd->maxshift) {
//         t.y = 0;
//     }
    return t;
  }
  
}
