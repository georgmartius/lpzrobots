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

#include "opticalflow.h"

// #include <osgDB/WriteFile>

using namespace std;
namespace lpzrobots {

  OpticalFlow::Vec2i OpticalFlow::Vec2i::operator + (const Vec2i& v) const {
    return Vec2i(x+v.x,y+v.y);
  }
  OpticalFlow::Vec2i OpticalFlow::Vec2i::operator * (int i) const {
    return Vec2i(x*i,y*i);
  }
  OpticalFlow::Vec2i OpticalFlow::Vec2i::operator / (int i) const {
    return Vec2i(x/i,y/i);
  }

  OpticalFlow::OpticalFlow(OpticalFlowConf conf)
    : conf(conf), fields(0), data(0), cnt(4), avgerror(-1) {
    num = conf.points.size() * (bool(conf.dims & X) + bool(conf.dims & Y));
    data = new sensor[num];
    memset(data,0,sizeof(sensor)*num);
    memset(lasts,0 , 4*sizeof(void*));

    if(conf.maxFlow > 0.4) conf.maxFlow=0.4;
  }

  OpticalFlow::~OpticalFlow(){
    delete[] data;
    for(int i=0; i<4; i++){
      if(lasts[i]) lasts[i]->unref();
    }
  }

  list<Pos> OpticalFlow::getDefaultPoints(int num){
    list<Pos> ps;
    if(num<=1) {
      ps += Pos(0,0,0); // only center
    }else{
      double spacing = 2.0/(num-1);
      for(int i=0; i<num; i++){
        ps += Pos(-1.0+spacing*i,0,0);
      }
    }
    return ps;
  }


  void OpticalFlow::intern_init(){
    assert(camera->isInitialized());
    const osg::Image* img = camera->getImage();
    assert(img && img->getPixelFormat()==GL_RGB  && img->getDataType()==GL_UNSIGNED_BYTE);
    width  = img->s();
    height = img->t();
    maxShiftX = int(width * conf.maxFlow);
    maxShiftY = int(height * conf.maxFlow);
    if(conf.fieldSize == 0)
      conf.fieldSize = max(width,height)/12;
    conf.fieldSize = min(conf.fieldSize, min(width, height)/4); // make sure it is not too large.
    if(conf.verbose) printf("Optical Flow (OF): Size %i, maxShiftX %i, maxShiftY %i\n",
                            conf.fieldSize, maxShiftX, maxShiftY);
    // calculate field positions
    FOREACHC(list<Pos>, conf.points, p){
      // the points are in coordinates -1 to 1
      //  so 0 should map to the center of the image and -1 and 1 to the
      //  borders plus an offset to have enought space to match the flow
      int offsetX = conf.fieldSize/2+maxShiftX + 2;
      int offsetY = conf.fieldSize/2+maxShiftY + 2;
      Vec2i field(int(offsetX + (p->x() + 1.0)/2.0*(width  - 2*offsetX)),
                  int(offsetY + (p->y() + 1.0)/2.0*(height - 2*offsetY))); // maybe round here
      fields.push_back(field);
      if(conf.verbose) printf("OF field: %3i, %3i\n", field.x, field.y);
    }

    for(int i=0; i<4; i++){
      lasts[i] = new osg::Image(*img, osg::CopyOp::DEEP_COPY_IMAGES);
    }

    oldFlows.resize(fields.size());
  };


  /// Performs the calculations
  bool OpticalFlow::sense(const GlobalData& globaldata){
    const osg::Image* img = camera->getImage();
    int k=0; // sensor buffer index
    int i=0; // field index
    // do not use quickmp here because the sense function is already in a quickmp loop.
    FOREACHC(list<Vec2i>, fields, f){
      FlowDelList flows;
      // we do optical flow detection with a cascade of delays (1,2,4)
      for(int t=1; t<=4; t*=2){
        double minerror;
        Vec2i flow = calcFieldTransRGB(*f, img, lasts[(cnt-t)%4], minerror);
        if(avgerror<0) avgerror = minerror;
        else avgerror = 0.99*avgerror + minerror*0.01;
        // if the flow is too high, then do not continue (at the end of the loop)
        bool stophere = (fabs(flow.x) > maxShiftX/2  || fabs(flow.y) > maxShiftY/2);
        // if maximum shift then prob. something is wrong (e.g. black image)
        //  so skip this value
        if (stophere && (fabs(flow.x) == maxShiftX || fabs(flow.y) == maxShiftY)){
          if(conf.verbose>1) printf("OF Warning: maximal shift (%i)\n", i);
          break;
        }
        if( minerror > avgerror*3){
          if(conf.verbose>2) printf("OF Warning: bad quality %i(%i) badness %lf\n",
                                    i, t, minerror/avgerror);
          if(stophere) break; else continue;
        }
        if(conf.verbose>3)
          printf("OF detect %i(%i): %3i,%3i (%3i,%3i) error: %lf (%lf)\n",
                 i, t, flow.x*4/t, flow.y*4/t,flow.x, flow.y,minerror, avgerror);

        flows.push_back(pair<Vec2i, int>(flow,t));
        if (stophere) break;
      }

      // we combine the flows with different delays and the old flow is always in
      Vec2i flow=oldFlows[i];
      FOREACHC(FlowDelList, flows, fl){// delay 1 is scaled by 4 and delay 4 stays
        flow = flow + (fl->first*(4/fl->second));
      }
      flow = flow/(flows.size()+1);
      //      if(conf.dims & X) data[k++] = (flow.x/(double)maxShiftX)/lenp1;
      if(conf.dims & X) data[k++] = flow.x/(double)maxShiftX;
      if(conf.dims & Y) data[k++] = flow.y/(double)maxShiftY;
      oldFlows[i] = flow; // save the flow
      i++;
    }
    // copy image to memory
    memcpy(lasts[cnt%4]->data(), img->data(), img->getImageSizeInBytes());
    cnt++;
    return true;
  }

  int OpticalFlow::get(sensor* sensors, int length) const {
    assert(length>=num);
    memcpy(sensors, data, num * sizeof(sensor));
    return num;
  }


  // this is taken from Georg's vid.stab video stabilization tool
  double OpticalFlow::compareSubImg(const unsigned char* const I1,
                                    const unsigned char* const I2,
                                    const Vec2i& field, int size,
                                    int width, int height, int bytesPerPixel,
                                    int d_x, int d_y) {
    int k, j;
    const unsigned char* p1 = NULL;
    const unsigned char* p2 = NULL;
    int s2 = size / 2;
    double sum = 0;

    p1=I1 + ((field.x - s2) + (field.y - s2)*width)*bytesPerPixel;
    p2=I2 + ((field.x - s2 + d_x) + (field.y - s2 + d_y)*width)*bytesPerPixel;
    // TODO: use some mmx or sse stuff here
    for (j = 0; j < size; j++) {
        for (k = 0; k < size * bytesPerPixel; k++) {
          sum += abs((int)*p1 - (int)*p2);
          p1++;
          p2++;
        }
        p1 += (width - size) * bytesPerPixel;
        p2 += (width - size) * bytesPerPixel;
    }
    return sum/((double) size *size* bytesPerPixel);
  }

  // this is taken from Georg's vid.stab video stabilization tool
  OpticalFlow::Vec2i OpticalFlow::calcFieldTransRGB(const Vec2i& field,
                                                    const osg::Image* current,
                                                    const osg::Image* previous,
                                                    double& minerror) const {
    Vec2i t;
    const unsigned char *I_c = current->data(), *I_p = previous->data();
    int i, j;

    minerror = 1e20;
    // check only every second step
    for (i = -maxShiftX; i <= maxShiftX; i += 2) {
      for (j=-maxShiftY; j <= maxShiftY; j += 2) {
        double error = compareSubImg(I_c, I_p, field, conf.fieldSize,
                                     width, height, 3, i, j);
        if (error < minerror) {
          minerror = error;
          t.x = i;
          t.y = j;
        }
      }
    }
    // check around the best match of above
    for (i = t.x - 1; i <= t.x + 1; i += 2) {
      for (j = -t.y - 1; j <= t.y + 1; j += 2) {
        double error = compareSubImg(I_c, I_p, field, conf.fieldSize,
                                     width, height, 3, i, j);
        if (error < minerror) {
          minerror = error;
          t.x = i;
          t.y = j;
        }
      }
    }
    return t;
  }

}
