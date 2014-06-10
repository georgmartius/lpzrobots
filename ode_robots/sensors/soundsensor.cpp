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

#include <osg/Matrix>
#include <selforg/controller_misc.h>
#include "primitive.h"
#include "mathutils.h"
#include "soundsensor.h"

namespace lpzrobots {

  SoundSensor::SoundSensor(Dimensions dim, Measure measure/*=Angle*/,
                           int segments/*=1*/, int levels/*=1*/, double maxDistance/*=50*/,
                           double noisestrength)
    : dim(dim), measure(measure),
      segments(segments), levels(levels), maxDistance(maxDistance), noisestrength(noisestrength)
  {
    int len = getSensorNumber();
    val = new double[len];
    memset(val,0,sizeof(double)*len);
    oldangle = new double[levels];
    memset(oldangle,0,sizeof(double)*levels);
    setBaseInfo(SensorMotorInfo("Sound").changequantity(SensorMotorInfo::Other));
  }

  SoundSensor::~SoundSensor() {
    if(val) delete[] val;
    if(oldangle) delete[] oldangle;
  }

  float SoundSensor::distanceDependency(const Sound& s, double distance){
    return (1-clip(distance/maxDistance,0.0,1.0)) * s.intensity;
  }

  bool SoundSensor::sense(const GlobalData& globaldata){
    int len = getSensorNumber();
    memset(val,0,sizeof(double)*len);
    int *cnt = new int[len];
    memset(cnt,0,sizeof(int)*len);

    if(!globaldata.sounds.empty()){
      // multiple signal are simply averaged combined
      FOREACHC(SoundList, globaldata.sounds, s){
        Pos relpos = own->toLocal(s->pos);
        // we have to look at the right dimension because sometimes the
        // robot's coordinate system has Z not looking to the sky
        double x = (dim == X) ? relpos.z() : relpos.x();
        double y = (dim == Y) ? relpos.z() : relpos.y();

        float dist = relpos.length();
        // close enough and not from us.
        if(dist<maxDistance && s->sender != (void*)own){
          int l = clip((int)(s->frequency/2.0+0.5)*levels,0,levels-1);
          // normalise
          double len = sqrt(x*x + y*y);
          if(len>0){ x/=len, y/=len; }

          double angle = atan2(y, x);
          double intens = distanceDependency(*s, dist);
          if(intens<=0) continue;
          // add noise to angle, the more the lower the intensity maximal noisestrength*360Deg
          angle += random_minusone_to_one(0)*2*M_PI*(1-pow(intens,0.25))*noisestrength;
          intens += random_minusone_to_one(0)*noisestrength;
          intens=clip(intens,0.0,1.0);
          switch (measure){
          case Segments:
            {
              int segm = clip((int)((angle+M_PI)/(2*M_PI)*segments),0,segments-1);
              val[segm*levels+l]= intens;
              cnt[segm*levels+l]++;
            }
            break;
          case Angle:
            val[3*l]   += intens;
            val[3*l+1] += sin(angle);
            val[3*l+2] += cos(angle);
            cnt[3*l]++; cnt[3*l+1]++; cnt[3*l+2]++;
            break;
          case AngleVel:
            {   // calc derivatives of angle values
              double d = angle - oldangle[l];
              double scale = 10;
              if(d>M_PI)  d-=2*M_PI;
              if(d<-M_PI)  d+=2*M_PI;

              oldangle[l]= angle;
              val[2*l]   = intens;
              val[2*l+1] = scale*d;
              cnt[3*l]++; cnt[3*l+1]++;
            }
            break;
          }
        }
      }
    }
    for(int k=0; k<len; k++){
      if(cnt[k]>0) val[k]/=cnt[k];
    }
    delete[] cnt;
    return true;
  }

  int SoundSensor::getSensorNumber() const{
    switch(measure){
    case Segments:
      return segments*levels;
    case Angle:
      return 3*levels; // for each level, angle (sin,cos) and intensity
    case AngleVel:
      return 2*levels; // for each level, angle derivative and intensity
    }
    return 0;
  }

  std::list<sensor> SoundSensor::getList() const{
    int len = getSensorNumber();
    std::list<sensor> s;
    for(int i=0; i<len; i++){
      s.push_back(val[i]);
    }
    return s;
  }

  int SoundSensor::get(sensor* sensors, int length) const {
    int len = std::min(getSensorNumber(),length);
    memcpy(sensors, val, sizeof(sensor)*len);
    return len;
  }

}
