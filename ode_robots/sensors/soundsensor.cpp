/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    georg@nld.ds.mpg.de                                                  *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2010-09-17 10:08:42  martius
 *   ir sensors did not work properly because of bug in osgprimitves. Resolved now
 *   Soundsensor: some comments added (really unfinished stuff)
 *
 *   Revision 1.1  2007/11/07 13:22:59  martius
 *   new sound sensor
 *
 *                                                                 *
 ***************************************************************************/

#include <osg/Matrix>
#include <selforg/controller_misc.h>
#include "primitive.h"
#include "mathutils.h"
#include "soundsensor.h"

namespace lpzrobots {

  SoundSensor::SoundSensor(Dimensions dim, Measure measure/*=Angle*/, 
			   int segments/*=1*/, int levels/*=1*/, float maxDistance/*=1000*/) 
    : dim(dim), measure(measure), 
      segments(segments), levels(levels), maxDistance(maxDistance)
  {
    int len = getSensorNumber();
    val = new double[len];
    memset(val,0,sizeof(double)*len);   
    oldangle = new double[levels];
    memset(oldangle,0,sizeof(double)*levels);   
  }

  SoundSensor::~SoundSensor() {
    if(val) delete[] val;
    if(oldangle) delete[] oldangle;
  }
  
  bool SoundSensor::sense(const GlobalData& globaldata){    
    int len = getSensorNumber();
    memset(val,0,sizeof(double)*len);
    if(!globaldata.sounds.empty()){      
      // todo combine more then one signal
      FOREACHC(SoundList, globaldata.sounds, s){	
	Pos relpos = own->toLocal(s->pos);
	// we have to look at the right dimension because sometimes the
	// robot's coordinate system is has Z not looking to the sky
	double x = (dim == X) ? relpos.z() : relpos.x();
	double y = (dim == Y) ? relpos.z() : relpos.y();
	  
	float dist = relpos.length();
	// close enough and not from us.
	if(dist<maxDistance && s->sender != (void*)own){ 
	  int l = clip((int)(s->frequency/2.0+0.5)*levels,0,levels-1);
	  // normalise
	  double len = sqrt(x*x + y*y);
	  if(len>0){ x/=len, y/=len; }

          //Todo: if the internsity is very low, then we should not be able to detect the direction!
	  double angle = atan2(y, x);	 
	  
	  switch (measure){
	  case Segments:
	    {
	      int segm = clip((int)((angle+M_PI)/(2*M_PI)*segments),0,segments-1);
	      val[segm*levels+l]= s->intensity; // todo: add distance dependance
	    }
	    break;
	  case Angle:
	    val[3*l]   = s->intensity; // todo: add distance dependance	  	        
	    val[3*l+1] = sin(angle);
	    val[3*l+2] = cos(angle);
	    break;
	  case AngleVel:
	    {   // calc derivatives of angle values
	      double d = angle - oldangle[l];
	      double scale = 10;
	      // if(d>2*M_PI/scale)  d=0;
	      // if(d<-2*M_PI/scale) d=0;
	      if(d>M_PI)  d-=2*M_PI;
	      if(d<-M_PI)  d+=2*M_PI;
	      
	      oldangle[l]=angle;	      
	      val[3*l]   = s->intensity; // todo: add distance dependance	  	        
	      val[3*l+1] = scale*d;
	    }
	    break;
	  }
	}
      }
    }
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
  
  std::list<sensor> SoundSensor::get() const{
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
