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
 *   Revision 1.12.4.5  2006-03-28 09:55:12  robot3
 *   -main: fixed snake explosion bug
 *   -odeconfig.h: inserted cameraspeed
 *   -camermanipulator.cpp: fixed setbyMatrix,
 *    updateFactor
 *
 *   Revision 1.12.4.4  2006/02/08 16:14:28  martius
 *   no namespace using
 *
 *   Revision 1.12.4.3  2006/01/10 15:08:15  martius
 *   controlinterval is 1 by default
 *
 *   Revision 1.12.4.2  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.12.4.1  2005/11/14 17:37:00  martius
 *   changed makefile structure to have and include directory
 *   mode to selforg
 *
 *   Revision 1.12  2005/11/09 13:29:19  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef __ODECONFIG_H
#define __ODECONFIG_H

#include <ode/ode.h>
#include <selforg/configurable.h>
#include "odehandle.h"

namespace lpzrobots {

class OdeConfig : public Configurable {
public:
  OdeConfig() :
    name ("Simulation Environment: ")
  {
    simStepSize=0.01;
    controlInterval=1;
    realTimeFactor=1.0;
    noise=0.1;
    gravity=-9.81;
    drawInterval=calcDrawInterval();
    cameraSpeed=100;
      // prepare name;
    Configurable::insertCVSInfo(name, "$RCSfile$", "$Revision$");
  }

  virtual ~OdeConfig(){}

  virtual paramkey getName() const {
    return name;
  }

  virtual paramlist getParamList() const{
    paramlist list;
    list.push_back(std::pair<paramkey, paramval> (std::string("noise"), noise));
    list.push_back(std::pair<paramkey, paramval> (std::string("simstepsize"), simStepSize));
    list.push_back(std::pair<paramkey, paramval> (std::string("realtimefactor"), realTimeFactor));
    list.push_back(std::pair<paramkey, paramval> (std::string("drawinterval"), drawInterval));
    list.push_back(std::pair<paramkey, paramval> (std::string("controlinterval"), controlInterval));
    list.push_back(std::pair<paramkey, paramval> (std::string("cameraspeed"), cameraSpeed));
    list.push_back(std::pair<paramkey, paramval> (std::string("gravity"), gravity));
    return list;
  } 


  paramval getParam(const paramkey& key) const {
    if(key == "noise") return noise; 
    else if(key == "simstepsize") return simStepSize; 
    else if(key == "realtimefactor") return realTimeFactor; 
    else if(key == "drawinterval") return drawInterval; 
    else if(key == "controlinterval") return controlInterval; 
    else if(key == "cameraspeed") return cameraSpeed; 
    else if(key == "gravity") return gravity; 
    else  return 0.0;
  }
        
  bool setParam(const paramkey& key, paramval val){
    if(key == "noise") noise = val; 
    else if(key == "simstepsize") {
      simStepSize=std::max(0.0000001,val); 
      drawInterval=calcDrawInterval();
    }else if(key == "realtimefactor"){
      realTimeFactor=std::max(0.0,val); 
      drawInterval=calcDrawInterval();
    }
    else if(key == "drawinterval") drawInterval=(int)val; 
    else if(key == "controlinterval") controlInterval=(int)val; 
    else if(key == "cameraspeed") cameraSpeed=(int)val; 
    else if(key == "gravity") {
      gravity=val; 
      dWorldSetGravity ( odeHandle.world , 0 , 0 , gravity );
    }
    else return false;
    return true;
  }

  void setOdeHandle(const OdeHandle& odeHandle){
    this->odeHandle = odeHandle;
  }

private:
  /// calculates the draw interval with simStepSize and realTimeFactor so that we have 25 frames/sec
  int calcDrawInterval(){
    if(realTimeFactor>0 && simStepSize>0){
      return int(ceil(1/(25.0*simStepSize/realTimeFactor)));
    }else return 50;
  }


public:
  double simStepSize;
  double realTimeFactor;
  int drawInterval;
  int controlInterval;
  double noise;
  double gravity;
  double cameraSpeed;
  std::string name;
  OdeHandle odeHandle;
};

}

#endif
