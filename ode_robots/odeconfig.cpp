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
 *   Revision 1.4  2007-01-25 14:09:15  martius
 *   use new configurable addParameter feature
 *
 *   Revision 1.3  2006/08/04 15:07:46  martius
 *   documentation
 *
 *   Revision 1.2  2006/07/14 11:57:23  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2006/06/29 16:32:44  robot3
 *   implementation of odeconfig moved to odeconfig.cpp
 *
 *   Revision 1.12.4.7  2006/05/08 12:12:36  robot3
 *   changes from Revision 1.40.4.27 reverted
 *
 *   Revision 1.12.4.6  2006/04/27 16:17:38  robot3
 *   implemented some functions for motionblurcallback
 *
 *   Revision 1.12.4.5  2006/03/28 09:55:12  robot3
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
#include "odeconfig.h"

#include <ode/ode.h>
#include "mathutils.h"

namespace lpzrobots {

  OdeConfig::OdeConfig() :
    Configurable ("Simulation Environment", "$Id$")
  {
    realTimeFactor=1.0;
    gravity=-9.81;
    simStepSize = 0.01;
    controlInterval = 1;
    drawInterval = calcDrawInterval25();
    addParameterDef("noise",            &noise,0.1);
    addParameterDef("cameraspeed",      &cameraSpeed,100);
    addParameterDef("motionpersistence",&motionPersistence,0.09);
    // prepare name;
    videoRecordingMode=false;
    drawBoundings=false;
  }
        
  Configurable::paramlist OdeConfig::getParamList() const{
    paramlist list = Configurable::getParamList();
    list.push_back(std::pair<paramkey, paramval> (std::string("controlinterval"), controlInterval));
    list.push_back(std::pair<paramkey, paramval> (std::string("simstepsize"), simStepSize));
    list.push_back(std::pair<paramkey, paramval> (std::string("gravity"), gravity));
    list.push_back(std::pair<paramkey, paramval> (std::string("realtimefactor"), realTimeFactor));
    list.push_back(std::pair<paramkey, paramval> (std::string("drawinterval"), drawInterval));
    return list;
  } 

  Configurable::paramval OdeConfig::getParam(const paramkey& key) const {
    if(key == "realtimefactor") return realTimeFactor; 
    else if(key == "gravity") return gravity; 
    else if(key == "simstepsize") return simStepSize;
    else if(key == "drawinterval") return drawInterval;
    else if(key == "controlinterval") return controlInterval;
    else return Configurable::getParam(key);
  }

  bool OdeConfig::setParam(const paramkey& key, paramval val){
    if(key == "simstepsize") {
      simStepSize=std::max(0.0000001,val); 
      drawInterval=calcDrawInterval25();
    }else if(key == "realtimefactor"){
      realTimeFactor=std::max(0.0,val); 
      //      if (videoRecordingMode)
      drawInterval=calcDrawInterval25();
      //      else
      //	drawInterval=calcDrawInterval50();
    } else if(key == "gravity") {
      gravity=val; 
      dWorldSetGravity ( odeHandle.world , 0 , 0 , gravity );
    } else if(key == "controlinterval") {
      controlInterval = std::max(1,int(val)); 
    } else if(key == "drawinterval") {
      drawInterval = std::max(1,int(val)); 
    } else {
      return Configurable::setParam(key,val);      
    }
    return true;
  }


  void OdeConfig::setOdeHandle(const OdeHandle& odeHandle){
    this->odeHandle = odeHandle;
  }

  void OdeConfig::setVideoRecordingMode(bool mode) {
    if (mode)
      drawInterval=calcDrawInterval25();
    else
      drawInterval=calcDrawInterval50();
    videoRecordingMode=mode;
  }

  /// calculates the draw interval with simStepSize and realTimeFactor so that we have 25 frames/sec
  int OdeConfig::calcDrawInterval25(){
    if(realTimeFactor>0 && simStepSize>0){
      return int(ceil(1/(25.0*simStepSize/realTimeFactor)));
    }else return 50;
  }

  /// calculates the draw interval with simStepSize and realTimeFactor so that we have 50 frames/sec
  /// this is much better for graphical visualization (smoother)
  int OdeConfig::calcDrawInterval50(){
    if(realTimeFactor>0 && simStepSize>0){
      return int(ceil(1/(50.0*simStepSize/realTimeFactor)));
    }else return 50;
  }


}
