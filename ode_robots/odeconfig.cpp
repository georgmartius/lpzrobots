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
 *   Revision 1.11  2009-08-10 14:48:15  der
 *   calcDrawInterval gets a double
 *
 *   Revision 1.10  2009/08/05 16:14:02  martius
 *   added framerate to adjust
 *   some parameters are really handled as ints now (still with custom set function)
 *
 *   Revision 1.9  2008/09/16 14:43:45  martius
 *   motionpersistence it 0 by default
 *
 *   Revision 1.8  2008/04/23 07:17:16  martius
 *   makefiles cleaned
 *   new also true realtime factor displayed,
 *    warning if out of sync
 *   drawinterval in full speed is 10 frames, independent of the speed
 *
 *   Revision 1.7  2007/07/30 14:13:06  martius
 *   drawBoundings moved to osgHandle
 *
 *   Revision 1.6  2007/06/21 16:33:43  martius
 *   random seed
 *
 *   Revision 1.5  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.4  2007/01/25 14:09:15  martius
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
    fps=25;
    randomSeed=0;
    addParameterDef("noise",            &noise,0.1);
    addParameterDef("cameraspeed",      &cameraSpeed,100);
    addParameterDef("motionpersistence",&motionPersistence,0.0);
    addParameterDef("controlinterval"  ,&controlInterval,1);
    addParameterDef("drawinterval", &drawInterval,calcDrawInterval(fps,realTimeFactor));
    // prepare name;
    videoRecordingMode=false;
  }
        
  Configurable::paramlist OdeConfig::getParamList() const{
    paramlist list = Configurable::getParamList();
    list.push_back(std::pair<paramkey, paramval> (std::string("simstepsize"), simStepSize));
    list.push_back(std::pair<paramkey, paramval> (std::string("gravity"), gravity));
    list.push_back(std::pair<paramkey, paramval> (std::string("realtimefactor"), realTimeFactor));
    list.push_back(std::pair<paramkey, paramval> (std::string("fps"), fps));
    list.push_back(std::pair<paramkey, paramval> (std::string("randomseed"), randomSeed));
    return list;
  } 

  Configurable::paramval OdeConfig::getParam(const paramkey& key) const {
    if(key == "realtimefactor") return realTimeFactor; 
    else if(key == "gravity") return gravity; 
    else if(key == "simstepsize") return simStepSize;
    else if(key == "fps") return fps;
    else if(key == "randomseed") return randomSeed;
    else return Configurable::getParam(key);
  }

  bool OdeConfig::setParam(const paramkey& key, paramval val){
    if(key == "simstepsize") {
      simStepSize=std::max(0.0000001,val); 
      drawInterval=calcDrawInterval(fps,realTimeFactor);
    }else if(key == "realtimefactor"){
      realTimeFactor=std::max(0.0,val);      
      if (videoRecordingMode)
	drawInterval=calcDrawInterval(25,realTimeFactor);
      else
      	drawInterval=calcDrawInterval(fps,realTimeFactor);
    }else if(key == "fps"){
      fps=std::max(0.0001,val);      
      if (videoRecordingMode)
	drawInterval=calcDrawInterval(25,realTimeFactor);
      else
      	drawInterval=calcDrawInterval(fps,realTimeFactor);
    } else if(key == "gravity") {
      gravity=val; 
      dWorldSetGravity ( odeHandle.world , 0 , 0 , gravity );
    } else if(key == "controlinterval") {
      controlInterval = std::max(1,int(val)); 
    } else if(key == "drawinterval") {
      drawInterval = std::max(1,int(val)); 
    } else if(key == "randomseed") { // this is readonly!
    } else {
      return Configurable::setParam(key,val);      
    }
    return true;
  }
  
  void OdeConfig::setOdeHandle(const OdeHandle& odeHandle){
    this->odeHandle = odeHandle;
  }

  void OdeConfig::setVideoRecordingMode(bool mode) {
    //    if (mode)
    drawInterval=calcDrawInterval(25,realTimeFactor);
      //    else
      //      drawInterval=calcDrawInterval50(realTimeFactor);
    videoRecordingMode=mode;
  }

  void OdeConfig::calcAndSetDrawInterval(double Hz, double rtf){
    drawInterval = calcDrawInterval(Hz,rtf);
  }

  /// calculates the draw interval with simStepSize and realTimeFactor so that we have 25 frames/sec
  int OdeConfig::calcDrawInterval(double Hz, double rtf){
    if(rtf>0 && simStepSize>0){
      return int(ceil(1/((double)Hz*simStepSize/rtf)));
    }else return 50;
  }


}
