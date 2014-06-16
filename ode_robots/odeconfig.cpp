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
 ***************************************************************************/
#include "odeconfig.h"

#include <ode-dbl/ode.h>
#include "mathutils.h"

namespace lpzrobots {

  OdeConfig::OdeConfig() :
    Configurable ("Simulation Environment", "$Id$")
  {
    addParameterDef("noise",            &noise,0.1, 0, 1, "global noise strength");
    addParameterDef("cameraspeed",      &cameraSpeed,100, 1,1000, "camera speed");
    addParameterDef("controlinterval"  ,&controlInterval,1, 1, 100,
                    "interval in steps between subsequent controller calls");

    addParameterDef("realtimefactor"   ,&realTimeFactor, 1, 0, 100,
                    "speed of simulation wrt. real time (0: full speed)");
    addParameterDef("simstepsize"      ,&simStepSize,    0.01,0.000001,.1,
                    "stepsize of the physical simulation (in seconds)");
    addParameterDef("gravity"          ,&gravity,        -9.81,-20,20,
                    "strengh of gravity (-9.81 is earth gravity)");
    addParameterDef("randomseed"          ,&randomSeedCopy,   0,
                    "random number seed (cmdline -r) (readonly)");
    addParameterDef("fps"              ,&fps,            25,0.0001,200, "frames per second");
    addParameterDef("logwhilerecording",&logWhileRecording, true,
                    "record log file and store agents while recording a video");

    drawInterval = calcDrawInterval(fps,realTimeFactor);
    // prepare name;
    videoRecordingMode=false;
  }


  void OdeConfig::notifyOnChange(const paramkey& key){
    if(key == "simstepsize") {
      simStepSize=std::max(0.000001,simStepSize);
      drawInterval=calcDrawInterval(fps,realTimeFactor);
    }else if(key == "realtimefactor"){
      realTimeFactor=std::max(0.0,realTimeFactor);
      if (videoRecordingMode)
        drawInterval=calcDrawInterval(25,realTimeFactor);
      else
              drawInterval=calcDrawInterval(fps,realTimeFactor);
    }else if(key == "fps"){
      fps=std::max(0.0001,fps);
      if (videoRecordingMode)
        drawInterval=calcDrawInterval(25,realTimeFactor);
      else
              drawInterval=calcDrawInterval(fps,realTimeFactor);
    } else if(key == "gravity") {
      dWorldSetGravity ( odeHandle.world , 0 , 0 , gravity );
    } else if(key == "controlinterval") {
      controlInterval = std::max(1,controlInterval);
    } else if(key == "randomseed") { // this is readonly!
      std::cerr << "randomseed is readonly" << std::endl;
      randomSeedCopy = randomSeed; // reset changes
    }
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

  /// calculates the draw interval with simStepSize and realTimeFactor so that we have Hz frames/sec
  int OdeConfig::calcDrawInterval(double Hz, double rtf){
    if(rtf>0 && simStepSize>0){
      return int(ceil(1/((double)Hz*simStepSize/rtf)));
    }else return 50;
  }


}
