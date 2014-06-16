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
#ifndef __ODECONFIG_H
#define __ODECONFIG_H

#include <selforg/configurable.h>
#include "odehandle.h"

namespace lpzrobots {

  /**
     The class $name holds the configurable parameters of the simulation environment.
  */
  class OdeConfig : public Configurable {
  public:

    // creates new instance of OdeConfig with default values
    OdeConfig();

    virtual ~OdeConfig() {}

    virtual long int getRandomSeed() const { return randomSeed; }

    virtual void setRandomSeed(long int randomSeed){
      this->randomSeed=randomSeed;
      randomSeedCopy = randomSeed;
    }

    virtual void setOdeHandle(const OdeHandle& odeHandle);

    virtual void setVideoRecordingMode(bool mode);

    virtual void calcAndSetDrawInterval(double Hz, double rtf);

    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);


  private:
    /// calculates the draw interval with simStepSize and realTimeFactor so that we have the Hz frames/sec
    int calcDrawInterval(double Hz, double rtf);

  public:
    bool videoRecordingMode;
    bool logWhileRecording;
    double simStepSize;
    int drawInterval;
    int controlInterval;
    double noise;
    double gravity;
    double cameraSpeed;
    OdeHandle odeHandle;

    double realTimeFactor;
    double fps;
  protected:
    long randomSeed;
    double randomSeedCopy;
  };

}

#endif
