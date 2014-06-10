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
#ifndef           SPEAKER_H_
#define           SPEAKER_H_

#include "motor.h"

namespace lpzrobots {

  /**
     This "motor" emulates a speaker or piezo element to produce sound.
     The sound can be detected by sound sensors (@see SoundSensor).
     Note that obstacles do not interact with the sound in any way.
   */
  class Speaker: public Motor {
  public:
    Speaker(float frequency, double visualSize = 0.5, Pos visualOffset = Pos(0,0,1))
      : own(0), frequency(frequency), intensity(0.0),
        visualSize(visualSize), visualOffset(visualOffset) {
    }
    virtual ~Speaker() {};

    virtual void init(Primitive* own, Joint* joint = 0){
      this->own=own;
    }

    virtual int getMotorNumber() const{
      return 1;
    };

    virtual bool act(GlobalData& globaldata){
      Sound s = Sound(globaldata.time, own->getPosition(),
                      intensity,frequency, (void*)own);
      globaldata.sounds.push_back(s);
      s.createVisual(globaldata, visualSize, visualOffset);

      return true;
    }

    virtual int set(const motor* values, int length){
      if(length>0)
        intensity=std::max(std::min(values[0],1.0),0.0);
      return 1;
    };

  private:
    Primitive* own;
    float frequency;
    float intensity;

    double visualSize;
    Pos visualOffset;
  };

}

#endif             /* !SPEAKER_H_ */
