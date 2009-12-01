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
 ***************************************************************************
 ** Started on  Mon Oct 15 18:01:12 2007 Georg Martius
 ** Last update Mon Oct 15 18:01:12 2007 Georg Martius
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-12-01 17:32:10  martius
 *   adapted Makefiles to ignore backward compat. errors
 *
 *   Revision 1.1  2007/11/07 13:17:40  martius
 *   motors in a general sense (like sound, light,...)
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef   	SPEAKER_H_
# define   	SPEAKER_H_

#include "motor.h"

namespace lpzrobots {
  
  /**
     This "motor" emulates a speaker or piezo element to produce sound.
     The sound can be detected by sound sensors (@see SoundSensor).
     Note that obstacles do not interact with the sound in any way.
   */
  class Speaker: public Motor {
  public: 
    Speaker(float frequency)
      : frequency(frequency) {
    }
    virtual ~Speaker() {};
    
    virtual void init(Primitive* own){
      this->own=own;
    }

    virtual int getMotorNumber() const{
      return 1;
    };

    virtual bool act(GlobalData& globaldata){
      globaldata.sounds.push_back(Sound(globaldata.time, own->getPosition(),
					intensity,frequency, (void*)own));
      return true;
    }

    virtual int set(const motor* values, int length){
      if(length>0)
	intensity=values[0];
      return 1;
    };    
    
  private:
    Primitive* own;
    float frequency;    
    float intensity;
  };

}

#endif 	    /* !SPEAKER_H_ */
