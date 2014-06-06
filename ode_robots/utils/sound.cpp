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

#include "sound.h"
#include "osghandle.h"
#include "osgprimitive.h"
#include "globaldata.h"
#include "tmpprimitive.h"
#include "globaldata.h"

namespace lpzrobots {

  Sound::Sound(double time, const Pos& pos, float intensity, float frequency, void* sender)
    : time(time), pos(pos),
      frequency(frequency), sender(sender) {
    this->intensity=std::max(std::min(intensity, 1.0f), 0.0f);
  }

  Sound::~Sound(){
  }

  void Sound::createVisual(GlobalData& globalData, double visualSize, Pos visualOffset) const {
    globalData.addTmpObject(new TmpDisplayItem(new OSGSphere((intensity)*visualSize),
                                               TRANSM(pos + visualOffset),
                                               Color(255-int((frequency+1.0)*128.0),
                                                     0,int((frequency+1.0)*128.0),0.4)),
                            globalData.odeConfig.simStepSize*globalData.odeConfig.controlInterval);

  }

}
