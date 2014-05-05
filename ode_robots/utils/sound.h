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

#ifndef           SOUND_H_
#define           SOUND_H_

#include "pos.h"

namespace lpzrobots {

  class GlobalData;

  /// Object that represents a sound signal in the simulator
  class Sound {
  public:
    Sound(double time, const Pos& pos, float intensity, float frequency, void* sender);

    ~Sound();

    void createVisual(GlobalData& globalData, double visualSize, Pos visualOffset) const;

    /// nice predicate function for finding old sound signals
    struct older_than : public std::unary_function<const Sound&, bool> {
      older_than(double time) : time(time) {}
      double time;
      bool operator()(const Sound& s) { return s.time < time; }
    };

    double time;
    Pos pos;    ///< emission position
    float intensity; ///< intensity 0..1
    float frequency; ///< frequency -1..1
    void* sender;    ///< pointer to the sender (can be used for self
                     ///detection)

  };

} // end namespace

#endif             /* !SOUND_H_ */
