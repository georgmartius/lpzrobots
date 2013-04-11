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
#ifndef __PLATFUSSSCHLANGESERVO
#define __PLATFUSSSCHLANGESERVO

#include "schlangeservo2.h"
#include "oneaxisservo.h"

namespace lpzrobots {

  /**
   * This is a class, which models a snake like robot with flat ends and a big body in the middle.
   * It consists of a number of equal elements, each linked
   * by a universal joint powered by 2 servos
   **/
  class PlattfussSchlange: public SchlangeServo2
  {

  public:
    PlattfussSchlange ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                        const SchlangeConf& conf,
                        const std::string& name);

    virtual ~PlattfussSchlange();

  private:
    virtual Primitive* createSegment(int index, const OdeHandle& odeHandle);
  };

}

#endif
