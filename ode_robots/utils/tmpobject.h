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
#ifndef __TMPOBJECT_H
#define __TMPOBJECT_H

#include "osghandle.h"
#include "odehandle.h"
#include "pose.h"

namespace lpzrobots {

  /**
     this is the base-class for objects that exist temporarily like
     some indicator of manipulation or a message text
   */
  class TmpObject {
  public:
    TmpObject()
      : time(0) {} ;

    virtual ~TmpObject() {};
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle) = 0;
    /// deletes the object
    virtual void deleteObject() = 0;
    /// update graphics here
    virtual void update() =0 ;

    void setExpireTime(double time) { this->time= time; }
    bool expired(double time) { return this->time < time;}

  protected:
    double time;
  };

}

#endif
