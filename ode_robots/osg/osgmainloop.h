/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Rald Der       <ralfder at mis dot mpg dot de>                       *
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
#ifndef __OSGMAINLOOP_H
#define __OSGMAINLOOP_H

#include <ode-dbl/ode.h>

#include "odehandle.h"
#include "globaldata.h"

class OSGMainLoop {
public:
  OSGMainLoop(){

  }
  
protected:
  /// user defined start function (called at the beginning of the simulation)
  void (*startFunction)(const OdeHandle&, GlobalData& globalData);
  /// user defined end function (called after the simulation)
  void (*endFunction)(GlobalData& globalData); 
  /// pointer to the config function of the user
  void (*configFunction)(GlobalData& globalData);
  // command function, set by user
  void (*commandFunction)(const OdeHandle&, GlobalData& globalData, int key); 
  /// pointer to the user defined additional function
  void (*collisionCallback)(const OdeHandle&, void* data, dGeomID o1, dGeomID o2);
  /// pointer to the user defined additional function which is executed in each simulationstep
  void (*additionalCallback)(GlobalData& globalData, bool draw, bool pause);
}


#endif
