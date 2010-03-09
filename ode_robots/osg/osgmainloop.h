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
 ***************************************************************************
 *                                                                         *
 *   This file provides the main simulation loop using open scene graph    *
 *   It creates the world scene and call the call back functions and so on.*
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2010-03-09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.2  2006/07/14 12:23:35  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2005/12/06 10:13:24  martius
 *   openscenegraph integration started
 *
 *                                                                 *
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
