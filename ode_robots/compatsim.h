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
 *                                                                         *
 *  compatsim is provided for compatibility to the old simulation          *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.2  2005-12-15 17:02:04  martius
 *   light is in sky and standart cams removed
 *   config has a default implentation now
 *
 *   Revision 1.1.2.1  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __COMPATSIM_H
#define __COMPATSIM_H

#include "simulation.h"

namespace lpzrobots {

class CompatSim : public Simulation {
public:
    
  CompatSim(void (*startFun)(const OdeHandle&, const OsgHandle&, GlobalData& globalData), 
	    void (*endFun)(GlobalData& globalData), 
	    void (*configFun)(GlobalData& globalData) = 0, 
	    void (*commandFun)(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key) = 0, 
	    void (*collCallbackFun)(const OdeHandle&, void* data, dGeomID o1, dGeomID o2) = 0,
	    void (*addCallbackFun)(GlobalData& globalData, bool draw, bool pause) = 0);
  virtual ~CompatSim();

  // the following function have to be overloaded.
  virtual void start(const OdeHandle&, const OsgHandle&, GlobalData& globalData);
  virtual void end(GlobalData& globalData);
  virtual void config(GlobalData& globalData);
  // the following functions have dummy default implementations
  virtual void command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key);
  /// @return true if collision is treated, false otherwise
  virtual bool collCallback(const OdeHandle&, void* data, dGeomID o1, dGeomID o2);
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause);

protected:
  void (*startFun)(const OdeHandle& odeHandle, const OsgHandle&, GlobalData& globalData);
  void (*endFun)(GlobalData& globalData);
  void (*configFun)(GlobalData& globalData);
  void (*commandFun)(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key);
  void (*collCallbackFun)(const OdeHandle&, void* data, dGeomID o1, dGeomID o2);
  void (*addCallbackFun)(GlobalData& globalData, bool draw, bool pause);

};
}

#endif
