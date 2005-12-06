/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   $Log$
 *   Revision 1.1.2.1  2005-12-06 10:13:23  martius
 *   openscenegraph integration started
 *
 *                                                                 *
 ***************************************************************************/

#include "compatsim.h"

namespace lpzrobots {


CompatSim::CompatSim(void (*startFun)(const OdeHandle&, const OsgHandle&, GlobalData& globalData), 
		     void (*endFun)(GlobalData& globalData), 
		     void (*configFun)(GlobalData& globalData), 
		     void (*commandFun)(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key), 
		     void (*collCallbackFun)(const OdeHandle&, void* data, dGeomID o1, dGeomID o2),
		     void (*addCallbackFun)(GlobalData& globalData, bool draw, bool pause))
  : startFun(startFun), endFun(endFun), configFun(configFun), commandFun(commandFun),
    collCallbackFun(collCallbackFun), addCallbackFun(addCallbackFun) {
}

CompatSim::~CompatSim() {}


void CompatSim::start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& globalData){
  if(startFun) startFun(odeHandle, osgHandle, globalData);
}

void CompatSim::end(GlobalData& globalData){
  if(endFun) endFun(globalData);
};

void CompatSim::config(GlobalData& globalData){
  if(configFun) configFun(globalData);
}

void CompatSim::command(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
			GlobalData& globalData, int key) {
  if(commandFun) commandFun(odeHandle, osgHandle, globalData, key);
}

bool CompatSim::collCallback(const OdeHandle& odeHandle, void* data, dGeomID o1, dGeomID o2) { 
  if(collCallbackFun){
    collCallbackFun(odeHandle, data, o1, o2);
    return true;
  } else 
    return false;
}

void CompatSim::addCallback(GlobalData& globalData, bool draw, bool pause) {
  if(addCallbackFun)
    addCallbackFun(globalData, draw, pause);
}

}
