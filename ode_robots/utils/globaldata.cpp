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
#include "globaldata.h"
#include <algorithm>
#include "odeagent.h"
#include "osgprimitive.h"

namespace lpzrobots {

  void GlobalData::addTmpObject(TmpObject* i, double duration){
    if(i){
      i->setExpireTime(time+duration);
      uninitializedTmpObjects.push_back(std::pair<double, TmpObject*>(time+duration,i));
    }
  }

  void GlobalData::initializeTmpObjects(const OdeHandle& odeHandle,
                                        const OsgHandle& osgHandle){
    if(!uninitializedTmpObjects.empty()){
      FOREACH(TmpObjectList, uninitializedTmpObjects, i){
        i->second->init(odeHandle, osgHandle);
        tmpObjects.insert(TmpObjectList::value_type(i->first, i->second));
      }
      uninitializedTmpObjects.clear();
    }
  }

  void GlobalData::updateTmpObjects(const OsgHandle& osgHandle){
    if(!tmpObjects.empty()){
      FOREACH(TmpObjectMap, tmpObjects, i){
        i->second->update();
      }
    }
  }

  /// removes a particular temporary display item even if it is not yet expired
  bool GlobalData::removeTmpObject(TmpObject* obj){
    if(!tmpObjects.empty()){
      TmpObjectMap::iterator i = tmpObjects.begin();
      while(i != tmpObjects.end()){
        if( i->second == obj ){
          i->second->deleteObject();
          delete i->second;
          tmpObjects.erase(i);
          return true;
        }
        ++i;
      }
    }
    return false;
  }

  void GlobalData::removeExpiredObjects(double time){
    if(!tmpObjects.empty()){
      if(time<0) time=this->time;
      TmpObjectMap::iterator i = tmpObjects.begin();
      while(i != tmpObjects.end()){
        if( i->first < time ){
          TmpObjectMap::iterator tmp = i;
          ++tmp;
          i->second->deleteObject();
          delete i->second;
          tmpObjects.erase(i);
          i=tmp;
        }else{
          break; // since they are ordered we can stop here
        }
      }
    }

    // remove old signals from sound list
    if(!sounds.empty())
      sounds.remove_if(Sound::older_than(time));
  }


  AgentList& GlobalData::getAgents() {
    transform(agents.begin(), agents.end(), std::back_inserter(baseAgents), dynamic_agent_caster<OdeAgent*> ());
    return baseAgents;
  }

}
