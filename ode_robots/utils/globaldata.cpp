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

  void GlobalData::addTmpDisplayItem(TmpDisplayItem i, double duration){
    i.setExpireTime(time+duration);
    uninitializedTmpDisplayItems.push_back(i);
  }
  
  void GlobalData::initializeTmpDisplayItems(const OsgHandle& osgHandle){
    if(uninitializedTmpDisplayItems.size()>0){
      FOREACH(TmpDisplayItemList, uninitializedTmpDisplayItems, i){
        i->init(osgHandle);
        tmpDisplayItems.push_back(*i);
      }
      uninitializedTmpDisplayItems.clear();
    }
  }

  struct delOldTmpItems : 
    public std::unary_function< const TmpDisplayItem&, bool> 
  {
    delOldTmpItems(double time) : time(time) {}
    double time;
    bool operator()(const TmpDisplayItem& item) const {
      if(item.expired(time)){
        ((TmpDisplayItem)item).deleteItem();
        return true;
      }else
        return false;
    }
  };
 
  void GlobalData::removeExpiredItems(){
    // we cannot use remove_if because we want to delete the primitives
    if(tmpDisplayItems.size()>0)
      tmpDisplayItems.remove_if(delOldTmpItems(time));
    // remove old signals from sound list
    if(sounds.size()>0)
      sounds.remove_if(Sound::older_than(time));
  }


  AgentList& GlobalData::getAgents() {
    transform(agents.begin(), agents.end(), std::back_inserter(baseAgents), dynamic_agent_caster<OdeAgent*> ());
    return baseAgents;
  }

}
