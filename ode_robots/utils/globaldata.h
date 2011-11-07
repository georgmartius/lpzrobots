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
#ifndef __GLOBALDATA_H
#define __GLOBALDATA_H

#include <vector>
#include "odehandle.h"
#include "odeconfig.h"
#include "sound.h"
#include "tmpdisplayitem.h"
#include <selforg/plotoption.h>
#include <selforg/globaldatabase.h>

class Configurable;

namespace lpzrobots {

  class OdeAgent;
  class AbstractObstacle;
  class Primitive;
  class OSGPrimitive;

  typedef std::vector<AbstractObstacle*> ObstacleList;
  typedef Configurable::configurableList ConfigList;
  typedef std::vector<OdeAgent*> OdeAgentList;
  typedef std::list<Sound> SoundList;
  typedef std::list<PlotOption> PlotOptionList;
  typedef std::list<std::pair<OSGPrimitive*, double> > TmpObjectsList;
  typedef std::list<TmpDisplayItem> TmpDisplayItemList;

  /**
   Data structure holding all essential global information.
   */
  class GlobalData : public GlobalDataBase {
    public:
      GlobalData() {
        sim_step = 0;
        environment = 0;
      }

      virtual ~GlobalData() {}

      OdeConfig odeConfig;
      ObstacleList obstacles;
      OdeAgentList agents;
      Primitive* environment; /// < this is used to be able to attach objects to the static environment

      SoundList sounds; ///< sound space

      PlotOptionList plotoptions; ///< plotoptions used for new agents
      std::list<Configurable*> globalconfigurables; ///< global configurables plotted by all agents

      double time;
      long int sim_step; ///< time steps since start
    
      
      /// adds a temporary display item with given life duration in sec
      virtual void addTmpDisplayItem(TmpDisplayItem i, double duration);
      virtual void initializeTmpDisplayItems(const OsgHandle& osgHandle);

      /// removes all expired sounds and temporary display items
      virtual void removeExpiredItems();

      virtual AgentList& getAgents();

    private:

      TmpDisplayItemList uninitializedTmpDisplayItems;
      TmpDisplayItemList tmpDisplayItems;
      AgentList baseAgents;
  };

}

#endif
