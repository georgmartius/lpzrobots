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
#include <map>
#include "odehandle.h"
#include "odeconfig.h"
#include "sound.h"
#include "tmpobject.h"
#include <selforg/plotoption.h>
#include <selforg/globaldatabase.h>
#include <selforg/backcallervector.h>

class Configurable;

namespace lpzrobots {

  class OdeAgent;
  class AbstractObstacle;
  class Primitive;
  class OSGPrimitive;

  typedef std::vector<AbstractObstacle*> ObstacleList;
  typedef Configurable::configurableList ConfigList;
  typedef BackCallerVector<OdeAgent*> OdeAgentList;
  typedef std::list<Sound> SoundList;
  typedef std::list<PlotOption> PlotOptionList;
  typedef std::multimap<double, TmpObject* > TmpObjectMap;
  typedef std::list< std::pair<double, TmpObject*> > TmpObjectList;

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

      // Todo: the sound visualization could be done with the new TmpObjects
      SoundList sounds; ///< sound space

      PlotOptionList plotoptions; ///< plotoptions used for new agents
      std::list<Configurable*> globalconfigurables; ///< global configurables plotted by all agents

      double time;
      long int sim_step; ///< time steps since start

      /// returns the list of all agents
      virtual AgentList& getAgents();


      /// adds a temporary display item with given life duration in sec
      virtual void addTmpObject(TmpObject* i, double duration);

      /// called by Simulation to initialize tmp objects
      virtual void initializeTmpObjects(const OdeHandle& odeHandle,
                                        const OsgHandle& osgHandle);
      /// called by Simulation to update tmp objects
      virtual void updateTmpObjects(const OsgHandle& osgHandle);

      /** called by Simulation to removes all expired sounds and temporary objects.
          Optionally a time can be specified otherwise the internal time is used.
      */
      virtual void removeExpiredObjects(double time = -1);

      /** removes a particular temporary display item even if it is not yet expired
          @return true if it was deleted (found) */
      virtual bool removeTmpObject(TmpObject* i);


    private:

      TmpObjectList uninitializedTmpObjects;
      TmpObjectMap  tmpObjects;
      AgentList     baseAgents;
  };

}

#endif
