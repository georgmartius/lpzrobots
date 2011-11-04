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

#ifndef __GLOBALDATABASE_H_
#define __GLOBALDATABASE_H_

#include "agent.h"
#include "configurablelist.h"

#ifndef NOCONFIGURATOR
namespace lpzrobots {
  class ConfiguratorProxy;
}
#endif

typedef std::vector<Agent*> AgentList;

class GlobalDataBase {
  public:
    GlobalDataBase();

    virtual ~GlobalDataBase();

    virtual AgentList& getAgents() = 0;

    template <typename Derived> struct dynamic_agent_caster {
        Agent* operator()(Derived instance) { return dynamic_cast<Agent*>(instance); }
    };

    /**
     * Creates the Configurator and, if already exists, destroys the old one.
     */
    void createConfigurator();

    /**
     * Destroys the Configurator if it was created.
     */
    void removeConfigurator();
  
    /**
     * @return true if the Configurator is open and alive
     */
    bool isConfiguratorOpen();

    ConfigurableList configs;
    
#ifndef NOCONFIGURATOR
    lpzrobots::ConfiguratorProxy * configurator;
#else
    void* configurator;
#endif
  
};

#endif /* __GLOBALDATABASE_H_ */
