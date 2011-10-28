/***************************************************************************
 *   Copyright (C) 2011 by                                                 *
 *   Research Network for Self-Organization of Robot Behavior              *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    wrabe@informatik.uni-leipzig.de                                      *
 *    Georg.Martius@mis.mpg.de                                             *
 *    ralfder@mis.mpg.de                                                   *
 *    frank@nld.ds.mpg.de                                                  *
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
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2011-10-28 17:17:28  guettler
 *   improved Configurator integration
 *
 *   Revision 1.2  2011/10/27 15:54:36  martius
 *   new build system with -config shell script and configurator intragration
 *
 *   Revision 1.1  2011/10/25 12:31:16  guettler
 *   new base class for ode_robots/GlobalData;
 *   support for creation of Configurator (via ConfiguratorProxy)
 *
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
