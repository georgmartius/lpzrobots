/***************************************************************************
 *   Copyright (C) 2011 by                                                 *
 *   Research Network for Self-Organization of Robot Behavior              *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    wrabe@informatik.uni-leipzig.de                                      *
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
 *   Revision 1.4  2011-10-27 15:54:36  martius
 *   new build system with -config shell script and configurator intragration
 *
 *   Revision 1.3  2011/10/25 12:52:42  guettler
 *   fix with static p_thread call function
 *
 *   Revision 1.2  2011/10/25 12:24:46  guettler
 *   removed Q from ConfiguratorProxy; other changes
 *
 *   Revision 1.1  2011/07/11 16:05:12  guettler
 *   - access to Configurator is now provided by ConfiguratorProxy
 *   - creating static lib instead of dynamic variant
 *   - establish correct directory structure for including configurator into other non-qt projects
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __CONFIGURATORPROXY_H_
#define __CONFIGURATORPROXY_H_


#include <selforg/configurablelist.h>
#include <selforg/callbackable.h>


namespace lpzrobots {

  class QConfigurator;

  /**
   * Proxy which controls the creation process of the QConfigurator
   */
  class ConfiguratorProxy : public Callbackable {
    public:
      ConfiguratorProxy(ConfigurableList& configList);
      virtual ~ConfiguratorProxy();

      virtual void doOnCallBack(BackCaller* source, BackCaller::CallbackableType type = BackCaller::DEFAULT_CALLBACKABLE_TYPE);

      void createConfigurator();

    private:
      ConfigurableList& configList;
      pthread_t configuratorThread;
      QConfigurator* configurator;

  };

}

#endif /* __CONFIGURATORPROXY_H_ */
