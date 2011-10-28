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
 *   Revision 1.5  2011-10-28 16:14:21  guettler
 *   Configurator closes if ConfiguratorProxy is deleted
 *
 *   Revision 1.4  2011/10/27 15:54:36  martius
 *   new build system with -config shell script and configurator intragration
 *
 *   Revision 1.3  2011/10/25 12:52:42  guettler
 *   fix with static p_thread call function
 *
 *   Revision 1.2  2011/10/25 12:25:32  guettler
 *   instance is now created in a separate thread with p_threads
 *
 *   Revision 1.1  2011/07/11 16:06:01  guettler
 *   - access to Configurator is now provided by ConfiguratorProxy
 *   - creating static lib instead of dynamic variant
 *   - establish correct directory structure for including configurator into other non-qt projects
 *
 *                                                                         *
 ***************************************************************************/

#include "ConfiguratorProxy.h"
#include "QConfigurator.h"
#include <pthread.h>

using namespace std;

namespace lpzrobots {
  
  static void* createConfiguratorThread(void* thread);

  ConfiguratorProxy::ConfiguratorProxy(ConfigurableList& configList) :
    configList(configList) {
    pthread_create(&configuratorThread, NULL, createConfiguratorThread, this);
  }

  ConfiguratorProxy::~ConfiguratorProxy() {
    if (configurator!=0) {
      configurator->close();
      configurator = 0;
    }
  }

  void ConfiguratorProxy::doOnCallBack(BackCaller* source, BackCaller::CallbackableType type /*= BackCaller::DEFAULT_CALLBACKABLE_TYPE*/) {
    if (type == ConfigurableList::CALLBACK_CONFIGURABLE_LIST_BEING_DELETED) {
      // TODO: fix this
      //      delete configurator;
      //      delete this;
    }
  }

  void ConfiguratorProxy::createConfigurator() {
    int argc=1;
    char* argv0 = (char*)"configurator";
    char** argv = &argv0;
    QApplication app(argc, argv);

    QString appPath = QString(argv[0]);
    //    configurator = new QConfigurator(appPath.mid(0, appPath.lastIndexOf("/") + 1), configList);
    configurator = new QConfigurator(appPath, configList);

    configurator->show();

    configList.addCallbackable(this, ConfigurableList::CALLBACK_CONFIGURABLE_LIST_BEING_DELETED);

    app.exec();
  }

  static void* createConfiguratorThread(void* thread) {
    ConfiguratorProxy* proxy = dynamic_cast<ConfiguratorProxy*> ((ConfiguratorProxy*) thread);
    if (proxy)
      proxy->createConfigurator();
    else {
      cerr << "createConfiguratorProcess()::Shit happens" << endl;
    }
    return NULL;
  }

}
