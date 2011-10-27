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
 *   Revision 1.2  2011-10-27 15:54:36  martius
 *   new build system with -config shell script and configurator intragration
 *
 *   Revision 1.1  2011/10/25 12:31:16  guettler
 *   new base class for ode_robots/GlobalData;
 *   support for creation of Configurator (via ConfiguratorProxy)
 *
 *                                                                         *
 ***************************************************************************/

#include "globaldatabase.h"

#ifndef NOCONFIGURATOR
#define COMPILE_CONFIGURATOR 1 // default
#endif

#ifndef NOCONFIGURATOR
#include <configurator/ConfiguratorProxy.h>

using namespace lpzrobots;
#endif

void GlobalDataBase::createConfigurator() {
#ifndef NOCONFIGURATOR
  if(configurator==0){
    configurator = new ConfiguratorProxy(configs);
  }
#endif
}

bool GlobalDataBase::isConfiguratorOpen(){
  return configurator!=0;
}

void GlobalDataBase::removeConfigurator() {
#ifndef NOCONFIGURATOR
  if(configurator!=0){
    delete configurator;
  }
#endif
  configurator = 0;  
}
