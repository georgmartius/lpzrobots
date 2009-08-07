/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   Revision 1.8  2009-08-07 13:31:54  martius
 *   sim_steps moved here
 *
 *   Revision 1.7  2009/08/07 09:11:18  martius
 *   plotoptions and globalconfigurables are now in globaldata
 *
 *   Revision 1.6  2007/11/07 13:27:55  martius
 *   sound added
 *
 *   Revision 1.5  2007/03/16 10:56:33  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2006/07/14 12:23:56  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.3.4.5  2006/06/25 17:01:26  martius
 *   removed using namespace std
 *
 *   Revision 1.3.4.4  2006/03/29 15:10:00  martius
 *   Dummy Primitive for Environment
 *
 *   Revision 1.3.4.3  2005/12/06 10:13:26  martius
 *   openscenegraph integration started
 *
 *   Revision 1.3.4.2  2005/11/15 12:30:24  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.3.4.1  2005/11/14 17:37:25  martius
 *   moved to selforg
 *
 *   Revision 1.3  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef __GLOBALDATA_H
#define __GLOBALDATA_H


#include <vector>
#include "odehandle.h"
#include "odeconfig.h"
#include "sound.h"
#include <selforg/plotoption.h>

class Configurable;

namespace lpzrobots {

class OdeAgent;
class AbstractObstacle;
class Primitive;

typedef std::vector<AbstractObstacle*> ObstacleList;      
typedef std::vector<Configurable*> ConfigList;
typedef std::vector<OdeAgent*>     OdeAgentList; 
typedef std::list<Sound>           SoundList; 
typedef std::list<PlotOption>      PlotOptionList;

/**
  Data structure holding all essential global information.
*/
struct GlobalData
{
  GlobalData() { 
    time        = 0;
    sim_step    = 0;
    environment = 0;
  }

  OdeConfig odeConfig;
  ConfigList configs;
  ObstacleList obstacles;
  OdeAgentList agents;
  Primitive* environment; /// < this is used to be able to attach objects to the static environment

  SoundList sounds;  ///< sound space

  PlotOptionList plotoptions;     ///< plotoptions used for new agents
  std::list<Configurable*> globalconfigurables; ///< global configurables plotted by all agents

  double time;
  long int sim_step; ///< time steps since start
};

}

#endif
