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
 *   $Log: globaldata.h,v $
 *   Revision 1.1  2007/10/12 15:27:42  martius
 *   simple test for controllers
 *
 *   Revision 1.1  2007/07/10 15:02:33  robot3
 *   *** empty log message ***
 *
 *
 ***************************************************************************/
#ifndef __GLOBALDATA_H
#define __GLOBALDATA_H

#include <vector>

class Agent;
class Configurable;

typedef std::vector<Configurable*> ConfigList;       ///
typedef std::vector<Agent*> AgentList;         ///

/**
  Data structure holding all essential global information.
*/
struct GlobalData
{
  GlobalData() { }

  ConfigList configs;
  AgentList agents;
};

#endif
