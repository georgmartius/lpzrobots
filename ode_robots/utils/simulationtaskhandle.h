/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.4  2009-10-06 11:50:56  robot12
 *  some bugfixes
 *
 *  Revision 1.3  2009/09/17 14:13:09  guettler
 *  - some bugfixes for critical sections
 *  - support to set number of threads per core
 *
 *  Revision 1.2  2009/08/21 09:49:07  robot12
 *  (guettler) support for tasked simulations.
 *  - use the simulation template_taskedSimulations.
 *  - merged (not completely) from lpzrobots_tasked.
 *  - graphics is supported, but only for one simulation of a pool
 *
 *  Revision 1.1.2.1  2009/08/11 15:59:20  guettler
 *  - support for tasked simulations, does not yet work with graphics
 *  - in development state
 *										   *
 *                                                                         *
 **************************************************************************/
#ifndef _SIMULATIONTASKHANDLE_H_
#define _SIMULATIONTASKHANDLE_H_

#include <vector>

namespace lpzrobots
{

  /**
   * struct which holds all structural data for the simulations.
   * A specialized class can be deduced from this one to hold necessary
   * information, for the methods start and restart of the Simulation.
   */
  struct SimulationTaskHandle
  {
  };

}

#endif /* _SIMULATIONTASKHANDLE_H_ */
