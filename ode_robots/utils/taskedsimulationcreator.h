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
 *  Revision 1.2  2009-08-21 09:49:07  robot12
 *  (guettler) support for tasked simulations.
 *  - use the simulation template_taskedSimulations.
 *  - merged (not completely) from lpzrobots_tasked.
 *  - graphics is supported, but only for one simulation of a pool
 *
 *  Revision 1.1.2.1  2009/08/11 15:59:20  guettler
 *  - support for tasked simulations, does not yet work with graphics
 *  - in development state
 *					   *
 *                                                                         *
 **************************************************************************/
#ifndef _TASKEDSIMULATIONCREATOR_H_
#define _TASKEDSIMULATIONCREATOR_H_

namespace lpzrobots
{

  /**
   * Defines a method to construct a TaskedSimulation. This method is needed by the
   * SimulationTask, provided through the SimulationTaskSupervisor.
   * If you like to get the singleton instance of SimulationTaskSupervisor, you have
   * to pass as argument a deduced instance of the TaskedSimulationCreator.
   */
  class TaskedSimulationCreator
  {
  public:

    /**
     * Constructs and returns an instance of this class.
     */
    TaskedSimulationCreator() {}

    /**
     * Destroys the instance.
     */
    virtual ~TaskedSimulationCreator() {}

    /**
     * Builds (constructs) an instance of TaskedSimulation.
     * This method must be overwritten by a deduced builder
     * @return the builded instance with basic type TaskedSimulation
     */
    virtual TaskedSimulation* buildTaskedSimulationInstance() = 0;

  };

}

#endif /* _TASKEDSIMULATIONCREATOR_H_ */
