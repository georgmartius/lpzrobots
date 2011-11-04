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
