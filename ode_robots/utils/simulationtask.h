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
 *                                                                         *
 *                                                                         *
 *                                                                         *
 *                                                                         *
 *                                                                         *
 * $Log$
 * Revision 1.2  2009-08-21 09:49:07  robot12
 * (guettler) support for tasked simulations.
 * - use the simulation template_taskedSimulations.
 * - merged (not completely) from lpzrobots_tasked.
 * - graphics is supported, but only for one simulation of a pool
 *
 * Revision 1.1.2.1  2009/08/11 15:59:20  guettler
 * - support for tasked simulations, does not yet work with graphics
 * - in development state
 * *
 *                                                                         *
 ***************************************************************************/
#ifndef _SIMULATIONTASK_H_
#define _SIMULATIONTASK_H_

#include "taskedsimulation.h"
#include "taskedsimulationcreator.h"
#include <string>
#include <string.h>

namespace lpzrobots {

  /**
   *
   */
  class SimulationTask
  {
  public:
    SimulationTask(int taskId) : taskId(taskId), sim(0) { }

    virtual ~SimulationTask() { }

    virtual int startTask(SimulationTaskHandle& simTaskHandle, TaskedSimulationCreator& simTaskCreator, int* argc, char** argv, std::string nameSuffix)
    {
      int returnValue=0;
      sim = simTaskCreator.buildTaskedSimulationInstance();
      if(sim!=0)
      {
        char buffer[20];
        sprintf(buffer,"%i",taskId);
        sim->setTaskNameSuffix(std::string(" - ").append(nameSuffix).append(" - ").append(buffer));
        sim->setTaskId(taskId);
        sim->setSimTaskHandle(simTaskHandle);
        returnValue = sim->run(*argc, argv)? 0 : 1;
        delete sim;
        return returnValue;
      } else
        return 1;
    }

  protected:
    int taskId;
    TaskedSimulation* sim;
  };

}

#endif /* _SIMULATIONTASK_H_ */
