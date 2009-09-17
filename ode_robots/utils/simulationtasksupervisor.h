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
 *  Revision 1.3  2009-09-17 14:13:09  guettler
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
#ifndef _SIMULATIONTASKSUPERVISOR_H_
#define _SIMULATIONTASKSUPERVISOR_H_

#include "simulationtask.h"
#include <string>

namespace osg
{
  class ArgumentParser;
}


namespace lpzrobots
{

  class LpzRobotsViewer;

  class SimulationTaskSupervisor
  {
  public:

    /**
     * Returns the singleton instance of this class.
     * There is no way to instantiate yourself this class.
     * @param simTaskHandle The global SimulationTaskHandle were you can put in your needed data.
     * This handle is shared by all parallel running simulations.
     * @return the singleton instance of this class
     */
    static inline SimulationTaskSupervisor* getInstance()
    {
      if(singletonInstance==0)
        singletonInstance = new SimulationTaskSupervisor();
      return singletonInstance;
    }

    /**
     * Destroys the singleton instance of this class.
     */
    static void destroyInstance()
    {
      if (singletonInstance!=0)
      {
        delete singletonInstance;
        singletonInstance=0;
      }
    }

    static void setSimTaskHandle(SimulationTaskHandle& _simTaskHandle)
    {
      simTaskHandle= &_simTaskHandle;
    }

    static void setTaskedSimCreator(TaskedSimulationCreator& _taskedSimCreator)
    {
      taskedSimCreator = &_taskedSimCreator;
    }

    /**
     * Sets the number of total threads running at one time.
     * @param numberThreads
     */
    static void setNumberThreads(int numberThreads);

    /**
     * Sets the number of threads created per core. The default value is 1.
     * So if your machine has e.g. 4 cores, 4 threads are created.
     * If you have much code which must be synchronized, it may be
     * useful to increase the number of threads per core, 2 is a good value.
     * @param numberThreadsPerCore
     */
    static void setNumberThreadsPerCore(int numberThreadsPerCore);

    /**
     * Creates one SimulationTask with taskId=SimulationTaskHandle.simTaskList.size().
     * @param argc count of arguments in argv
     * @param argv array of arguments, given to Simulation when the tasks starts
     */
    virtual void createSimTask()
    {
      SimulationTask* simTask = new SimulationTask(simTaskList.size());
      SimulationTaskSupervisor::simTaskList.push_back(simTask);
    }

    /**
     * Same as createSimTask, but create more than one task at once.
     * taskIds assigned in ascending order.
     * @param taskCount number of tasks to create
     * @param argc count of arguments in argv
     * @param argv array of arguments, given to Simulation when the tasks starts
     */
    virtual void createSimTasks(int taskCount)
    {
      for (int i=0; i<taskCount; i++)
        createSimTask();
    }

    /**
     * Runs all generated SimulationTasks.
     */
    virtual void runSimTasks(int* argc, char** argv);

    /**
     * Sets a suffix to be appended to the window name to identify your simTask
     */
    virtual void setSimTaskNameSuffix(std::string name);

  protected:

    SimulationTaskSupervisor() {}

    virtual ~SimulationTaskSupervisor() {}

    static SimulationTaskSupervisor* singletonInstance;
    static SimulationTaskHandle* simTaskHandle;
    static TaskedSimulationCreator* taskedSimCreator;
    static std::vector<SimulationTask*> simTaskList;
    static int*   argc;
    static char** argv;
    static osg::ArgumentParser* parser;
    static LpzRobotsViewer* viewer;
    static std::string nameSuffix;


  };

} // end namespace lpzrobots

#endif /* _SIMULATIONTASKSUPERVISOR_H_ */
