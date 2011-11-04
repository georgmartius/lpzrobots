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
