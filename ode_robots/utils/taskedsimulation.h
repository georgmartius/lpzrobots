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
 *  Revision 1.1.2.1  2009/08/11 15:59:21  guettler
 *  - support for tasked simulations, does not yet work with graphics
 *  - in development state
 *					   *
 *                                                                         *
 **************************************************************************/
#ifndef _TASKEDSIMULATION_H_
#define _TASKEDSIMULATION_H_

#include "simulation.h"
#include "simulationtaskhandle.h"
#include <string>
#include <selforg/quickmp.h>

namespace lpzrobots {

  class TaskedSimulation : public Simulation {
    public:
      TaskedSimulation() :
        Simulation(), taskId(0), simTaskHandle(0) {
        useOdeThread = false;
        useOsgThread = false;
        useQMPThreads = false;
        inTaskedMode = true;
      }

      virtual ~TaskedSimulation() {
      }

      /// start() is called at the first start of the cycles and should create all the object (obstacles, agents...).
      virtual void start(const OdeHandle&, const OsgHandle&, GlobalData& globalData,
          SimulationTaskHandle& simTaskHandle, int taskId) {
      }

      /**
       * restart() is called at the second and all following starts of the cylce
       * The end of a cycle is determined by (simulation_time_reached==true)
       * @param the odeHandle
       * @param the osgHandle
       * @param globalData
       * @return if the simulation should be restarted; this is false by default
       */
      virtual bool restart(const OdeHandle&, const OsgHandle&, GlobalData& globalData, SimulationTaskHandle&,
          int taskId) {
        return false;
      }

      /** optional additional callback function which is called every simulation step.
       Called between physical simulation step and drawing.
       @param draw indicates that objects are drawn in this timestep
       @param pause always false (only called of simulation is running)
       @param control indicates that robots have been controlled this timestep
       */
      virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control, SimulationTaskHandle&,
          int taskId) {
      }

      /** is called if a key was pressed.
       For keycodes see: osgGA::GUIEventAdapter
       @return true if the key was handled
       */
      virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down,
          SimulationTaskHandle&, int taskId) {
        return false;
      }

      /**
       * Sets the taskId of the associated SimulationTask.
       * This method is called by the associated SimulationTask.
       * @param taskId of the associated SimulationTask
       */
      void setTaskId(int taskId) {
        this->taskId = taskId;
        if (taskId == 0)
          noGraphics = false;
        else
          noGraphics = true;
        // inform osg relevant stuff that no graphics is used
        osgHandle.noGraphics=noGraphics;
      }

      void setTaskNameSuffix(std::string nameSuffix) {
        windowName.append(nameSuffix);
      }

      /**
       * Sets the global SimulationTaskHandle. This method is
       * called by the associated SimulationTask.
       * @param simTaskHandle
       */
      void setSimTaskHandle(SimulationTaskHandle& simTaskHandle) {
        this->simTaskHandle = &simTaskHandle;
      }

    private:
      /**
       * Overwrite the usage of threads for ODE and OSG.
       * @see Simulation::processCmdLine(int arg, char** argv)
       */
      void processCmdLine(int argc, char** argv) {
        Simulation::processCmdLine(argc, argv);
        useOdeThread = false;
        useOsgThread = false;
        useQMPThreads = false;
        inTaskedMode = true;
        if (taskId!=0) {
          noGraphics = true;
          // inform osg relevant stuff that no graphics is used
          osgHandle.noGraphics=noGraphics;
        }
      }

      /**
       * Overwrite to avoid thread conflicts while
       * accessing the same file. Just disable it.
       */
      virtual bool storeCfg(const char* filenamestem, const std::list<std::string>& comments = std::list<std::string>()) {
        return true;
      }

      /**
       * Overwrite the usage of threads for ODE and OSG.
       * @see Configurable::restoreCFG(int arg, char** argv)
       */
      bool restoreCfg(const char* filenamestem) {
        bool result = Simulation::restoreCfg(filenamestem);
        useOdeThread = false;
        useOsgThread = false;
        useQMPThreads = false;
        inTaskedMode = true;
        if (taskId!=0) {
          noGraphics = true;
          // inform osg relevant stuff that no graphics is used
          osgHandle.noGraphics=noGraphics;
        }
        return result;
      }

      int taskId;
      SimulationTaskHandle* simTaskHandle;
      std::string nameSuffix;

      void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& globalData) {
        QMP_CRITICAL(61);
        start(odeHandle, osgHandle, globalData, *simTaskHandle, taskId);
        QMP_END_CRITICAL(61);
      }

      bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& globalData) {
        bool doRestart;
        QMP_CRITICAL(62);
        doRestart = restart(odeHandle, osgHandle, globalData, *simTaskHandle, taskId);
        QMP_END_CRITICAL(62);
        return doRestart;
      }

      virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
        QMP_CRITICAL(63);
        addCallback(globalData, draw, pause, control, *simTaskHandle, taskId);
        QMP_END_CRITICAL(63);
      }
      ;

      bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& globalData, int key, bool down) {
        return command(odeHandle, osgHandle, globalData, key, down, *simTaskHandle, taskId);
      }

  };

}

#endif /* _TASKEDSIMULATION_H_ */
