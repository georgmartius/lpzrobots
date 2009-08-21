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
 *										   *
 *                                                                         *
 **************************************************************************/
#include "simulationtasksupervisor.h"
// simple profiling (only enabled if QPPOF is defined (Makefile))
#ifdef QPROF
#include "quickprof.h"
#define QP(x) x
#else
#define QP(x)
#endif
// simple multithread api
#include <selforg/quickmp.h>

#include <ode/ode.h>
#include <selforg/stl_adds.h>

using namespace std;

namespace lpzrobots {

  SimulationTaskSupervisor* SimulationTaskSupervisor::singletonInstance = 0;
  SimulationTaskHandle* SimulationTaskSupervisor::simTaskHandle = 0;
  TaskedSimulationCreator* SimulationTaskSupervisor::taskedSimCreator = 0;
  std::vector<SimulationTask*> SimulationTaskSupervisor::simTaskList;
  int* SimulationTaskSupervisor::argc = 0;
  char** SimulationTaskSupervisor::argv = 0;
  osg::ArgumentParser* SimulationTaskSupervisor::parser = 0;
  //LpzRobotsViewer*               SimulationTaskSupervisor::viewer=0;
  std::string SimulationTaskSupervisor::nameSuffix = "(tasked)";

  void SimulationTaskSupervisor::runSimTasks(int* _argc, char** _argv) {
    argc = _argc;
    argv = _argv;
    //viewer = LpzRobotsViewer::getViewerInstance(*argc, argv);
    //parser = viewer->getArgumentParser();
    QP(PROFILER.init());
    QMP_SHARE(simTaskHandle);
    //QMP_SHARE(parser);
    //QMP_SHARE(viewer);
    QMP_SHARE(argc);
    QMP_SHARE(argv);
    dInitODE();
    QMP_PARALLEL_FOR(i, 0, simTaskList.size(),quickmp::INTERLEAVED){
      QMP_USE_SHARED(simTaskHandle, SimulationTaskHandle*);
      //QMP_USE_SHARED(parser, osg::ArgumentParser*);
      //QMP_USE_SHARED(viewer, LpzRobotsViewer*);
      QMP_USE_SHARED(argc, int*);
      QMP_USE_SHARED(argv, char**);
      simTaskList[i]->startTask(*simTaskHandle, *taskedSimCreator, argc, argv, nameSuffix);
      delete (simTaskList[i]);
    }
    QMP_END_PARALLEL_FOR;
    QP(cout << "Profiling summary:" << endl << PROFILER.getSummary() << endl);
    QP(cout << endl << PROFILER.getSummary(quickprof::MILLISECONDS) << endl);
    QP(float timeSinceInit=PROFILER.getTimeSinceInit(quickprof::MILLISECONDS));
    QP(cout << endl << "total sum:      " << timeSinceInit << " ms"<< std::endl);
    // dCloseODE ();
    // clean simTaskList
    simTaskList.clear();
  }

  void SimulationTaskSupervisor::setNumberThreads(int numberThreads)
  {
    QMP_SET_NUM_THREADS(numberThreads);
  }

  void SimulationTaskSupervisor::setSimTaskNameSuffix(std::string name) {
    nameSuffix = name;
  }
}

