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

#include <ode-dbl/ode.h>
#include <selforg/stl_adds.h>
#include <signal.h>
#include <primitive.h>

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
    int laststate;
    sigset_t sigs;

    sigemptyset(&sigs);
    sigaddset(&sigs, SIGPIPE);
    sigaddset(&sigs, SIGABRT); // hack for invalid frees
    pthread_sigmask(SIG_BLOCK, &sigs, NULL);
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &laststate);

    argc = _argc;
    argv = _argv;
    //viewer = LpzRobotsViewer::getViewerInstance(*argc, argv);
    //parser = viewer->getArgumentParser();
    QP(PROFILER.init());
    //SimulationTaskHandle simTaskHandleCopy = *simTaskHandle;
    //QMP_SHARE(simTaskHandleCopy);
    //QMP_SHARE(parser);
    //QMP_SHARE(viewer);
    QMP_SHARE(argc);
    QMP_SHARE(argv);
    dInitODE();

    Primitive::setDestroyGeomFlag(false);
    QMP_PARALLEL_FOR(i, 0, simTaskList.size(),quickmp::INTERLEAVED){
      //QMP_USE_SHARED(simTaskHandleCopy, SimulationTaskHandle);
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
    // 20091023; guettler:
    // hack for tasked simulations; there are some problems if running in parallel mode,
    // if you do not destroy the geom, everything is fine (should be no problem because world is destroying geoms too)
    Primitive::setDestroyGeomFlag(true);
    // clean simTaskList
    simTaskList.clear();
  }

  void SimulationTaskSupervisor::setNumberThreads(int numberThreads)
  {
    if (numberThreads > 0)
      QMP_SET_NUM_THREADS(numberThreads);
  }

  void SimulationTaskSupervisor::setNumberThreadsPerCore(int numberThreadsPerCore)
  {
    if (numberThreadsPerCore > 0) // else ignore
      setNumberThreads(numberThreadsPerCore * QMP_GET_NUM_PROCS());
  }

  void SimulationTaskSupervisor::setSimTaskNameSuffix(std::string name) {
    nameSuffix = name;
  }
}

