/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   $Log$
 *   Revision 1.92  2009-03-25 14:05:12  guettler
 *   bugfix: PlotOption reference was not direct available
 *
 *   Revision 1.91  2009/03/21 14:27:43  martius
 *   screen capturing compatible with both OSG 2.2 and higher
 *
 *   Revision 1.90  2009/03/12 08:44:01  martius
 *   fixed video recording
 *
 *   Revision 1.89  2009/01/20 22:41:19  martius
 *   manipulation of agents with the mouse implemented ( a dream... )
 *
 *   Revision 1.88  2008/09/16 14:46:01  martius
 *   redirected ODE output to a logfile ode.log
 *   this made the announcement counters and stuff obsolete
 *   changed some comments about the parallel stuff
 *
 *   Revision 1.87  2008/08/26 18:58:32  martius
 *   comments
 *
 *   Revision 1.86  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.85  2008/05/06 17:14:17  martius
 *   buildsystem further tuned,
 *   help in Makefile
 *   osg/data directory is also installed and registered at osg_robots
 *
 *   Revision 1.84  2008/05/05 09:35:35  guettler
 *   hud now displays if in pause mode
 *
 *   Revision 1.83  2008/05/02 17:20:04  martius
 *   *** empty log message ***
 *
 *   Revision 1.82  2008/04/30 10:04:51  guettler
 *   typo fix
 *
 *   Revision 1.81  2008/04/29 10:14:57  guettler
 *   -fixed interval bug (now its really correct)
 *
 *   Revision 1.80  2008/04/29 08:47:15  guettler
 *   test printout removed
 *
 *   Revision 1.79  2008/04/29 07:05:41  guettler
 *   fixed the bug that guiloggerinterval was not properly set when executing
 *   start with params -g -pause, same fixed for neuronviz and filelogging
 *
 *   Revision 1.78  2008/04/23 07:17:16  martius
 *   makefiles cleaned
 *   new also true realtime factor displayed,
 *    warning if out of sync
 *   drawinterval in full speed is 10 frames, independent of the speed
 *
 *   Revision 1.77  2008/04/18 14:00:09  guettler
 *   cosmetic changes, added some printouts
 *
 *   Revision 1.76  2008/04/18 10:38:15  guettler
 *   -extended profiling output
 *   -the OdeThread is now synchronized with one step delay for the
 *    WiredControllers (when using flag -odethread)
 *
 *   Revision 1.75  2008/04/17 15:59:00  martius
 *   OSG2 port finished
 *
 *   Revision 1.74.2.14  2008/04/17 15:04:55  martius
 *   shadow is 5 on default, also improved cmd line parsing of shadow and texsize
 *
 *   Revision 1.74.2.13  2008/04/17 14:19:55  martius
 *   cosmetic
 *
 *   Revision 1.74.2.12  2008/04/16 16:44:55  martius
 *   Viewer used in different threading models
 *    we cannot set SingleThread and open our own thread
 *
 *   Revision 1.74.2.11  2008/04/15 16:21:52  martius
 *   Profiling
 *   Multithreading also for OSG and ODE but disables because of instabilities
 *
 *   Revision 1.74.2.10  2008/04/14 11:25:30  guettler
 *   The OSG step (Viewer) runs now in a seperate thread! A Sideeffect is that
 *   the simulation runs one step out of sync with the ode, don't worry about
 *   that. This increases the simulation speed up to 30% on a test pc.
 *   Together with the parallelisation of the ODE we have an total speed up of
 *   94% with the shadowtype 5 on an dual Pentium3 1Ghz and NVidia5250!
 *
 *   Revision 1.74.2.9  2008/04/14 10:49:23  guettler
 *   The ODE simstep runs now in a parallel thread! A Sideeffect is that
 *   the simulation runs one step out of sync with the ode, don't worry about
 *   that. This increases the simulation speed up to 50% on a test pc.
 *
 *   Revision 1.74.2.8  2008/04/11 16:40:29  martius
 *   pthread option
 *
 *   Revision 1.74.2.7  2008/04/11 13:46:49  martius
 *   quickMP multithreading included
 *
 *   Revision 1.74.2.6  2008/04/11 10:41:34  martius
 *   config file added
 *
 *   Revision 1.74.2.5  2008/04/10 07:40:17  guettler
 *   Optimised parameters for the ShadowTechnique ParallelSplitShadowMap.
 *
 *   Revision 1.74.2.4  2008/04/09 14:25:35  martius
 *   shadow cmd line option
 *
 *   Revision 1.74.2.3  2008/04/09 13:57:59  guettler
 *   New ShadowTechnique added.
 *
 *   Revision 1.74.2.2  2008/04/09 10:18:41  martius
 *   fullscreen and window options done
 *   fonts on hud changed
 *
 *   Revision 1.74.2.1  2008/04/08 14:09:23  martius
 *   compiles and runs with OSG2.2. Juhu
 *
 *   Revision 1.74  2007/12/11 14:11:35  martius
 *   addcallback draw flag was wrong
 *
 *   Revision 1.73  2007/12/06 14:52:43  martius
 *   mouse grasp started
 *
 *   Revision 1.72  2007/12/06 10:30:52  der
 *   the TV mode is now the default camera mode!
 *
 *   Revision 1.71  2007/11/07 13:13:01  martius
 *   sound signals
 *   draw at the end of drawinterval
 *   FOREACH used more, indentation
 *
 *   Revision 1.70  2007/09/28 12:31:49  robot3
 *   The HUDSM is not anymore deduced from StatisticalTools, so the statistics
 *   can be updated independently from the HUD
 *   addPhysicsCallbackable and addGraphicsCallbackable now exists in Simulation
 *
 *   Revision 1.69  2007/09/27 10:47:04  robot3
 *   mathutils: moved abs to selforg/stl_adds.h
 *   simulation,base: added callbackable support,
 *   added WSM (WindowStatisticsManager) funtionality
 *
 *   Revision 1.68  2007/08/29 13:08:26  martius
 *   added HUD with time and caption
 *
 *   Revision 1.67  2007/08/24 11:52:18  martius
 *   timer reset on key stroke handling
 *   different substance callback handling
 *
 *   Revision 1.66  2007/07/31 08:20:33  martius
 *   list of spaces for internal collision detection added
 *
 *   Revision 1.65  2007/07/19 15:54:55  martius
 *   fixme added
 *
 *   Revision 1.64  2007/07/03 13:09:32  martius
 *   odeHandle knows about time
 *
 *   Revision 1.63  2007/06/21 16:19:59  martius
 *   -nopgraphics option which disables graphics rendering
 *
 *   Revision 1.62  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.61  2007/05/09 14:57:25  robot3
 *   to increase or reduce the simulation speed (realtimefactor), use + and -
 *   to toggle to the maximum simulation speed, use * in the simulation
 *
 *   Revision 1.60  2007/04/20 12:18:56  martius
 *   help output
 *
 *   Revision 1.59  2007/03/26 13:06:19  martius
 *   use new commandline interface
 *
 *   Revision 1.58  2007/03/16 10:55:03  martius
 *   new nearcallback structure
 *   Toplevel nearcallback uses robots collision control
 *   normal nearcallback uses new substance implementation
 *
 *   Revision 1.57  2007/03/05 17:54:24  martius
 *   soundMan with parameters
 *
 *   Revision 1.56  2007/02/27 11:56:40  robot5
 *   Minor changes for SoundMan functionalities.
 *
 *   Revision 1.55  2007/02/12 13:29:40  martius
 *   addCallback has flag about controllstep
 *
 *   Revision 1.54  2006/12/13 09:13:03  martius
 *   agents get comments about changed parameter for logfile
 *
 *   Revision 1.53  2006/12/11 18:31:34  martius
 *   list of configurables for agents
 *   reference counting and memleaks fixed
 *   onlycontrol used in steps where  controller is not used
 *
 *   Revision 1.52  2006/11/30 10:06:41  robot5
 *   Added support for Sndchanger (experimental). Startup with argument -s.
 *
 *   Revision 1.51  2006/10/20 14:24:55  martius
 *   max velocity for joint-correction limited
 *
 *   Revision 1.50  2006/09/22 10:57:36  martius
 *   again hard collisions
 *
 *   Revision 1.49  2006/09/21 22:09:01  martius
 *   timeleak is seldom annouced
 *
 *   Revision 1.48  2006/09/21 16:17:27  der
 *   different friction because of terrain
 *
 *   Revision 1.47  2006/09/21 16:01:48  martius
 *   relaxed collisions
 *
 *   Revision 1.46  2006/09/20 15:30:40  martius
 *   shadowsize
 *
 *   Revision 1.45  2006/09/20 12:55:09  martius
 *   ERP = 0.999 and CFM=0.0001
 *
 *   Revision 1.44  2006/08/30 09:00:19  martius
 *   code for full collision control inserted (as a comment)
 *
 *   Revision 1.43  2006/08/04 15:07:46  martius
 *   documentation
 *
 *   Revision 1.42  2006/07/14 15:17:33  fhesse
 *   start option for intended simulation time added
 *   -simtime [min]
 *
 *   Revision 1.40.4.34  2006/07/10 12:03:47  martius
 *   Cosmetics
 *
 *   Revision 1.40.4.33  2006/06/29 16:41:14  robot3
 *   you can now see the bounding shapes of meshes
 *   if you type ./start -drawboundings on command line
 *
 *   Revision 1.40.4.32  2006/06/29 16:31:47  robot3
 *   includes cleared up
 *
 *   Revision 1.40.4.31  2006/06/25 16:52:23  martius
 *   filelogging is done with a plotoption
 *
 *   Revision 1.40.4.30  2006/05/28 22:11:44  martius
 *   - noshadow cmdline flag
 *
 *   Revision 1.40.4.29  2006/05/15 13:07:35  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.40.4.28  2006/05/08 12:12:20  robot3
 *   changes from Revision 1.40.4.27 reverted
 *
 *   Revision 1.40.4.27  2006/04/27 16:31:35  robot3
 *   -motionblur inlucded
 *   -if the simulation is not in videoRedordingMode,
 *    50fps are now as standard used.
 *
 *   Revision 1.40.4.26  2006/03/29 15:07:29  martius
 *   Dummy Primitive for Environment
 *
 *   Revision 1.40.4.25  2006/03/19 13:38:08  robot3
 *   cameramanipulator for race mode included
 *
 *   Revision 1.40.4.24  2006/03/08 11:21:20  robot3
 *   Following Cameramanipulator is now on key 2
 *
 *   Revision 1.40.4.23  2006/03/06 16:53:49  robot3
 *   now ExtendedViewer is used because of the new getCurrentCameraManipulator(),
 *   code optimizations
 *
 *   Revision 1.40.4.22  2006/03/04 15:04:33  robot3
 *   cameramanipulator is now updated with every draw intervall
 *
 *   Revision 1.40.4.21  2006/03/03 12:11:32  robot3
 *   neccessary changes made for new cameramanipulators
 *
 *   Revision 1.40.4.20  2006/02/22 15:26:23  martius
 *   frame grabbing with osg works again
 *
 *   Revision 1.40.4.19  2006/02/20 10:50:20  martius
 *   pause, random, windowsize, Ctrl-keys
 *
 *   Revision 1.40.4.18  2006/02/14 17:36:03  martius
 *   prevent overflow in time sync
 *
 *   Revision 1.40.4.17  2006/02/14 17:31:12  martius
 *   much better time syncronisation
 *
 *   Revision 1.40.4.16  2006/02/01 14:00:32  martius
 *   remerging of non-fullscreen start
 *
 *   Revision 1.40.4.15  2006/02/01 10:24:34  robot3
 *   new camera manipulator added
 *
 *   Revision 1.40.4.13  2006/01/12 22:32:51  martius
 *   key eventhandler integrated
 *
 *   Revision 1.40.4.12  2006/01/12 15:16:53  martius
 *   transparency
 *
 *   Revision 1.40.4.11  2005/12/29 16:49:48  martius
 *   end is obsolete
 *   tidyUp is used for deletion
 *
 *   Revision 1.40.4.10  2005/12/29 12:54:19  martius
 *   multiple tesselhints
 *
 *   Revision 1.40.4.9  2005/12/15 17:02:04  martius
 *   light is in sky and standart cams removed
 *   config has a default implentation now
 *
 *   Revision 1.40.4.8  2005/12/14 15:36:25  martius
 *   do not check for unused vars
 *
 *   Revision 1.40.4.7  2005/12/13 18:10:33  martius
 *   changelog
 *
 *   Revision 1.40.4.6  2005/12/11 23:35:07  martius
 *   *** empty log message ***
 *
 *   Revision 1.40.4.5  2005/12/09 16:53:16  martius
 *   camera is working now
 *
 *   Revision 1.40.4.4  2005/12/06 17:38:13  martius
 *   *** empty log message ***
 *
 *   Revision 1.40.4.3  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.40.4.2  2005/11/15 12:29:14  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.40.4.1  2005/11/14 17:37:00  martius
 *   changed makefile structure to have and include directory
 *   mode to selforg
 *
 *   Revision 1.40  2005/11/10 09:20:48  martius
 *   10 minute notification
 *
 *   Revision 1.39  2005/10/21 11:59:58  martius
 *   realtimefactor written small
 *
 *   Revision 1.38  2005/10/18 15:31:32  fhesse
 *   one comment improved
 *
 *   Revision 1.37  2005/10/06 17:11:26  martius
 *   switched to stl lists
 *
 *   Revision 1.36  2005/09/27 13:59:03  martius
 *   doInternals after control
 *
 *   Revision 1.35  2005/09/23 09:55:16  martius
 *   odeConfig gets OdeHandle explicit
 *
 *   Revision 1.34  2005/09/22 13:17:11  martius
 *   OdeHandle and GlobalData finished
 *   doInternalStuff included
 *
 *   Revision 1.33  2005/09/22 11:21:57  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.32  2005/09/20 10:54:34  robot3
 *   camera module:
 *   -pressing key c now centers on focused robot
 *   -pressing key b now moves 5.0f behind the robot
 *   -tiny bugfixing (nullpointer crashes etc.)
 *
 *   Revision 1.31  2005/09/19 16:01:58  martius
 *   use dsSetSimulationTime
 *
 *   Revision 1.30  2005/09/13 15:36:38  martius
 *   disabled advanced modes
 *   new grabframe interface used
 *
 *   Revision 1.29  2005/09/13 13:26:00  martius
 *   random seed adjustable
 *   usage with -h
 *
 *   Revision 1.28  2005/09/11 15:16:41  martius
 *   fast frame capturing enabled
 *
 *   Revision 1.27  2005/09/02 17:18:15  martius
 *   camera modes changed
 *
 *   Revision 1.26  2005/08/25 07:36:16  fhesse
 *   unistd.h included to be able to use usleep
 *
 *   Revision 1.25  2005/08/23 11:39:31  robot1
 *   advancedFollowing mode integrated (for switching)
 *
 *   Revision 1.24  2005/08/22 20:34:22  martius
 *   Display robot name when looking at it.
 *   Init robot view always
 *
 *   Revision 1.23  2005/08/22 12:41:05  robot1
 *   advancedTV mode integrated (for switching)
 *
 *   Revision 1.22  2005/08/16 10:09:07  robot1
 *   tiny bugfixing
 *
 *   Revision 1.21  2005/08/12 12:43:57  robot1
 *   command for switching between agents implemented
 *
 *   Revision 1.20  2005/08/12 11:55:01  robot1
 *   camera module integrated
 *
 *   Revision 1.19  2005/08/03 20:34:39  martius
 *   basic random number initialisation
 *   contains returns index instead of bool
 *
 *   Revision 1.18  2005/07/29 10:22:47  martius
 *   drawInterval honored
 *   real time syncronisation
 *
 *   Revision 1.17  2005/07/27 13:23:16  martius
 *   new color and position construction
 *
 *   Revision 1.16  2005/07/21 12:18:43  fhesse
 *   window size 640x480
 *
 *   Revision 1.15  2005/07/18 08:35:21  martius
 *   drawcallback is additionalcallback now
 *
 *   Revision 1.14  2005/07/15 11:35:52  fhesse
 *   added parameter gravity
 *
 *   Revision 1.13  2005/07/13 08:39:21  robot8
 *   added the possibility to use an additional command function, which handels special Inputs if the ODE simulation window has the focus
 *
 *   Revision 1.12  2005/07/11 11:19:38  robot8
 *   adding the line, where the pointer to the additional draw function is set to the value of the parameter drawCallback
 *
 *   Revision 1.11  2005/07/08 10:14:05  martius
 *   added contains (helper for stringlist search)
 *
 *   Revision 1.10  2005/07/07 10:23:36  martius
 *   added user draw callback
 *
 *   Revision 1.9  2005/06/30 13:23:38  robot8
 *   completing the call of the dynamic collisionCallback-function for  standard collisions
 *
 *   Revision 1.8  2005/06/29 09:27:03  martius
 *   *** empty log message ***
 *
 *   Revision 1.7  2005/06/29 09:25:17  martius
 *   customized callback for collision
 *
 *   Revision 1.6  2005/06/22 15:39:49  fhesse
 *   path to textures
 *
 *   Revision 1.5  2005/06/20 10:03:26  fhesse
 *   collision treatment by agents included
 *
 *   Revision 1.4  2005/06/17 09:33:53  martius
 *   aligned values on showParams
 *
 *   Revision 1.3  2005/06/15 14:01:31  martius
 *   moved all general code from main to simulation
 *                                                                 *
 ***************************************************************************/
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <selforg/abstractcontroller.h>
#include <selforg/abstractwiring.h>

#include <selforg/callbackable.h>

#include "simulation.h"
#include "odeagent.h"
#include "console.h"

#include <osg/Version>
#include <osg/ShapeDrawable>
#include <osg/ArgumentParser>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osgUtil/SceneView>
// #include <osgUtil/Optimizer>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgGA/StateSetManipulator>

#include "primitive.h"
#include "abstractobstacle.h"

#include "cameramanipulatorTV.h"
#include "cameramanipulatorFollow.h"
#include "cameramanipulatorRace.h"
#include "motionblurcallback.h"

// simple multithread api
#include "quickmp.h"
// simple profiling (only enabled if QPPOF is defined (Makefile))
#ifdef QPROF
#include "quickprof.h"
#define QP(x) x
#else
#define QP(x)
#endif

#include <pthread.h>
#include "odeconfig.h"

/// read the installation PREFIX (to find data directory)
#include "install_prefix.conf"

namespace lpzrobots {

  using namespace std;
  using namespace osg;
  using namespace osgViewer;
  using namespace osgUtil;

  // forward declaration of static functions
  static void* odeStep_run(void* p);
  static void* osgStep_run(void* p);
  static FILE* ODEMessageFile = 0; // file handler for ODE messages
  static void printODEMessage (int num, const char *msg, va_list ap);

  int Simulation::ctrl_C = 0;

  Simulation::Simulation()
    : Configurable("lpzrobots-ode_robots", "0.4"){
    // default values are set in Base::Base()
    addParameter("Shadow",&shadow);
    addParameter("ShadowTextureSize",&shadowTexSize);
    addParameter("UseNVidia",&useNVidia);
    addParameterDef("WindowWidth",&windowWidth,640);
    addParameterDef("WindowHeight",&windowHeight,480);
    addParameterDef("UseOdeThread",&useOdeThread,0);
    addParameterDef("UseOsgThread",&useOsgThread,0);

//     nextLeakAnnounce = 20;
//     leakAnnCounter = 1;
    truerealtimefactor = 1;
    sim_step = 0;
    state    = none;
    pause    = false;
    noGraphics=false;
    simulation_time=-1;
    simulation_time_reached=false;
    viewer   = 0;
    arguments= 0;
    // we have to count references by our selfes
    osg::Referenced::ref();
    osgGA::GUIEventHandler::ref();
    //    Producer::Camera::Callback::ref();
    odeThreadCreated=false;
    osgThreadCreated=false;

    videostream = new VideoStream();

  }

  Simulation::~Simulation() {
    if(state!=running)
      return;
    dJointGroupDestroy ( odeHandle.jointGroup );
    dWorldDestroy ( odeHandle.world );
    dSpaceDestroy ( odeHandle.space );
    dCloseODE ();

    state=closed;
    if(arguments)
      delete arguments;
    // we have to count references by our selfes
    osgGA::GUIEventHandler::unref();
    //    Producer::Camera::Callback::unref_nodelete();
    //    Producer::Camera::Callback::unref_nodelete();
    osg::Referenced::unref_nodelete();
  }

  bool Simulation::init(int argc, char** argv) {

    /**************** ODE-Section   ***********************/
    odeHandle.init(&globalData.time);
    // redirect ODE messages to our print function (writes into file ode.msg)
    dSetMessageHandler(printODEMessage);

    globalData.odeConfig.setOdeHandle(odeHandle);

    //set Gravity to Earth level
    dWorldSetGravity ( odeHandle.world , 0 , 0 , globalData.odeConfig.gravity );
    dWorldSetERP ( odeHandle.world , 0.3 );
    dWorldSetCFM ( odeHandle.world,1e-4);

    dWorldSetContactMaxCorrectingVel (odeHandle.world, 100); // default is infinity
    dWorldSetContactSurfaceLayer (odeHandle.world, 0.001); // default is 0

    cmd_handler_init();

    globalData.environment = new DummyPrimitive();

    // add ode config to config list
    globalData.configs.push_back(&(globalData.odeConfig));
    globalconfigurables.push_back(&(globalData.odeConfig));

    /**************** OpenSceneGraph-Section   ***********************/

    osgDB::FilePathList l = osgDB::getDataFilePathList();
#ifdef PREFIX
    l.push_back(PREFIX+string("/share/lpzrobots/data"));// installation path
#else
    l.push_back("../../osg/data");
#endif
    l.push_back("data");
    osgDB::setDataFilePathList(l);

    // load config file (first in the current directory and then in ~/.lpzrobots/)
    sprintf(odeRobotsCfg,"ode_robots");
    if(!restoreCfg(odeRobotsCfg)){
      const char* home = getenv("HOME");
      if(!home){
	fprintf(stderr,"Cannot determine HOME directory!");
      } else {
	sprintf(odeRobotsCfg,"%s/.lpzrobots/ode_robots",home);
	if(!restoreCfg(odeRobotsCfg)){
	  // create directory
	  char dir[1024];
	  sprintf(dir,"%s/.lpzrobots",home);
	  mkdir(dir,0x755);
	  storeOdeRobotsCFG();
	}
      }
    }
    // process cmdline (possibly overwrite values from cfg file
    processCmdLine(argc, argv);

    if(!noGraphics) {
      // create fake command line options to make osg do what we want
      insertCmdLineOption(argc, argv);
      // use an ArgumentParser object to manage the program arguments.
      arguments = new ArgumentParser(&argc, argv);

      // set up the usage document, in case we need to print out how to use this program.
      arguments->getApplicationUsage()->setApplicationName(arguments->getApplicationName() );
      arguments->getApplicationUsage()->setDescription(
	       "Lpzrobots Simulator, <robot.informatik.uni-leipzig.de>");
      arguments->getApplicationUsage()->setCommandLineUsage(arguments->getApplicationName() );
      arguments->getApplicationUsage()->addCommandLineOption(
	       "-h or --help", "Display this information");
      // if user request help write it out to cout.
      if (arguments->read("-h") || arguments->read("--help")) {
	arguments->getApplicationUsage()->write(std::cout);
	return false;
      }
      // any option left unread are converted into errors to write out later.
      //    arguments->reportRemainingOptionsAsUnrecognized();

      // report any errors if they have occured when parsing the program aguments.
      if (arguments->errors()) {
	arguments->writeErrorMessages(std::cout);
	return false;
      }

      // construct the viewer.
      viewer = new Viewer(*arguments);
      if(useOsgThread!=0){
	viewer->setThreadingModel(Viewer::CullDrawThreadPerContext);
      }else{
	viewer->setThreadingModel(Viewer::SingleThreaded);
      }

      // add the state manipulator
      viewer->addEventHandler( new osgGA::StateSetManipulator(viewer->getCamera()->getOrCreateStateSet()) );

      // add the thread model handler
      viewer->addEventHandler(new osgViewer::ThreadingHandler);

      // add the window size toggle handler
      viewer->addEventHandler(new osgViewer::WindowSizeHandler);

      // add the stats handler
      viewer->addEventHandler(new osgViewer::StatsHandler);

      // add the help handler
      viewer->addEventHandler(new osgViewer::HelpHandler(arguments->getApplicationUsage()));

      // add the record camera path handler
      viewer->addEventHandler(new osgViewer::RecordCameraPathHandler);

      // add the record camera path handler
      viewer->addEventHandler(this);
      // add callback for video recording
    #if OPENSCENEGRAPH_MAJOR_VERSION == 2 &&  OPENSCENEGRAPH_MINOR_VERSION <= 2
      viewer->getCamera()->setPostDrawCallback(videostream.get());
    #else
      viewer->getCamera()->setFinalDrawCallback(videostream);
    #endif
    }else{
      globalData.odeConfig.realTimeFactor=0;
    }

    // information on terminal, created with figlet.
    // See also logo.txt, we had to quote all backslashes
    printf ("%s\n",
"+----------------------------------------------------------------+\n\
|   _     ____ _________       _           _                     |\n\
|  | |   |  _ \\__  /  _ \\ ___ | |__   ___ | |_ ___               |\n\
|  | |   | |_) |/ /| |_) / _ \\| '_ \\ / _ \\| __/ __|              |\n\
|  | |___|  __// /_|  _ < (_) | |_) | (_) | |_\\__ \\              |\n\
|  |_____|_|  /____|_| \\_\\___/|_.__/ \\___/ \\__|___/              |\n\
|                                                                |\n\
| lpzRobots simulator, http://robot.informatik.uni-leipzig.de    |\n\
+----------------------------------------------------------------+" );
    printf ( "Press Ctrl-C on the console for a commandline interface.\n" );
    printf ( "Press h      on the graphics window for help).\n\n" );
    printf ( "Random number seed: %li\n", globalData.odeConfig.randomSeed);

    for(int i=0; i<3; i++) {
      osgHandle.tesselhints[i] = new TessellationHints();
      osgHandle.tesselhints[i]->ref();
    }
    osgHandle.tesselhints[0]->setDetailRatio(0.1f); // Low
    osgHandle.tesselhints[1]->setDetailRatio(1.0f); // Middle
    osgHandle.tesselhints[2]->setDetailRatio(3.0f); // High

    osgHandle.color = Color(1,1,1,1);

    osgHandle.scene=makeScene();
    if (!osgHandle.scene)
      return false;

    osgHandle.normalState = new StateSet();
    osgHandle.normalState->ref();

    // set up blending for transparent stateset
    osg::StateSet* stateset = new StateSet();
    osg::BlendFunc* transBlend = new osg::BlendFunc;
    transBlend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
    stateset->setAttributeAndModes(transBlend, osg::StateAttribute::ON);
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //stateset->setRenderBinDetails(5,"RenderBin");
    stateset->setMode(GL_CULL_FACE,osg::StateAttribute::ON); // disable backface because of problems
    osgHandle.transparentState = stateset;
    osgHandle.transparentState->ref();

    if(!noGraphics) {
      keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

      // setup the camera manipulators
      keyswitchManipulator->addMatrixManipulator( '1', "Static", new CameraManipulator(osgHandle.scene, globalData) );
      keyswitchManipulator->addMatrixManipulator( '2', "Follow", new CameraManipulatorFollow(osgHandle.scene, globalData) );
      keyswitchManipulator->addMatrixManipulator( '3', "TV", new CameraManipulatorTV(osgHandle.scene, globalData) );
      keyswitchManipulator->addMatrixManipulator( '4', "Race", new CameraManipulatorRace(osgHandle.scene, globalData) );

      keyswitchManipulator->selectMatrixManipulator(2);
      viewer->setCameraManipulator( keyswitchManipulator );

      // get details on keyboard and mouse bindings used by the viewer.
      viewer->getUsage(*(arguments->getApplicationUsage()));
    }

    state=initialised;
    return true;
  }


  bool Simulation::run(int argc, char** argv) {


    if(!init(argc, argv))
      return false;

    initializeConsole();
    QP(PROFILER.init());

    //********************Simulation start*****************
    state=running;
    globalData.time=0;
    resetSyncTimer();

    start(odeHandle, osgHandle, globalData);

    if(!noGraphics) {
      // optimize the scene graph, remove redundant nodes and state etc.
      // osgUtil::Optimizer optimizer;
      // optimizer.optimize(root);


      // add model to viewer.
      viewer->setSceneData(root);

      // create the windows and run the threads.
      viewer->realize();

      // set title
      osgViewer::Viewer::Windows windows;
      viewer->getWindows(windows);
      assert(windows.size()>0);

      // set our motion blur callback as the draw operator on each window
      FOREACH(osgViewer::Viewer::Windows, windows, itr){
	if(globalData.odeConfig.motionPersistence > 0)
	  (*itr)->add(new MotionBlurOperation(globalData));
	(*itr)->setWindowName("Lpzrobots - Selforg");
      }
    }

    if(!noGraphics) {
      while ( (!viewer->done()) && (!simulation_time_reached) ) {
	if(!loop())
	  break;
      }
    } else {
      while ( !simulation_time_reached) {
	if(!loop())
	  break;
      }
    }
    if(useOdeThread!=0) pthread_join (odeThread, NULL);
    if(useOsgThread!=0) pthread_join (osgThread, NULL);
    closeConsole();
    end(globalData);
    tidyUp(globalData);
    return true;

  }

  bool Simulation::config(GlobalData& globalData) {
    return handleConsole(globalData);
  }

  void Simulation::end(GlobalData& globalData) {}

  bool Simulation::loop() {
    // we run the physical simulation as often as "drawinterval",
    //  the drawing of all object should occur if t==0
    bool run=true;
    for(int t = 0; t < globalData.odeConfig.drawInterval; t++) {
      // Parametereingabe
      if (control_c_pressed()) {
	cmd_begin_input();
	run=config(globalData);
	cmd_end_input();
	resetSyncTimer();
      }

      // the simulation just runs if pause is not enabled
      if (!pause) {
	// increase time
	globalData.time += globalData.odeConfig.simStepSize;
	sim_step++;
	// print simulation time every 10 min.
	if(noGraphics &&
	   sim_step % long(600.0/globalData.odeConfig.simStepSize) ==0) {
	  printf("Simulation time: %li min\n",
		 sim_step/ long(60/globalData.odeConfig.simStepSize));
	}
	// finish simulation, if intended simulation time is reached
	if(simulation_time!=-1) { // check time only if activated
	  if( (sim_step/ ( long(1/globalData.odeConfig.simStepSize)*60))  == simulation_time) {
	    if (!simulation_time_reached) { // print out once only
	      printf("%li min simulation time reached -> simulation stopped \n", simulation_time);
	    }
	    simulation_time_reached=true;
	  }
	}


// 	SEQUENCIAL VERSION
// 	// for all agents: robots internal stuff and control step if at controlInterval
// 	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++) {
// 	  if ( (sim_step % globalData.odeConfig.controlInterval ) == 0 ) {
// 	    (*i)->step(globalData.odeConfig.noise, globalData.time);
// 	    (*i)->getRobot()->doInternalStuff(globalData);
// 	  } else {
// 	    (*i)->onlyControlRobot();
// 	  }
// 	}

// 	PARALLEL VERSION
	if ( (sim_step % globalData.odeConfig.controlInterval ) == 0 ) {
	  QP(PROFILER.beginBlock("controller                   "));
	  QMP_SHARE(globalData);
	  // there is a problem with the useOdeThread in the loop (not static)
    	  if (useOdeThread!=0) {
  	    QMP_PARALLEL_FOR(i, 0, globalData.agents.size(),quickmp::INTERLEAVED){
    	      QMP_USE_SHARED(globalData, GlobalData);
              globalData.agents[i]->stepOnlyWiredController(globalData.odeConfig.noise, globalData.time);
  	    }
  	    QMP_END_PARALLEL_FOR;
  	  } else {
  	    QMP_PARALLEL_FOR(i, 0, globalData.agents.size(),quickmp::INTERLEAVED){
	      QMP_USE_SHARED(globalData, GlobalData);
    	      globalData.agents[i]->step(globalData.odeConfig.noise, globalData.time);
	    }
	    QMP_END_PARALLEL_FOR;
   	  }
	  QP(PROFILER.endBlock("controller                   "));
	}else{ // serial execution is sufficient here
	  FOREACH(OdeAgentList, globalData.agents, i) {
	    (*i)->onlyControlRobot();
	  }
	}

	/****************** Simulationstep *****************/
	if(useOdeThread!=0){
	  if (odeThreadCreated)
	    pthread_join (odeThread, NULL);
	  else odeThreadCreated=true;
   	}
	// Do this here because it
	// can provide collision handling (old style collision handling)
	// and this crashes in parallel version
	QP(PROFILER.beginBlock("internalstuff_and_addcallback"));
	FOREACH(OdeAgentList, globalData.agents, i) {
	  if (useOdeThread!=0)
	    (*i)->setMotorsGetSensors();
	  (*i)->getRobot()->doInternalStuff(globalData);
	}
	addCallback(globalData, t==(globalData.odeConfig.drawInterval-1), pause,
		    (sim_step % globalData.odeConfig.controlInterval ) == 0);
	QP(PROFILER.endBlock("internalstuff_and_addcallback"));

	// manipulate agents (with mouse)
	if(!noGraphics){
	  videostream->pause = pause;
	  osgGA::MatrixManipulator* mm =
	    keyswitchManipulator->getCurrentMatrixManipulator();
	  if(mm) {
	    CameraManipulator* cm = dynamic_cast<CameraManipulator*>(mm);
	    if(cm) cm->manipulateAgent(osgHandle);
	  }
	}

	if(useOdeThread!=0)
	  pthread_create (&odeThread, NULL, odeStep_run,this);
	else
	  odeStep();

 	// call all registered physical callbackable classes
	// SEQUENCIAL
	// 	FOREACH(vector<Callbackable*>, physicsCallbackables, i) {
	// 	  (*i)->doOnCallBack();
	// 	}
	// PARALLEL
	unsigned int pcaSize=physicsCallbackables.size();
        QP(PROFILER.beginBlock("physicsCB                    "));
	if(pcaSize==1){
	  physicsCallbackables.front()->doOnCallBack();
	}else if (pcaSize>1){
	  QMP_SHARE(physicsCallbackables);
	  QMP_PARALLEL_FOR(i, 0, pcaSize){
	    QMP_USE_SHARED(physicsCallbackables, vector<Callbackable*>);
	    physicsCallbackables[i]->doOnCallBack();
	  }
	  QMP_END_PARALLEL_FOR;
	}
	QP(PROFILER.endBlock("physicsCB                    "));

	// remove old signals from sound list
	globalData.sounds.remove_if(Sound::older_than(globalData.time));
      }

      if(t==(globalData.odeConfig.drawInterval-1) && !noGraphics) {
	if(useOsgThread!=0){
	  QP(PROFILER.beginBlock("graphics aync"));
	  if (osgThreadCreated)
	    pthread_join (osgThread, NULL);
	  else osgThreadCreated=true;
	  QP(PROFILER.endBlock("graphics aync"));
	}
	QP(PROFILER.beginBlock("graphicsUpdate               "));
	/************************** Update the scene ***********************/
	FOREACH(ObstacleList, globalData.obstacles, i) {
	  (*i)->update();
	}
	FOREACH(OdeAgentList, globalData.agents, i) {
	  (*i)->getRobot()->update();
	}
	// draw sound blobs
	if(!globalData.sounds.empty()){
	  FOREACH(SoundList, globalData.sounds, i){
	    i->render(osgHandle);
	  }
	}

	// update the camera
	osgGA::MatrixManipulator* mm = keyswitchManipulator->getCurrentMatrixManipulator();
	if(mm) {
	  CameraManipulator* cameramanipulator = dynamic_cast<CameraManipulator*>(mm);
	  if(cameramanipulator)
	    cameramanipulator->update();
	}
	// update timestats
	setTimeStats(globalData.time,globalData.odeConfig.realTimeFactor,
		     truerealtimefactor,pause);

	// call all registered graphical callbackable classes
	FOREACH(list<Callbackable*>, graphicsCallbackables, i) {
	  (*i)->doOnCallBack();
	}
        QP(PROFILER.endBlock("graphicsUpdate               "));

	//        onPostDraw(*(viewer->getCamera()));*/
        if(useOsgThread!=0){
	  pthread_create (&osgThread, NULL, osgStep_run,this);
	}else{
  	QP(PROFILER.beginBlock("graphics                     "));
	  osgStep();
	  QP(PROFILER.endBlock("graphics                     "));
	}

      }

    }
    /************************** Time Syncronisation ***********************/
    // Time syncronisation of real time and simulations time
    long elapsed = timeOfDayinMS() - realtimeoffset;
    // simulation speed (calculates more precise again if not pause or max speed)
    if(!pause) truerealtimefactor = (globalData.time*1000.0 - simtimeoffset)/(elapsed+1);
    // get refresh rate of about 10 frames in full speed
    if(globalData.odeConfig.realTimeFactor==0.0){
      globalData.odeConfig.calcAndSetDrawInterval(10,truerealtimefactor);
    }
    if(globalData.odeConfig.realTimeFactor!=0.0 && !pause) {
      // difference between actual time and current time in milliseconds
       long diff = long((globalData.time*1000.0 - simtimeoffset)
		       / globalData.odeConfig.realTimeFactor  ) - elapsed;
      if(diff > 10000 || diff < -10000){ // check for overflow or other weird things
	resetSyncTimer();
      }else {
	if(diff > 4) { // if less the 3 milliseconds we don't call usleep since it needs time
	  usleep((diff-2)*1000);
	  //	  nextLeakAnnounce=100;
	}
      }
      // the video steam should look perfectly syncronised
      if(videostream->isOpen())
	truerealtimefactor=globalData.odeConfig.realTimeFactor;
      else{
	//  true speed of simulations. In case resetSyncTimer() was just called this
	//  gives wrong values, thats why we test on elapsed
	if(!justresettimes)
	  truerealtimefactor = (globalData.time*1000.0 - simtimeoffset)/(elapsed+max(1l,diff));
      }
      justresettimes=false;
    } else if (pause) {
      usleep(10000);
    }


    return run;
  }

  bool Simulation::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&) {
    bool handled = false;
    switch(ea.getEventType()) {
    case(osgGA::GUIEventAdapter::KEYDOWN): {
      handled = command(odeHandle, osgHandle, globalData, ea.getKey(), true);
      if(handled) {
	break;
      }
      //	printf("Key: %i\n", ea.getKey());
      switch(ea.getKey()) {
      case 6 : // Ctrl - f
	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++) {
	  if(!(*i)->removePlotOption(File)) {
		  PlotOption po(File, Controller, filelogginginterval, globalconfigurables);
	    (*i)->addPlotOption(po);
	  }
	}
	handled= true;
	break;
      case 7 : // Ctrl - g
	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++) {
	  if(!(*i)->removePlotOption(GuiLogger)) {
		  PlotOption po(GuiLogger, Controller, guiloggerinterval, globalconfigurables);
	    (*i)->addPlotOption(po);
	  }
	}
	handled=true;
	break;
      case 65450: // keypad *  // normal * is allready used by LOD
	globalData.odeConfig.setParam("realtimefactor", 0);
	std::cout << "realtimefactor = " << globalData.odeConfig.getParam("realtimefactor") << std::endl;
	handled=true;
	break;
      case 65451: // keypad +
      case 43: // +
	{
	  double rf = globalData.odeConfig.realTimeFactor;
	  if (rf >= 2)
	    globalData.odeConfig.setParam("realtimefactor", rf+1);
	  else if (rf>=1.0)
	    globalData.odeConfig.setParam("realtimefactor", rf+0.25);
	  else if (rf>=0.1)
	    globalData.odeConfig.setParam("realtimefactor", rf+0.1);
	  else
	    globalData.odeConfig.setParam("realtimefactor", 0.1);
	  std::cout << "realtimefactor = " <<  globalData.odeConfig.getParam("realtimefactor")<< std::endl;
	  handled=true;
	}
	break;
      case 65453: // keypad -
      case 45: // -
	{
	  double rf = globalData.odeConfig.realTimeFactor;
	  if (rf>2)
	    globalData.odeConfig.setParam("realtimefactor", rf-1);
	  else if (rf>1.0)
	    globalData.odeConfig.setParam("realtimefactor", rf-0.25);
	  else if (rf>0.1)
	    globalData.odeConfig.setParam("realtimefactor", rf-0.1);
	  else
	    globalData.odeConfig.setParam("realtimefactor", 0.1);
	  std::cout << "realtimefactor = " <<  globalData.odeConfig.getParam("realtimefactor")<< std::endl;
	  handled=true;
	}
	break;
      case 18:  // Ctrl - r
	if(videostream->isOpen()) {
	  printf("Stop video recording!\n");
	  videostream->close();
	  //	    printf("Switching back to 50fps!\n");
	  globalData.odeConfig.videoRecordingMode=false;
	} else {
	  //	    printf("For video recording the simulation now switches to 25fps!\n");
	  globalData.odeConfig.videoRecordingMode=true;
	  char dir[128];
	  char filename[140];
	  createNewDir("video", dir);
	  printf("Start video recording in %s!\n", dir);
	  sprintf(filename, "%s/frame", dir);
	  videostream->open(filename);
	}
	handled=true;
	break;
      case 16: // Ctrl - p
	pause = !pause;
	printf( pause ? "Pause\n" : "Continue\n" );
	handled = true;
	break;
	//     case 15: // Ctrl - o // TEST
	//       {
	// 	SceneView* sv = viewer->getSceneHandlerList().front()->getSceneView();
	// 	Vec3 p(400,400,0);
	// 	Pos o;
	// 	if(!sv->projectWindowIntoObject(p,o)){
	// 	  printf("SHIT happens always\n");
	// 	}
	// 	o.print();
	// 	handled = true;
	//       }
	//       break;
      default:
	// std::cout << ea.getKey() << std::endl;
	return false;
	break;
      }
    }
      break;
    case(osgGA::GUIEventAdapter::KEYUP):
      handled = command(odeHandle, osgHandle, globalData, ea.getKey(), false);
    default:
      break;
    }
    if(handled)
      resetSyncTimer();
    return handled;
  }

  void Simulation::getUsage (osg::ApplicationUsage& au) const {
    au.addKeyboardMouseBinding("Simulation: Ctrl-f","File-Logging on/off");
    au.addKeyboardMouseBinding("Simulation: Ctrl-g","Restart the Gui-Logger");
    au.addKeyboardMouseBinding("Simulation: Ctrl-r","Start/Stop video recording");
    au.addKeyboardMouseBinding("Simulation: Ctrl-p","Pause on/off");
    au.addKeyboardMouseBinding("Simulation: +","increase simulation speed (realtimefactor)");
    au.addKeyboardMouseBinding("Simulation: -","decrease simulation speed (realtimefactor)");
    au.addKeyboardMouseBinding("Simulation: *","set maximum simulation speed (realtimefactor=0)");
    bindingDescription(au);
  }

  void Simulation::accept(osgGA::GUIEventHandlerVisitor& v) {
    v.visit(*this);
  }


  /// clears obstacle and agents lists and delete entries
  void Simulation::tidyUp(GlobalData& global) {
    QP(cout << "Profiling summary:" << endl << PROFILER.getSummary() << endl);
    QP(cout << endl << PROFILER.getSummary(quickprof::MILLISECONDS) << endl);
    QP(float timeSinceInit=PROFILER.getTimeSinceInit(quickprof::MILLISECONDS));
    QP(cout << endl << "total sum:      " << timeSinceInit << " ms"<< endl);
    QP(cout << "steps/s:        " << (((float)sim_step)/timeSinceInit * 1000.0) << endl);
    QP(cout << "realtimefactor: " << (((float)sim_step)/timeSinceInit * 10.0) << endl);

    // clear obstacles list
    for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++) {
      delete (*i);
    }
    global.obstacles.clear();

    // clear agents list
    for(OdeAgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++) {
      delete (*i)->getRobot();
      delete (*i)->getController();
      delete (*i)->getWiring();
      delete (*i);
    }
    if(global.environment) {
      delete global.environment;
      global.environment=0;
    }

    if(osgHandle.normalState)
      osgHandle.normalState->unref();
    if(osgHandle.transparentState)
      osgHandle.transparentState->unref();
    for(int i=0; i<3; i++) {
      if(osgHandle.tesselhints[i])
	osgHandle.tesselhints[i]->unref();
    }
    global.agents.clear();

    if(!noGraphics)    // delete viewer;
      viewer->getEventHandlers().clear();
    //        viewer->getEventHandlerList().clear();
  }


  /** creates fake command line options to make osg do what we want
      since we could not figure out how to disable the full screen mode
      we inject --window ... to the cmd line options
   */
  void Simulation::insertCmdLineOption(int& argc,char**& argv){
    char** nargv;
    int numnew=5;
    int nargc = argc+numnew;
    nargv=(char**)malloc(sizeof(char*)*nargc);
    memcpy(nargv,argv,sizeof(char*)*argc); // copy existing arguments
    memset(nargv+ argc,0,numnew*sizeof(char*)); // set all new args to 0
    nargv[argc++]=strdup("--window");
    nargv[argc++]=strdup("-1");
    nargv[argc++]=strdup("-1");
    nargv[argc++]=strdup(itos((int)windowWidth).c_str());
    nargv[argc++]=strdup(itos((int)windowHeight).c_str());
    assert(argc<=nargc);
    argv=nargv;
  }


  void Simulation::processCmdLine(int argc, char** argv) {
    if(contains(argv, argc, "-h") || contains(argv, argc, "--help"))
      usage(argv[0]);
    // guilogger-loading stuff here
    // start with online windows
    int index = contains(argv, argc, "-g");
    guiloggerinterval=5;
    if(index) {
      if(argc > index)
	guiloggerinterval=atoi(argv[index]);
      if (guiloggerinterval<1) // avoids a bug
        guiloggerinterval=5; // default value
      plotoptions.push_back(PlotOption(GuiLogger, Controller, guiloggerinterval, globalconfigurables));
    }

// logging to file
    filelogginginterval=5;
    index = contains(argv, argc, "-f");
    if(index) {
      if(argc > index)
	filelogginginterval=atoi(argv[index]);
      if (filelogginginterval<1) // avoids a bug
        filelogginginterval=5; // default value
      plotoptions.push_back(PlotOption(File, Controller, filelogginginterval, globalconfigurables));
    }

    // starting neuronviz
    neuronvizinterval=10;
    index = contains(argv, argc, "-n");
    if(index) {
      if(argc > index)
	neuronvizinterval=atoi(argv[index]);
      if (neuronvizinterval<1) // avoids a bug
        neuronvizinterval=10; // default value
      plotoptions.push_back(PlotOption(NeuronViz, Controller, neuronvizinterval, globalconfigurables));
    }

    // using SoundMan for acustic output
    index = contains(argv, argc, "-s");
    if(index) {
      string param="";
      if(argc > index)
	param=argv[index];
      plotoptions.push_back(PlotOption(SoundMan, Controller, 1, globalconfigurables,param));
    }

    index = contains(argv, argc, "-r");
    long seed=0;
    // initialize random number generator
    if(index && argc > index) {
      seed=atoi(argv[index]);
    } else {
      seed=time(0);
    }

    srand(seed);
    globalData.odeConfig.randomSeed=seed;

    int resolindex = contains(argv, argc, "-x");
    if(resolindex && argc > resolindex) {
      sscanf(argv[resolindex],"%lgx%lg", &windowWidth,&windowHeight);
    }
    windowWidth = windowWidth < 64 ? 64 : (windowWidth > 1600 ? 1600 : windowWidth);
    windowHeight = windowHeight < 64 ? 64 : (windowHeight > 1200 ? 1200 : windowHeight);

    if(contains(argv, argc, "-fs")){
      windowHeight=-1;
      windowWidth=-1;
      printf("running in fullscreen\n");
    }

    noGraphics = contains(argv, argc, "-nographics")!=0;
    pause = contains(argv, argc, "-pause")!=0;

    index = contains(argv, argc, "-shadow");
    if(index && (argc > index)) {
      shadow=(double)atoi(argv[index]);
      printf("shadowType=%lg\n",shadow);
    }

    index = contains(argv, argc, "-shadowsize");
    if(index && argc > index) {
      shadowTexSize = atoi(argv[index]);
      printf("shadowTexSize=%lg\n",shadowTexSize);
    }
    if(contains(argv, argc, "-noshadow")!=0) {
      shadow=0;
      printf("using no shadow\n");
    }

    osgHandle.drawBoundings= contains(argv, argc, "-drawboundings")!=0;

    // read intended simulation time
    index = contains(argv, argc, "-simtime");
    if (index) {
      if(argc > index){
	simulation_time=atol(argv[index]);
	printf("simtime=%li\n",simulation_time);
      }
    }

    index = contains(argv, argc, "-threads");
    if (index) {
      if(argc > index){
	int threads = atoi(argv[index]);
	QMP_SET_NUM_THREADS(threads);
	printf("Number of threads=%i\n", threads);
      }
    }

    if (contains(argv, argc, "-odethread")) {
      useOdeThread=1;
      printf("using separate OdeThread\n");
    }
    if (contains(argv, argc, "-osgthread")) {
      useOsgThread=1;
      printf("using separate OSGThread\n");
    }

    if (contains(argv, argc, "-savecfg")) {
      storeOdeRobotsCFG();
    }

  }


  // This function is called, if there was a possible Collision detected (in a space used at call of dSpaceCollide (0))
  void Simulation::nearCallback_TopLevel(void *data, dGeomID o1, dGeomID o2) {
    Simulation* me = (Simulation*) data;
    if (!me)
      return;

    bool collision_treated=false;
    // call robots collision treatments (old stuff, should be removed at some point)
    for(OdeAgentList::iterator i= me->globalData.agents.begin();
	(i != me->globalData.agents.end()) && !collision_treated; i++) {
      collision_treated=(*i)->getRobot()->collisionCallback(data, o1, o2);
    }

    if (collision_treated)
      return; // exit if collision was treated by a robot

    nearCallback(data, o1, o2);
  }


  void Simulation::nearCallback (void *data, dGeomID o1, dGeomID o2) {
    Simulation* me = (Simulation*) data;
    if (!me)
      return;
    if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {
      // colliding a space with something
      dSpaceCollide2 (o1,o2,data,&nearCallback);
      // collide all geoms internal to the space(s)
      // FIXME: I (Georg) believe this is not necessary because we have list of spaces to check!
      if (dGeomIsSpace (o1)) {
	if(! me->odeHandle.isIgnoredSpace((dxSpace*)o1) )
	  dSpaceCollide ((dxSpace*)o1,data,&nearCallback);
      }
      if (dGeomIsSpace (o2)) {
	if(! me->odeHandle.isIgnoredSpace((dxSpace*)o2) )
	  dSpaceCollide ((dxSpace*)o2,data,&nearCallback);
      }
    } else {
      // colliding two non-space geoms, so generate contact
      // points between o1 and o2
      /// use the new method with substances
      dSurfaceParameters surfParams;
      // check whether ignored pair (e.g. connected by joint)
      if(me->odeHandle.isIgnoredPair(o1, o2 )) {
	//cerr << "ign:  " << o1  << " " << o2  << "\t " << me->odeHandle.ignoredPairs->size()<< endl;
	return;
      }
      //cerr << "col:  " << o1  << " " << o2  << "\t " << me->odeHandle.ignoredPairs->size()<< endl;
      //      Primitive* p1 = (Primitive*)dGeomGetData (o1);
      //      Primitive* p2 = (Primitive*)dGeomGetData (o2);
      Primitive* p1 = dynamic_cast<Primitive*>((Primitive*)dGeomGetData (o1));
      Primitive* p2 = dynamic_cast<Primitive*>((Primitive*)dGeomGetData (o2));
      if(!p1 || !p2) {
	cerr << "collision detected without primitive\n";
	return;
      }

      int i,n;
      const int N = 80;
      dContact contact[N];
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      if(n>0) {
	const Substance& s1 = p1->substance;
	const Substance& s2 = p2->substance;
	int callbackrv = 1;
	if(s1.callback) {
	  callbackrv = s1.callback(surfParams, me->globalData, s1.userdata, contact, n,
				   o1, o2, s1, s2);
	}
	if(s2.callback && callbackrv==1) {
	  callbackrv = s2.callback(surfParams, me->globalData, s2.userdata, contact, n,
				   o2, o1, s2, s1 );
	}
	if(callbackrv==1) {
	  Substance::getSurfaceParams(surfParams, s1,s2, me->globalData.odeConfig.simStepSize);
	  //Substance::printSurfaceParams(surfParams);
	}
	if(callbackrv==0)
	  return;
	for (i=0; i < n; i++) {
	  contact[i].surface = surfParams;
	  dJointID c = dJointCreateContact (me->odeHandle.world,
					    me->odeHandle.jointGroup,&contact[i]);
	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
	}
      } // if contact points
    } // if geoms
  }


  /// internals

  void Simulation::control_c(int i) {
    ctrl_C++ ;
    // if (Control_C > 100)exit(0);
  }

  void Simulation::cmd_handler_exit(void) {
    signal(SIGINT,SIG_DFL);
    ctrl_C=0;
  }

  void Simulation::cmd_handler_init() {
    signal(SIGINT,control_c);
    atexit(cmd_handler_exit);
  }

  bool Simulation::control_c_pressed() {
    return ctrl_C!=0;
  }

  void Simulation::cmd_begin_input() {
    cmd_handler_exit();
  }

  void Simulation::cmd_end_input() {
    cmd_handler_init();
  }

  long Simulation::timeOfDayinMS() {
    struct timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec*1000 + t.tv_usec/1000;
  }

  void Simulation::resetSyncTimer() {
    realtimeoffset = timeOfDayinMS();
    simtimeoffset  = int(globalData.time*1000);
    justresettimes = true;
  }

  // this is used instead of the standard ODE print function
  //  to get rid of the annouying error messages
  static void printODEMessage (int num, const char *msg, va_list ap)
  {
    if(!ODEMessageFile){
      ODEMessageFile = fopen("ode.msg","w");
      if(!ODEMessageFile) return;
    }
    if (num) fprintf (ODEMessageFile,"%d: ",num);
    vfprintf (ODEMessageFile,msg,ap);
    fprintf (ODEMessageFile,"\n");
    fflush (ODEMessageFile);
  }



  bool Simulation::storeOdeRobotsCFG(){
    list<string> cs;
    cs+=string("Configruation file for lpzrobots ode simulation!");
    cs+=string("Most values are self-exlaining, also use -h with the simulator to learn more");
    cs+=string(" about the configuration.");
    cs+=string("The following values for Shadow are supported:");
    cs+=string("\t0: no shadow, 1: ShadowVolume, 2: ShadowTextue, 3: ParallelSplitShadowMap");
    cs+=string("\t4: SoftShadowMap, 5: ShadowMap (default)");
    if(storeCfg(odeRobotsCfg,cs)){
      printf("Configuration saved to %s!\n",odeRobotsCfg);
      return true;
    }else{
      fprintf(stderr,"Error while writing configuration file %s!\n", odeRobotsCfg);
      return false;
    }
  }


  void Simulation::usage(const char* progname) {
    printf("Usage: %s [-g [interval]] [-f [interval]] [-r seed] [-x WxH] [-fs] \n", progname);
    printf("\t\t [-pause] [-shadow N] [-noshadow] [-drawboundings] [-simtime [min]] [-threads N]\n");
    printf("\t\t [-odethread] [-osgthread] [-savecfg]\n");
    printf("\t-g interval\tuse guilogger (default interval 1)\n");
    printf("\t-f interval\twrite logging file (default interval 5)\n");
    printf("\t-n interval\tuse neuronviz (default interval 10)\n");
    printf("\t-s \"-disc|ampl|freq val\"\tuse soundMan \n");
    printf("\t-r seed\t\trandom number seed\n");
    printf("\t-x WxH\t\t* window size of width(W) x height(H) is used (default 640x480)\n");
    printf("\t-fs\t\tfullscreen mode\n");
    printf("\t-pause \t\tstart in pause mode\n");
    printf("\t-nographics \t\tstart without any graphics\n");
    printf("\t-noshadow \tdisables shadows and shaders (same as -shadow 0)\n");
    printf("\t-shadow [0..5]] \t* sets the type of the shadow to be used\n");
    printf("\t\t\t0: no shadow, 1: ShadowVolume, 2: ShadowTextue, 3: ParallelSplitShadowMap\n");
    printf("\t\t\t4: SoftShadowMap, 5: ShadowMap (default)\n");
    printf("\t-shadowsize size \t* sets the size of the shadow texture (default 2048)\n");
    printf("\t-drawboundings\tenables the drawing of the bounding shapes of the meshes\n");
    printf("\t-simtime min\tlimited simulation time in minutes\n");
    printf("\t-savecfg\tsafe the configuration file with the values given by the cmd line\n");
    printf("\t-threads N\tnumber of threads to use (default is the number of processors)\n");
    printf("\t-odethread\t* if given the ODE runs in its own thread. -> Sensors are delayed by 1\n");
    printf("\t-osgthread\t* if given the OSG runs in its own thread (recommended)\n");
    printf("\t* this parameter can be set in the configuration file ~/.lpzrobots/ode_robots.cfg\n");
    printf("More parameter concerning OSG graphics will follow...\n");
  }

  void Simulation::setCameraHomePos(const osg::Vec3& eye, const osg::Vec3& view) {
    if(!noGraphics) {
      osgGA::MatrixManipulator* mm =keyswitchManipulator->getCurrentMatrixManipulator();
      if(mm) {
	CameraManipulator* cameramanipulator = dynamic_cast<CameraManipulator*>(mm);
	if(cameramanipulator)
	  cameramanipulator->setHome(eye, view);
      }
    }
  }


  void createNewDir(const char* base, char *newdir) {
    struct stat s;
    for(int i=0; i<1000; i++) {
      sprintf(newdir,"%s%03i", base, i);
      if(stat(newdir,&s)!=0) { // file/dir does not exist -> take it
	mkdir(newdir, S_IREAD | S_IWRITE | S_IEXEC | S_IRGRP | S_IXGRP );
	return;
      }
    }
    assert(1); // should not happen
  }

  void Simulation::odeStep() {

    QP(PROFILER.beginBlock("collision                    "));
    // for parallelising the collision detection
    // we would need distinct jointgroups for each thread
    // also the most time is required by the global collision callback which is one block
    // so it makes no sense to is quickmp here
    dSpaceCollide ( odeHandle.space , this , &nearCallback_TopLevel );
    FOREACHC(vector<dSpaceID>, odeHandle.getSpaces(), i) {
      dSpaceCollide ( *i , this , &nearCallback );
    }
    QP(PROFILER.endBlock("collision                    "));

    QP(PROFILER.beginBlock("ODEstep                      "));
    dWorldStep ( odeHandle.world , globalData.odeConfig.simStepSize );
    dJointGroupEmpty (odeHandle.jointGroup);
    QP(PROFILER.endBlock("ODEstep                      "));
  }

  void Simulation::osgStep() {
    viewer->frame();

    // onPostDraw(*(viewer->getCamera()));
  }


  /// redirection function, because we can't call member function direct
  static void* odeStep_run(void* p) {
    Simulation* sim = dynamic_cast<Simulation*>((Simulation*)p);
    if(sim)
      sim->odeStep();
    else{
      cerr << "odeStep_run()::Shit happens" << endl;
    }
    return NULL;
  }

  /// redirection function, because we can't call member function direct
  static void* osgStep_run(void* p) {
    Simulation* sim = dynamic_cast<Simulation*>((Simulation*)p);
    if(sim)
      sim->osgStep();
    else{
      cerr << "osgStep_run()::Shit happens" << endl;
    }
    return NULL;
  }


}


