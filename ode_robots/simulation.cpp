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
#include "oderobot.h"
#include "odeagent.h"
#include "console.h"

#include <osg/Version>
#include <osg/ShapeDrawable>
#include <osg/ArgumentParser>
#include <osg/AlphaFunc>
#include <osgUtil/SceneView>
// #include <osgUtil/Optimizer>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgGA/StateSetManipulator>


#include <osgShadow/ShadowedScene>

#include "lpzviewer.h"
#include "lpzhelphandler.h"

#include "primitive.h"
#include "abstractobstacle.h"

#include "robotcameramanager.h"

#include "cameramanipulatorTV.h"
#include "cameramanipulatorFollow.h"
//#include "cameramanipulatorRace.h"
#include "motionblurcallback.h"

#include "randomobstacles.h"

// simple multithread api
#include <selforg/quickmp.h> // moved to selforg/utils

//#define QPROF
// simple profiling (only enabled if QPPOF is defined (Makefile) or above)
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

/**
   Namespace for the 3D robot simulator ode_robots
*/
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
    : plotoptions(globalData.plotoptions)
  {
    // default values are set in Base::Base()
    addParameter("ShadowTextureSize",&shadowTexSize);
    addParameter("UseNVidia",&useNVidia);
    addParameterDef("WindowWidth",&windowWidth,800);
    addParameterDef("WindowHeight",&windowHeight,600);
    addParameterDef("UseOdeThread",&useOdeThread,false);
    addParameterDef("UseOsgThread",&useOsgThread,false);
    addParameterDef("UseQMPThread",&useQMPThreads,true);
    addParameterDef("inTaskedMode",&inTaskedMode,false);

    addParameterDef("DefaultFPS",&defaultFPS,25);

    //     nextLeakAnnounce = 20;
    //     leakAnnCounter = 1;

    truerealtimefactor = 1;
    state    = none;
    pause    = false;
    noGraphics      = false;
    useKeyHandler   = false;
    simulation_time = -1;
    simulation_time_reached=false;
    viewer   = 0;
    arguments= 0;
    startConfigurator = false;

    // we have to count references by our selfes
    osg::Referenced::ref();
    osgGA::GUIEventHandler::ref();
    //    Producer::Camera::Callback::ref();

    odeThread = 0;
    osgThread = 0;
    odeThreadCreated=false;
    osgThreadCreated=false;

    videostream = new VideoStream();

    currentCycle = 1;
    windowName = "Lpzrobots - Selforg";

    // default color palette
    paletteFiles.push_back("colors/RGB_Full.gpl");
    paletteFiles.push_back("colors/DefaultColors.gpl");
    colorAliasFiles.push_back("colors/DefaultColorSchema.txt");
    verboseColorLoading=false;
  }


  Simulation::~Simulation() {
    QMP_CRITICAL(21);
    if(state!=running)
      return;

    if (!inTaskedMode)
      dCloseODE ();

    state=closed;
    if(arguments)
      delete arguments;
    // we have to count references by our selfes
    osgGA::GUIEventHandler::unref();
    //    Producer::Camera::Callback::unref_nodelete();
    //    Producer::Camera::Callback::unref_nodelete();
    osg::Referenced::unref_nodelete();

    QMP_END_CRITICAL(21);


    if(viewer) {
      // do not destroy window if in taskedmode, just do it at the end
      if (!inTaskedMode)
        delete viewer;
      viewer = 0;
    }
    osgDB::Registry::instance()->releaseGLObjects(0);
    // this does not help to get rid of the missing textures at restart with OSG 2.8
    // osgDB::Registry::instance()->clearObjectCache();
    // osgDB::Registry::instance()->clearArchiveCache();
    // osgDB::Registry::instance()->getOrCreateSharedStateManager()->prune();
  }

  void Simulation::addPaletteFile(const std::string& filename, bool verbose){
    paletteFiles.push_back(filename);
    verboseColorLoading=verbose;
  }
  void Simulation::addColorAliasFile(const std::string& filename, bool verbose){
    colorAliasFiles.push_back(filename);    
    verboseColorLoading=verbose;
  }


  bool Simulation::init(int argc, char** argv) {
    orig_argv = argv;

    //    QMP_CRITICAL(20);
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
    globalData.globalconfigurables.push_back(&(globalData.odeConfig));

    /**************** OpenSceneGraph-Section   ***********************/

    osgDB::FilePathList l = osgDB::getDataFilePathList();
    l.push_back("data");
    
    l.push_back("../../osg/data");
    const char* oderobotsdata = getenv("ODEROBOTSDATA");
    if(oderobotsdata){
      l.push_back(oderobotsdata);
    }
#ifdef PREFIX
    l.push_back(PREFIX+string("/share/lpzrobots/data"));// installation path
#endif
    osgDB::setDataFilePathList(l);
       
    osgHandle.init();
    addParameter("Shadow",&(osgHandle.cfg->shadowType));
    osgHandle.cfg->noGraphics = noGraphics;
    FOREACH(std::list<std::string>, paletteFiles, f){
      int rv = osgHandle.colorSchema()->loadPalette(*f);      
      if(rv<=0){
	cerr << "Error with palette file " << *f << ": " 
             << osgHandle.colorSchema()->getLoadErrorString(rv) << endl;
      }else if(verboseColorLoading){
        cerr << "Loaded " << rv << " colors from Palette " << *f << endl; 
      }
    }
    FOREACH(std::list<std::string>, colorAliasFiles, f){
      int rv = osgHandle.colorSchema()->loadAliases(*f);
      if(rv<=0){
	cerr << "Error with alias file " << *f << ": " 
             << osgHandle.colorSchema()->getLoadErrorString(rv) << endl;
      }else if(verboseColorLoading){
        cerr << "Loaded " << rv << " alias definitions " << *f << endl; 
      }
    }


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
	  mkdir(dir, S_IREAD | S_IWRITE | S_IEXEC | S_IRGRP | S_IXGRP);
	  storeOdeRobotsCFG();
	}
      }
    }
    // process cmdline (possibly overwrite values from cfg file
    if(!processCmdLine(argc, argv)) return false;    
    globalData.odeConfig.fps=defaultFPS;

    osgHandle.setup(windowWidth, windowHeight);

    if(!noGraphics) {
      // create fake command line options to make osg do what we want
      insertCmdLineOption(argc, argv);
      // use an ArgumentParser object to manage the program arguments.
      arguments = new ArgumentParser(&argc, argv);

//       // set up the usage document, in case we need to print out how to use this program.
//       arguments->getApplicationUsage()->setApplicationName(arguments->getApplicationName() );
//       arguments->getApplicationUsage()->setDescription(
// 	       "Lpzrobots Simulator, <robot.informatik.uni-leipzig.de>");
//       arguments->getApplicationUsage()->setCommandLineUsage(arguments->getApplicationName() );
//       arguments->getApplicationUsage()->addCommandLineOption(
// 	       "-h or --help", "Display this information");
//       // if user request help write it out to cout.
//       if (arguments->read("-h") || arguments->read("--help")) {
// 	arguments->getApplicationUsage()->write(std::cout);
// 	return false;
//       }
      // any option left unread are converted into errors to write out later.
      //    arguments->reportRemainingOptionsAsUnrecognized();

      // report any errors if they have occured when parsing the program aguments.
      if (arguments->errors()) {
	arguments->writeErrorMessages(std::cout);
	return false;
      }

      // construct the viewer.
      viewer = new LPZViewer(*arguments);
      if(useOsgThread && !osgHandle.cfg->shadowType==3){ // ParallelSplitShadowMap does not support threads
	viewer->setThreadingModel(Viewer::CullDrawThreadPerContext);
      }else{
	viewer->setThreadingModel(Viewer::SingleThreaded);
      }

      // add the ourself that we can react on keys and mouse
      viewer->addEventHandler(this);
      //     viewer->addEventHandler(new osgViewer::HelpHandler(arguments->getApplicationUsage()));
      viewer->addEventHandler(new LpzHelpHandler(arguments->getApplicationUsage()));
      viewer->addEventHandler(new osgViewer::WindowSizeHandler);
      viewer->addEventHandler(osgHandle.scene->robotCamManager); // resizing of video inlets

      if(useKeyHandler){
        // add the state manipulator
        viewer->addEventHandler( new osgGA::StateSetManipulator(viewer->getCamera()->getOrCreateStateSet()) );     
        viewer->addEventHandler(new osgViewer::ThreadingHandler);
        viewer->addEventHandler(new osgViewer::StatsHandler);
        viewer->addEventHandler(new osgViewer::RecordCameraPathHandler);
      }



      // add callback for video recording
#if OPENSCENEGRAPH_MAJOR_VERSION == 2 &&  OPENSCENEGRAPH_MINOR_VERSION <= 4
      viewer->getCamera()->setPostDrawCallback(videostream.get());
#else
      viewer->getCamera()->setFinalDrawCallback(videostream);
#endif
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
| LpzRobots simulator, http://robot.informatik.uni-leipzig.de    |\n\
+----------------------------------------------------------------+" );
    printf ( "Press Ctrl-C here on the terminal window for a commandline interface.\n" );
    printf ( "Press h      on the graphics window for help.\n\n" );
    printf ( "Random number seed: %li\n", globalData.odeConfig.getRandomSeed());

    makePhysicsScene();
    if (!noGraphics) {
      makeScene(osgHandle.scene, *osgHandle.cfg);
      if (!osgHandle.scene->scene)
	return false;
      osgHandle.parent=osgHandle.scene->scene;
      
      // add the display node to show what the robot cameras see
      osgHandle.scene->root->addChild(osgHandle.scene->robotCamManager->getDisplay());

      keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

      // setup the camera manipulators (make sure it is in agreement with the CameraMode enum)
      cameraHandle.cam=viewer->getCamera();
      CameraManipulator* cm[] = {
        new CameraManipulator(osgHandle.scene->scene, globalData, cameraHandle),
        new CameraManipulatorFollow(osgHandle.scene->scene, globalData, cameraHandle),
        new CameraManipulatorTV(osgHandle.scene->scene, globalData, cameraHandle),
        //      new CameraManipulatorRace(osgHandle.scene->scene, globalData, cameraHandle)
      };
        
      keyswitchManipulator->addMatrixManipulator( '1', "Static", cm[0]);
      keyswitchManipulator->addMatrixManipulator( '2', "Follow", cm[1]);
      keyswitchManipulator->addMatrixManipulator( '3', "TV",     cm[2]);
      //    keyswitchManipulator->addMatrixManipulator( '4', "Race",   cm[3]);          
      for(int i=0; i< 3; i++){
        globalData.agents.addCallbackable(cm[i], OdeAgentList::BACKCALLER_VECTOR_MODIFIED);
      }
      

      // select TV mode as default.
      keyswitchManipulator->selectMatrixManipulator(TV);
      viewer->setCameraManipulator( keyswitchManipulator );

      // get details on keyboard and mouse bindings used by the viewer.
      viewer->getUsage(*(arguments->getApplicationUsage()));
    }

    state=initialised;
    //    QMP_END_CRITICAL(20);
    // we created a new argv pointer in insertCmdLineOption
    if(orig_argv != argv) free(argv);     
    return true;
  }


  bool Simulation::run(int argc, char** argv) {


    if(!init(argc, argv)){
      tidyUp(globalData);
      return false;
    }

    if (!inTaskedMode) {
      initializeConsole();
      QP(PROFILER.init());
    }

    //********************Simulation start*****************
    state=running;
    globalData.time=0;
    resetSyncTimer();
    // default camera position
    setCameraHomePos (Pos(0, -20, 3),  Pos(0, 0, 0));

    start(odeHandle, osgHandle, globalData);

    printConfigs(globalData.configs);

    if(!noGraphics) {
      // optimize the scene graph, remove redundant nodes and state etc.
      // osgUtil::Optimizer optimizer;
      // optimizer.optimize(osgHandle.scene->root);

      // add model to viewer.
      viewer->setSceneData(osgHandle.scene->root);
      
      // add overlay from cameras
      viewer->setOffScreenData(osgHandle.scene->robotCamManager->getOffScreen());

      // create the windows and run the threads.
      viewer->realize();
      // we have to set our SIGINT handler again because the OSG overwrites it! Thanks!
      cmd_handler_init();

      // set title
      osgViewer::Viewer::Windows windows;
      viewer->getWindows(windows);
      assert(windows.size()>0);

      // set our motion blur callback as the draw operator on each window
      FOREACH(osgViewer::Viewer::Windows, windows, itr){
	if(globalData.odeConfig.motionPersistence > 0)
	  (*itr)->add(new MotionBlurOperation(globalData));
	(*itr)->setWindowName(windowName);
      }
    }
    if (startConfigurator)
      globalData.createConfigurator();

    while ( ( noGraphics || !viewer->done()) &&
	    (!simulation_time_reached || restart(odeHandle,osgHandle,globalData)) ) {
      if (simulation_time_reached) {
        printf("%li min simulation time reached (%li steps) -> simulation cycle (%i) stopped\n",
               (globalData.sim_step/6000), globalData.sim_step, currentCycle);
        // start a new cycle, set timer to 0 and so on...
        simulation_time_reached = false;
        globalData.time = 0;
        globalData.sim_step=0;
        this->currentCycle++;
        resetSyncTimer();
      }
      if(!loop())
	break;
    }
    if(useOdeThread) pthread_join (odeThread, NULL);
    if(useOsgThread) pthread_join (osgThread, NULL);
    QMP_CRITICAL(22);
    closeConsole();
    end(globalData);
    tidyUp(globalData);
    QMP_END_CRITICAL(22);
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
      if (control_c_pressed()){
	cmd_begin_input();
	run=config(globalData);
	cmd_end_input();
	resetSyncTimer();
      }

      // the simulation just runs if pause is not enabled
      if (!pause) {
	// increase time
	globalData.time += globalData.odeConfig.simStepSize;
	globalData.sim_step++;
	// print simulation time every 10 min.
	if(noGraphics &&
	   globalData.sim_step % long(600.0/globalData.odeConfig.simStepSize) ==0) {
	  printf("Simulation time: %li min\n",
		 globalData.sim_step/ long(60/globalData.odeConfig.simStepSize));
	}
	// finish simulation, if intended simulation time is reached
	if(simulation_time!=-1) { // check time only if activated
	  if( (globalData.sim_step/ ( long(1/globalData.odeConfig.simStepSize)*60))  == simulation_time) {
	    if (!simulation_time_reached) { // print out once only
	      printf("%li min simulation time reached -> simulation stopped \n", simulation_time);
	    }
	    simulation_time_reached=true;
            return run;
	  }
	}


// 	SEQUENCIAL VERSION
// 	// for all agents: robots internal stuff and control step if at controlInterval
// 	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++) {
// 	  if ( (globalData.sim_step % globalData.odeConfig.controlInterval ) == 0 ) {
// 	    (*i)->step(globalData.odeConfig.noise, globalData.time);
// 	    (*i)->getRobot()->doInternalStuff(globalData);
// 	  } else {
// 	    (*i)->onlyControlRobot();
// 	  }
// 	}

 	// for all agents: robots internal stuff and control step if at controlInterval
// 	PARALLEL VERSION
	if ( (globalData.sim_step % globalData.odeConfig.controlInterval ) == 0 ) {
          // render offscreen cameras (robot sensor cameras) (does not work in nographics mode)
          if(!noGraphics && viewer->needForOffScreenRendering()){
            QP(PROFILER.beginBlock("offScreenRendering           "));
            updateGraphics();
            viewer->renderOffScreen();
            QP(PROFILER.endBlock("offScreenRendering           "));
          }

	  QP(PROFILER.beginBlock("controller                   "));
	  if (useQMPThreads)
	  {
	    // PARALLEL VERSION (QMP)
            QMP_SHARE(globalData);
            // there is a problem with the useOdeThread in the loop (not static)
            if (useOdeThread) // whether to use a separate thread for ode
            {
              QMP_PARALLEL_FOR(i, 0, globalData.agents.size(),quickmp::INTERLEAVED)
              {
                QMP_USE_SHARED(globalData, GlobalData);
		globalData.agents[i]->beforeStep(globalData);
                globalData.agents[i]->stepOnlyWiredController(globalData.odeConfig.noise, globalData.time);
              }
              QMP_END_PARALLEL_FOR;
            } else {
              QMP_PARALLEL_FOR(i, 0, globalData.agents.size(),quickmp::INTERLEAVED)
              {
                QMP_USE_SHARED(globalData, GlobalData);
		globalData.agents[i]->beforeStep(globalData);
                globalData.agents[i]->step(globalData.odeConfig.noise, globalData.time);
              }
              QMP_END_PARALLEL_FOR;
            }
           } else {
             // SEQUENTIAL VERSION (NO QMP)
            // there is a problem with the useOdeThread in the loop (not static)
            if (useOdeThread) {
              FOREACH(OdeAgentList, globalData.agents, i) {
		(*i)->beforeStep(globalData);
                (*i)->stepOnlyWiredController(globalData.odeConfig.noise, globalData.time);
              }
            } else {
              FOREACH(OdeAgentList, globalData.agents, i) {
		(*i)->beforeStep(globalData);
                (*i)->step(globalData.odeConfig.noise, globalData.time);
              }
            }
          }
	  QP(PROFILER.endBlock("controller                   "));
	}else{ // serial execution is sufficient here
	  FOREACH(OdeAgentList, globalData.agents, i) {
	    (*i)->onlyControlRobot();
	  }
	}

	/****************** Simulationstep *****************/
	if(useOdeThread){
	  if (odeThreadCreated)
	    pthread_join (odeThread, NULL);
	  else odeThreadCreated=true;
   	}
	// Do this here because it
	// can provide collision handling (old style collision handling)
	// and this crashes in parallel version
	QP(PROFILER.beginBlock("internalstuff_and_addcallback"));
	FOREACH(OdeAgentList, globalData.agents, i) {
	  if (useOdeThread)
	    (*i)->setMotorsGetSensors();
	  (*i)->getRobot()->doInternalStuff(globalData);
	}
	addCallback(globalData, t==(globalData.odeConfig.drawInterval-1), pause,
		    (globalData.sim_step % globalData.odeConfig.controlInterval ) == 0);
	QP(PROFILER.endBlock("internalstuff_and_addcallback"));

	// manipulate agents (with mouse)
	if(!noGraphics){
	  videostream->pause = pause;

	  OSGCameraManipulator* mm =
	    keyswitchManipulator->getCurrentMatrixManipulator();

	  if(mm) {
	    CameraManipulator* cm = dynamic_cast<CameraManipulator*>(mm);
	    if(cm) cm->manipulateAgent(osgHandle);
	  }
	}

	if(useOdeThread)
	  pthread_create (&odeThread, NULL, odeStep_run,this);
	else
	  odeStep();

 	// call all registered physical callbackable classes
        QP(PROFILER.beginBlock("physicsCB                    "));
        if (useQMPThreads!=0)
          callBackQMP(Base::PHYSICS_CALLBACKABLE);
        else
          callBack(Base::PHYSICS_CALLBACKABLE);
        QP(PROFILER.endBlock("physicsCB                    "));
        
	// remove old sound signal and TmpObjects
	globalData.removeExpiredObjects();
      }

      // graphics rendering
      if(t==(globalData.odeConfig.drawInterval-1) && !noGraphics) {
	if(useOsgThread){
	  QP(PROFILER.beginBlock("graphics aync"));
	  if (osgThreadCreated)
	    pthread_join (osgThread, NULL);
	  else osgThreadCreated=true;
	  QP(PROFILER.endBlock("graphics aync"));
	}
	QP(PROFILER.beginBlock("graphicsUpdate               "));
	/************************** Update the scene ***********************/
        updateGraphics();

	// update the camera
	OSGCameraManipulator* mm = keyswitchManipulator->getCurrentMatrixManipulator();
	if(mm) {
	  CameraManipulator* cameramanipulator = dynamic_cast<CameraManipulator*>(mm);
	  if(cameramanipulator)
	    cameramanipulator->update();
	}
	// update timestats
	setTimeStats(globalData.time,globalData.odeConfig.realTimeFactor,
		     truerealtimefactor,pause);

	// call all registered graphical callbackable classes
        callBack(Base::GRAPHICS_CALLBACKABLE);
        QP(PROFILER.endBlock("graphicsUpdate               "));

        if(useOsgThread){
	  pthread_create (&osgThread, NULL, osgStep_run,this);
	}else{
          QP(PROFILER.beginBlock("graphics                     "));
	  osgStep();
	  QP(PROFILER.endBlock("graphics                     "));
	}

      } // end graphics rendering

    } // end for t drawinterval
    /************************** Time Syncronisation ***********************/
    // Time syncronisation of real time and simulations time
    long elapsed = timeOfDayinMS() - realtimeoffset;
    // simulation speed (calculate more precisely again if not pause or max speed)
    if(!pause) truerealtimefactor = (globalData.time*1000.0 - simtimeoffset)/(elapsed+1);
    if(globalData.odeConfig.realTimeFactor==0.0){
      // get refresh rate of fps/2 frames in full speed
      globalData.odeConfig.calcAndSetDrawInterval(globalData.odeConfig.fps/2,truerealtimefactor);
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


  void Simulation::updateGraphics(){
    /************************** Update the scene ***********************/    
    FOREACH(ObstacleList, globalData.obstacles, i) {
      (*i)->update();
    }
    FOREACH(OdeAgentList, globalData.agents, i) {
      (*i)->getRobot()->update();
    }

    // initialize those objects that are not yet initialized
    globalData.initializeTmpObjects(odeHandle, osgHandle);
    // draw/update temporary objects and sound blobs
    globalData.updateTmpObjects(osgHandle);
  }

  bool Simulation::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&) {
    bool handled = false;
    switch(ea.getEventType()) {
    case(osgGA::GUIEventAdapter::KEYDOWN): {
      handled = command(odeHandle, osgHandle, globalData, ea.getKey(), true);
      if(handled) {
	break;
      }
      	//printf("Key: %i\n", ea.getKey());
      switch(ea.getKey()) {
      case 3 : // Ctrl - c
	if (globalData.isConfiguratorOpen()){
          globalData.removeConfigurator();
        }else{
          globalData.createConfigurator();
        }
	break;
      case 6 : // Ctrl - f
	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++) {
	  if(!(*i)->removePlotOption(File)) {
	    PlotOption po(File, filelogginginterval);
	    (*i)->addAndInitPlotOption(po);
	  }
	}
	handled= true;
	break;
      case 7 : // Ctrl - g
	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++) {
	  if(!(*i)->removePlotOption(GuiLogger)) {
	    PlotOption po(GuiLogger, guiloggerinterval,
                          "-geometry +" + std::itos(windowWidth+12) + "+0");
	    (*i)->addAndInitPlotOption(po);
	  }
	}
	handled=true;
	break;
      case 13 : // Ctrl - m
	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++) {
	  if(!(*i)->removePlotOption(MatrixViz)) {
	    PlotOption po(MatrixViz, matrixvizinterval);
	    (*i)->addAndInitPlotOption(po);
	  }
	}
	handled=true;
	break;
      case 8 : // Ctrl - h
        {
          OSGCameraManipulator* mm =keyswitchManipulator->getCurrentMatrixManipulator();
          if(!mm) break;
          CameraManipulator* cameramanipulator = dynamic_cast<CameraManipulator*>(mm);
          if(!cameramanipulator) break;
          OdeAgent* agent = cameramanipulator->getWatchedAgent();
          if(agent && agent->getRobot()){
            agent->getRobot()->moveToPosition(Pos(0,0,.5));
          }
        }
	handled=true;
	break;

      case 65450: // keypad *  // normal * is allready used by LOD
	globalData.odeConfig.setParam("realtimefactor", 0);
	//std::cout << "realtimefactor = " << globalData.odeConfig.getParam("realtimefactor")
        //  << std::endl; // shown in the  hud anyway
	handled=true;
	break;
      case 65455: // keypad /
        globalData.odeConfig.setParam("realtimefactor", 1);
        //std::cout << "realtimefactor = " << globalData.odeConfig.getParam("realtimefactor")
        //  << std::endl; // shown in the  hud anyway
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
          //	  std::cout << "realtimefactor = " <<  globalData.odeConfig.getParam("realtimefactor")<< std::endl;
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
      case 19: // Ctrl - s // change ShadowTechnique
      {
    	  Base::changeShadowTechnique();
    	  handled=true;
      }
      break;
      case 'o': //  add random object
        {
          FOREACH(ObstacleList, globalData.obstacles, o){
            RandomObstacles* ro = dynamic_cast<RandomObstacles*>(*o);
            if(ro){
              ro->spawn();
              handled=true;            
            }
          }
          if(!handled){
            std::cout <<  "No RandomObstacles object found in the list of obstacles."<<std::endl;
            std::cout <<  " I create a default one, customize it by adding RandomObstacles in start()." << std::endl;
            // search ground
            AbstractGround* ag=0;
            FOREACH(ObstacleList, globalData.obstacles, o){
              ag = dynamic_cast<AbstractGround*>(*o);
              break;
            }
            RandomObstacles* ro = new RandomObstacles(odeHandle, osgHandle, 
                                                      RandomObstacles::getDefaultConf(ag));
            globalData.obstacles.push_back(ro);
            ro->spawn();
          }
        }
        break;
      case 'O': //  remove random object
        FOREACH(ObstacleList, globalData.obstacles, o){
          RandomObstacles* ro = dynamic_cast<RandomObstacles*>(*o);
          if(ro){
            ro->remove();
          }
        }
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
    au.addKeyboardMouseBinding("Sim: Ctrl-f","File-Logging on/off");
    au.addKeyboardMouseBinding("Sim: Ctrl-g","Restart the Gui-Logger");
    au.addKeyboardMouseBinding("Sim: Ctrl-m","Restart the MatrixViz");
    au.addKeyboardMouseBinding("Sim: Ctrl-c","Restart the Configurator");
    au.addKeyboardMouseBinding("Sim: Ctrl-h","Move watched agent to (0,0,0) position");
    au.addKeyboardMouseBinding("Sim: Ctrl-r","Start/Stop video recording");
    au.addKeyboardMouseBinding("Sim: Ctrl-p","Pause on/off");
    au.addKeyboardMouseBinding("Sim: +","increase simulation speed (realtimefactor)");
    au.addKeyboardMouseBinding("Sim: -","decrease simulation speed (realtimefactor)");
    au.addKeyboardMouseBinding("Sim: /","set normal simulation speed (realtimefactor=1)");
    au.addKeyboardMouseBinding("Sim: *","set maximum simulation speed (realtimefactor=0)");
    au.addKeyboardMouseBinding("Sim: Ctrl-s","change shadow technique");
    au.addKeyboardMouseBinding("Sim: o / O","add/remove random obstacle");
    bindingDescription(au);
  }

  void Simulation::accept(osgGA::GUIEventHandlerVisitor& v) {
    v.visit(*this);
  }


  /// clears obstacle and agents lists and delete entries
  void Simulation::tidyUp(GlobalData& global) {
    if (!inTaskedMode)
    {
      QP(cout << "Profiling summary:" << endl << PROFILER.getSummary() << endl);
      QP(cout << endl << PROFILER.getSummary(quickprof::MILLISECONDS) << endl);
      QP(float timeSinceInit=PROFILER.getTimeSinceInit(quickprof::MILLISECONDS));
      QP(cout << endl << "total sum:      " << timeSinceInit << " ms"<< endl);
      QP(cout << "steps/s:        " << (((float)globalData.sim_step)/timeSinceInit * 1000.0) << endl);
      QP(cout << "realtimefactor: " << (((float)globalData.sim_step)/timeSinceInit * 10.0) << endl);
    }

    if(!noGraphics && viewer)    // delete viewer;
      viewer->getEventHandlers().clear();
    //        viewer->getEventHandlerList().clear();
    
    // clear obstacles list
    for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++) {
      delete (*i);
    }
    global.obstacles.clear();

    // clear agents list
    for(OdeAgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++) {
      delete (*i);
    }
    if(global.environment) {
      delete global.environment;
      global.environment=0;
    }
    
    global.agents.clear();
    global.removeConfigurator();
    
    osgHandle.close();
    odeHandle.close();
    
    base_close();
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
    nargv[argc++]=(char*)"--window";
    nargv[argc++]=(char*)"-1";
    nargv[argc++]=(char*)"-1";
    nargv[argc++]=strdup(itos(windowWidth).c_str());
    nargv[argc++]=strdup(itos(windowHeight).c_str());
    argc=nargc; 
    argv=nargv; // this is definite memory loss
  }


  bool Simulation::processCmdLine(int argc, char** argv) {
    if(contains(argv, argc, "-h") || contains(argv, argc, "--help")){
      main_usage(argv[0]);
      usage();
      return false;
    }
    // guilogger-loading stuff here
    // start with online windows
    int index = contains(argv, argc, "-g");
    guiloggerinterval=5;
    if(index) {
      if(argc > index)
	guiloggerinterval=atoi(argv[index]);
      if (guiloggerinterval<1) // avoids a bug
        guiloggerinterval=5; // default value
      plotoptions.push_back(PlotOption(GuiLogger, guiloggerinterval,
                                       "-geometry +" + std::itos(windowWidth+12) + "+0"));
    }

// logging to file
    filelogginginterval=5;
    index = contains(argv, argc, "-f");
    if(index) {
      if(argc > index)
	filelogginginterval=atoi(argv[index]);
      if (filelogginginterval<1) // avoids a bug
        filelogginginterval=5; // default value
      std::string parameter="";
      if (contains(argv, argc, "ntst")) //no date and time in name of logfile
        parameter="no_time_in_filename";
      plotoptions.push_back(PlotOption(File, filelogginginterval, parameter));
    }

    // start configurator
    startConfigurator = contains(argv, argc, "-conf")!=0;

    // starting matrixviz
    matrixvizinterval=10;
    index = contains(argv, argc, "-m");
    if(index) {
      if(argc > index)
	matrixvizinterval=atoi(argv[index]);
      if (matrixvizinterval<1) // avoids a bug
        matrixvizinterval=10; // default value
      plotoptions.push_back(PlotOption(MatrixViz, matrixvizinterval));
    }

    // using SoundMan for acustic output
    index = contains(argv, argc, "-s");
    if(index) {
      string param="";
      if(argc > index)
	param=argv[index];
      plotoptions.push_back(PlotOption(SoundMan, 1, param));
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
    globalData.odeConfig.setRandomSeed(seed);

    int resolindex = contains(argv, argc, "-x");
    if(resolindex && argc > resolindex) {
      sscanf(argv[resolindex],"%ix%i", &windowWidth,&windowHeight);
    }
    windowWidth = windowWidth < 64 ? 64 : (windowWidth > 3200 ? 3200 : windowWidth);
    windowHeight = windowHeight < 64 ? 64 : (windowHeight > 2400 ? 2400 : windowHeight);

    if(contains(argv, argc, "-fs")){
      windowHeight=-1;
      windowWidth=-1;
      printf("running in fullscreen\n");
    }
    noGraphics = contains(argv, argc, "-nographics")!=0;
    // inform osg relevant stuff that no graphics is used
    osgHandle.cfg->noGraphics=noGraphics;
    if(noGraphics) 
      globalData.odeConfig.realTimeFactor=0;
    pause = contains(argv, argc, "-pause")!=0;

    index = contains(argv, argc, "-shadow");
    if(index && (argc > index)) {
      osgHandle.cfg->shadowType= atoi(argv[index]);
      printf("shadowType=%i\n",osgHandle.cfg->shadowType);
    }

    index = contains(argv, argc, "-shadowsize");
    if(index && argc > index) {
      shadowTexSize = min(max(atoi(argv[index]),32),1<<14);
      printf("shadowTexSize=%i\n",shadowTexSize);
    }
    if(contains(argv, argc, "-noshadow")!=0) {
      osgHandle.cfg->shadowType=0;
      printf("using no shadow\n");
    }

    index = contains(argv, argc, "-fps");
    if(index && argc > index) {
      defaultFPS = min(max(atoi(argv[index]),1),1000);
      printf("defaultFPS=%i\n",defaultFPS);
    }

    useKeyHandler = contains(argv, argc, "-allkeys")!=0;


    index = contains(argv, argc, "-rtf");
    if(index && (argc > index)) {
      globalData.odeConfig.realTimeFactor=max(0.0,atof(argv[index]));
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

    // initialize QuickMP with the number of processors
    QMP_SET_NUM_THREADS(0);
    index = contains(argv, argc, "-threads");
    if (index) {
      if(argc > index){
	int threads = atoi(argv[index]);
	if (threads==1)
	{ // if set to 1, disable use of QMP
	  useQMPThreads=false;
	  printf("Disabling QuickMP multithreading.\n");
	} else {
          useQMPThreads=true;
	  QMP_SET_NUM_THREADS(threads);
          printf("Number of QuickMP threads=%i\n", QMP_GET_MAX_THREADS());
	}
      }
    }

    if (contains(argv, argc, "-odethread")) {
      useOdeThread=true;
      printf("using separate OdeThread\n");
    }
    if (contains(argv, argc, "-osgthread")) {
      useOsgThread=true;
      printf("using separate OSGThread\n");
    }

    if (contains(argv, argc, "-savecfg")) {
      storeOdeRobotsCFG();
    }
    return true;
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
      // The collision of the geoms internal to the space(s)
      //  is done separately in odeStep() (for each space that is not ignored once)
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


  void Simulation::main_usage(const char* progname) {
    printf("Usage: %s [-g [interval]] [-f [interval] [ntst]] [-r seed] [-x WxH] [-fs] \n", progname);
    printf("    \t [-pause] [-shadow N] [-noshadow] [-drawboundings] [-simtime [min]] [-rtf X]\n");
    printf("    \t [-threads N] [-odethread] [-osgthread] [-savecfg] [-h|--help]\n");
    printf("    -conf\t\tuse Configurator\n");
    printf("    -g interval\t\tuse guilogger (default interval 1)\n");
    printf("    -f interval ntst\twrite logging file (default interval 5),\n\
    \t\t\tif ntst (no_time_stamp in log file name)\n");
    printf("    \t\t\tis given logfile names are generated without timestamp\n");
    printf("    -m interval\t\tuse matrixviz (default interval 10)\n");
    printf("    -s \"-disc|ampl|freq val\"\n    \t\t\tuse soundMan \n");
    printf("    -r seed\t\trandom number seed\n");
    printf("    -x WxH\t\t* window size of width(W) x height(H) is used (default 800x600)\n");
    printf("    -fps rate\t\t*framerate in 1/s\n");
    printf("    -fs\t\t\tfullscreen mode\n");
    printf("    -pause \t\tstart in pause mode\n");
    printf("    -rtf factor\t\treal time factor: ratio between simulation speed and real time\n\
    \t\t\t(special case 0: full speed) (default 1)\n");
    printf("    -allkeys\t\tall key strokes are available (useful for debugging  graphics)\n");
    printf("    -nographics\t\tstart without any graphics (implies -rtf 0)\n");
    printf("    -noshadow\t\tdisables shadows and shaders (same as -shadow 0)\n");
    printf("    -shadow [0..5]\t* sets the type of the shadow to be used\n");
    printf("    \t\t\t0: no shadow, 1: ShadowVolume, 2: ShadowTextue, 3: ParallelSplitShadowMap\n");
    printf("    \t\t\t4: SoftShadowMap, 5: ShadowMap (default)\n");
    printf("    -shadowsize N\t* sets the size of the shadow texture (default 2048)\n");
    printf("    -drawboundings\tenables the drawing of the bounding shapes of the meshes\n");
    printf("    -simtime min\tlimited simulation time in minutes\n");
    printf("    -savecfg\t\tsafe the configuration file with the values given by the cmd line\n");
    printf("    -threads N\t\tnumber of threads to use (0: number of processors (default))\n");
    printf("    -odethread\t\t* if given the ODE runs in its own thread. -> Sensors are delayed by 1\n");
    printf("    -osgthread\t\t* if given the OSG runs in its own thread (recommended)\n");
    printf("    -h --help\t\tshow this help\n");
    printf("    * this parameter can be set in the configuration file ~/.lpzrobots/ode_robots.cfg\n");
  }

  void Simulation::setCameraHomePos(const osg::Vec3& eye, const osg::Vec3& view) {
    if(!noGraphics) {
      OSGCameraManipulator* mm =keyswitchManipulator->getCurrentMatrixManipulator();
      if(mm) {
	CameraManipulator* cameramanipulator = dynamic_cast<CameraManipulator*>(mm);
	if(cameramanipulator)
	  cameramanipulator->setHome(eye, view);
      }
    }
  }

  void Simulation::setCameraMode(CameraMode mode) {
    if (!noGraphics) {
      keyswitchManipulator->selectMatrixManipulator(mode);
      // we have to re-set the camera manipulator to get an effect
      viewer->setCameraManipulator(keyswitchManipulator);
    }
  }

  void Simulation::setWatchedAgent(OdeAgent* agent) {
    if (agent && !noGraphics) {
      OSGCameraManipulator* mm =keyswitchManipulator->getCurrentMatrixManipulator();
      if(mm) {
        CameraManipulator* cameramanipulator = dynamic_cast<CameraManipulator*>(mm);
        if(cameramanipulator)
          cameramanipulator->setWatchedAgent(agent);
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

  void Simulation::osgStep()
  {
    if (viewer)
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

  /// restart() is called at the second and all following starts of the cylces
   bool Simulation::restart(const OdeHandle&, const OsgHandle&, GlobalData& globalData)
  {
    // do not restart!
    return false;
  }


}


