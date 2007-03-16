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
 ***************************************************************************
 *                                                                         *
 *  simulation.h and simulation.cpp provide a generic ode-robot simulation *
 *  framework. It implements the initialisation, the simulation loop,      * 
 *  and a basic command line interface.                                    *
 *  Usage: call simulation_init(), simulation_start(), simulation_close()  *
 *         see template_onerobot/main.cpp for an example                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.26  2007-03-16 10:53:18  martius
 *   new nearcallback structure
 *
 *   Revision 1.25  2007/02/12 13:29:48  martius
 *   addCallback has flag about controllstep
 *
 *   Revision 1.24  2006/12/13 09:13:03  martius
 *   agents get comments about changed parameter for logfile
 *
 *   Revision 1.23  2006/12/11 18:31:34  martius
 *   list of configurables for agents
 *   reference counting and memleaks fixed
 *   onlycontrol used in steps where  controller is not used
 *
 *   Revision 1.22  2006/09/20 15:30:40  martius
 *   shadowsize
 *
 *   Revision 1.21  2006/08/04 15:07:46  martius
 *   documentation
 *
 *   Revision 1.20  2006/07/14 15:17:33  fhesse
 *   start option for intended simulation time added
 *   -simtime [min]
 *
 *   Revision 1.18.4.19  2006/06/29 16:31:47  robot3
 *   includes cleared up
 *
 *   Revision 1.18.4.18  2006/06/25 16:52:23  martius
 *   filelogging is done with a plotoption
 *
 *   Revision 1.18.4.17  2006/05/28 22:12:03  martius
 *   - noshadow cmdline flag
 *
 *   Revision 1.18.4.16  2006/05/15 13:07:48  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.18.4.15  2006/03/30 12:34:47  martius
 *   documentation updated
 *
 *   Revision 1.18.4.14  2006/03/06 16:54:05  robot3
 *   now ExtendedViewer is used because of the new getCurrentCameraManipulator(),
 *   code optimizations
 *
 *   Revision 1.18.4.13  2006/02/22 15:26:32  martius
 *   frame grabbing with osg works again
 *
 *   Revision 1.18.4.12  2006/02/20 10:50:20  martius
 *   pause, random, windowsize, Ctrl-keys
 *
 *   Revision 1.18.4.11  2006/02/14 17:36:14  martius
 *   much better time syncronisation
 *
 *   Revision 1.18.4.10  2006/01/17 17:01:53  martius
 *   *** empty log message ***
 *
 *   Revision 1.18.4.9  2006/01/12 22:33:23  martius
 *   key eventhandler integrated
 *
 *   Revision 1.18.4.8  2005/12/29 16:49:48  martius
 *   end is obsolete
 *   tidyUp is used for deletion
 *
 *   Revision 1.18.4.7  2005/12/29 12:54:09  martius
 *   multiple Tesselhints
 *
 *   Revision 1.18.4.6  2005/12/15 17:02:04  martius
 *   light is in sky and standart cams removed
 *   config has a default implentation now
 *
 *   Revision 1.18.4.5  2005/12/11 23:35:07  martius
 *   *** empty log message ***
 *
 *   Revision 1.18.4.4  2005/12/09 16:53:17  martius
 *   camera is working now
 *
 *   Revision 1.18.4.3  2005/12/06 17:38:13  martius
 *   *** empty log message ***
 *
 *   Revision 1.18.4.2  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.18.4.1  2005/11/14 17:37:01  martius
 *   changed makefile structure to have and include directory
 *   mode to selforg
 *
 *   Revision 1.18  2005/10/06 17:11:26  martius
 *   switched to stl lists
 *
 *   Revision 1.17  2005/09/27 13:58:48  martius
 *   added drawLine
 *
 *   Revision 1.16  2005/09/22 13:17:11  martius
 *   OdeHandle and GlobalData finished
 *   doInternalStuff included
 *
 *   Revision 1.15  2005/09/22 11:21:57  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.14  2005/08/12 11:55:01  robot1
 *   camera module integrated
 *
 *   Revision 1.13  2005/08/03 20:33:30  martius
 *   changed signature of contains (but it stays compatible)
 *
 *   Revision 1.12  2005/07/27 13:23:16  martius
 *   new color and position construction
 *
 *   Revision 1.11  2005/07/18 08:35:27  martius
 *   drawcallback is additionalcallback now
 *
 *   Revision 1.10  2005/07/13 08:39:21  robot8
 *   added the possibility to use an additional command function, which handels special Inputs if the ODE simulation window has the focus
 *
 *   Revision 1.9  2005/07/08 10:14:05  martius
 *   added contains (helper for stringlist search)
 *
 *   Revision 1.8  2005/07/07 10:23:44  martius
 *   added user draw callback
 *
 *   Revision 1.7  2005/06/30 13:23:38  robot8
 *   completing the call of the dynamic collisionCallback-function for  standard collisions
 *
 *   Revision 1.6  2005/06/29 09:27:11  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2005/06/29 09:25:06  martius
 *   customized callback for collision
 *
 *   Revision 1.4  2005/06/17 08:42:01  martius
 *   documentation
 *
 *   Revision 1.3  2005/06/15 14:01:31  martius
 *   moved all general code from main to simulation
 *                                                                 *
 ***************************************************************************/
#ifndef __SIMULATION_H
#define __SIMULATION_H

// include base classes of class Simulation
#include "base.h"
#include <osgGA/GUIEventHandler>
#include <Producer/Camera>

#include <math.h>
#define PI M_PI // (3.14159265358979323846)
#include <vector>
#include <iterator>

#include "globaldata.h"
#include "grabframe.h"
#include "pos.h"

/***  some forward declarations  ***/
class PlotOption; // selforg

namespace lpzrobots {
  class ExtendedViewer;
}
/*** end of forward declarations ***/


namespace lpzrobots {

  class Simulation : public Base, public osgGA::GUIEventHandler, public Producer::Camera::Callback {
  public:

    typedef enum SimulationState { none, initialised, running, closed };
    
    Simulation();
    virtual ~Simulation();

    /** starts the Simulation. Do not overload it. 
	This function returns of the simulation is terminated.
	@return: true if closed regulary, false on error
    */
    bool run(int argc, char** argv);
  
    // the following function have to be overloaded.

    /// start() is called at the start and should create all the object (obstacles, agents...).
    virtual void start(const OdeHandle&, const OsgHandle&, GlobalData& globalData) = 0;

    // the following functions have dummy default implementations

    /// end() is called at the end and should tidy up
    virtual void end(GlobalData& globalData);
    /** config() is called when the user presses Ctrl-C */
    virtual void config(GlobalData& globalData);
    /** is called if a key was pressed. 
	For keycodes see: osgGA::GUIEventAdapter
	@return true if the key was handled
    */
    virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, 
			 int key, bool down) { return false; };

    /** this can be used to describe the key bindings used by command()     
     */
    virtual void bindingDescription(osg::ApplicationUsage & au) const {};

    /** collCallback() can be used to overload the standart collision handling.
	However it is called after the robots collision handling.       
	@return true if collision is treated, false otherwise
    */
    virtual bool collCallback(const OdeHandle&, void* data, dGeomID o1, dGeomID o2) { return false;};

    /** optional additional callback function which is called every simulation step. 
	Called between physical simulation step and drawing.
	@param draw indicates that objects are drawn in this timestep
	@param pause indicates that simulation is paused
	@param control indicates that robots have been controlled this timestep	
     */
    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {};

    ///////////////// Camera::Callback interface
    virtual void operator() (const Producer::Camera &);

  

  protected:
    // GUIEventHandler
    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
    virtual void getUsage (osg::ApplicationUsage & au) const;
    virtual void accept(osgGA::GUIEventHandlerVisitor& v);

    virtual bool init(int argc, char** argv);

    /** define the home position and view orientation of the camera.
	view.x is the heading angle in degree. view.y is the tilt angle in degree (nick), 
	view.z is ignored
    */
    void setCameraHomePos(const osg::Vec3& eye, const osg::Vec3& view);

    static void nearCallback_TopLevel(void *data, dGeomID o1, dGeomID o2);
    static void nearCallback(void *data, dGeomID o1, dGeomID o2);
    bool control_c_pressed();

    // plotoptions is a list of possible online output, 
    // if the list is empty no online gnuplot windows and no logging to file occurs.
    // The list is modified with commandline options, see run() in simulation.cpp
    std::list<PlotOption> plotoptions;
    /// this list contains by default only the odeconfig. This list os passed to the plotoptions.
    std::list<const Configurable*> globalconfigurables;


  private:
    void processCmdLine(int argc, char** argv);
    void loop();
    /// clears obstacle and agents lists and delete entries
    void tidyUp(GlobalData& globalData);

    void resetSyncTimer();
    long timeOfDayinMS();

    static void control_c(int i);
    static void cmd_handler_exit();
    static void cmd_handler_init();
    static void cmd_begin_input();
    static void cmd_end_input();

    // Commandline interface stuff
    static void usage(const char* progname);

  protected:
    GlobalData globalData;
    VideoStream videostream;

    int nextLeakAnnounce;
    int leakAnnCounter;
    long realtimeoffset;
    long simtimeoffset;

    int windowWidth;
    int windowHeight;
    bool pause;
    bool simulation_time_reached;
    long int simulation_time;

    long sim_step;

    int guiloggerinterval;
    int filelogginginterval;
    int neuronvizinterval;

    //  CameraType camType; // default is a non-moving and non-rotating camera
    //  OdeRobot* viewedRobot; // the robot who is viewed from the camera

  private:
    SimulationState state;
    osg::ArgumentParser* arguments;
    ExtendedViewer* viewer;
    Producer::Camera* cam;
    static int ctrl_C;
  };

  // /// initializes or resets the camera per user, if wanted
  // void camera_init(CameraType type, OdeRobot* robot);

  // /// starts the simulation.
  // void simulation_start(int argc, char** argv);
  // /// call this after the @simulation_start()@ has returned to tidy up.
  // void simulation_close();

  // Helper
  /// returns the index+1 if the list contains the given string or 0 if not
  int contains(char **list, int len,  const char *str);

  // Commandline interface stuff
  /// shows all parameters of all given configurable objects
  void showParams(const ConfigList& configs);
  /// offers the possibility to change parameter of all configurable objects in globalData.
  // also informs agents about changes
  void changeParams(GlobalData& globalData);
  
  /// creates a new directory with the stem base, which is not yet there (using subsequent numbers)
  void createNewDir(const char* base, char *newdir);
}

#endif
