/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   Revision 1.18.4.8  2005-12-29 16:49:48  martius
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

#include <math.h>
#define PI M_PI // (3.14159265358979323846)
#include <ode/ode.h>
#include <ode/common.h>
#include <vector>
#include <iterator>
using namespace std;

#include "odeconfig.h"
#include "camera.h"
#include "grabframe.h"

#include "globaldata.h"
#include "base.h"

namespace osg{
  class ArgumentParser;
}
namespace osgProducer{
  class Viewer;
}

namespace lpzrobots {

class Simulation : public Base {
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
  /** config() is called when the user presses Ctrl-C. 
      Default: Call @changeParams(globalData.configs)@ */
  virtual void config(GlobalData& globalData);
  /// command() is called if a key was pressed
  virtual void command(const OdeHandle&, GlobalData& globalData, int key) {};
  /** collCallback() can be used to overload the standart collision handling.
      However it is called after the robots collision handling.       
     @return true if collision is treated, false otherwise
  */
  virtual bool collCallback(const OdeHandle&, void* data, dGeomID o1, dGeomID o2) { return false;};
  /// addCallback()  optional additional callback function.
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause) {};

protected:
  virtual bool init(int argc, char** argv);

  /** define the home position and view orientation of the camera.
      view.x is the heading angle in degree. view.y is the tilt angle in degree (nick), 
      view.z is ignored
  */
  void setCameraHomePos(const osg::Vec3& eye, const osg::Vec3& view);

  static void nearCallback(void *data, dGeomID o1, dGeomID o2);
  bool control_c_pressed();

private:
  static void processCmdLine(int argc, char** argv);
  void loop(bool pause);
  /// clears obstacle and agents lists and delete entries
  void tidyUp(GlobalData& globalData);

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


  struct timeval realTime;
  int nextLeakAnnounce;
  int leakAnnCounter;

  long sim_step;
  
  //  CameraType camType; // default is a non-moving and non-rotating camera
  //  OdeRobot* viewedRobot; // the robot who is viewed from the camera

private:
  SimulationState state;
  osg::ArgumentParser* arguments;
  osgProducer::Viewer* viewer;
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
/// offers the possibility to change parameter of all configurable objects
void changeParams(ConfigList& configs);

}

#endif
