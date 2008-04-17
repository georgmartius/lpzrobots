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
 *                                                                         *
 *   $Log$
 *   Revision 1.19  2008-04-17 14:54:01  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.18  2007/11/06 14:58:58  martius
 *   major change!
 *   agent is now a robot with a wired controller,
 *   most code moved to wiredcontroller class
 *
 *   Revision 1.17  2007/08/29 11:33:20  martius
 *   simulation time enters logfile
 *
 *   Revision 1.16  2007/06/21 16:22:17  martius
 *   indentation
 *
 *   Revision 1.15  2007/05/07 20:58:21  robot3
 *   added support for Interface Callbackable (to find in selforg/utils)
 *   classes can now register at agent to get a callback every step
 *
 *   Revision 1.14  2007/03/28 11:36:53  robot3
 *   Moved int t from private to protected area because the OdeAgent needs access
 *   to it.
 *
 *   Revision 1.13  2007/03/05 17:52:14  martius
 *   plotoptions hav optional string parameter
 *
 *   Revision 1.12  2007/02/27 12:00:05  robot5
 *   Minor changes for SoundMan functionalities.
 *
 *   Revision 1.11  2007/02/01 15:53:08  martius
 *   inspectables list. Robot is added in case it is derived from Inspectable
 *
 *   Revision 1.10  2006/12/21 11:44:17  martius
 *   commenting style for doxygen //< -> ///<
 *   FOREACH and FOREACHC are macros for collection iteration
 *
 *   Revision 1.9  2006/12/13 09:13:24  martius
 *   agents get comments about changed parameter for logfile
 *
 *   Revision 1.8  2006/12/11 18:10:27  martius
 *   noisefactor and default constructor
 *
 *   Revision 1.7  2006/11/30 10:02:11  robot5
 *   Added support for Sndchanger (experimental). Startup with argument -s.
 *
 *   Revision 1.6  2006/11/23 13:06:10  martius
 *   onlyControlRobot is a new function. Useful for ODE simulations where physics runs faster than
 *   control cycle
 *
 *   Revision 1.5  2006/11/17 13:46:53  martius
 *   list of configureables to appear in configuration file
 *
 *   Revision 1.4  2006/08/04 15:16:13  martius
 *   documentation
 *
 *   Revision 1.3  2006/08/02 09:35:09  martius
 *   LastPlot as a dummy option added
 *
 *   Revision 1.2  2006/07/14 12:23:57  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.7  2006/06/25 16:51:35  martius
 *   configureable has name and revision
 *   a robot is configureable by default
 *
 *   Revision 1.1.2.6  2006/05/23 21:13:08  martius
 *   - add virtual destructor
 *
 *   Revision 1.1.2.5  2006/05/15 13:08:34  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.1.2.4  2006/03/30 12:33:53  fhesse
 *   trackrobot now protected to give OdeAgent access
 *
 *   Revision 1.1.2.3  2006/01/31 16:45:18  martius
 *   neuronviz plotoption
 *
 *   Revision 1.1.2.2  2005/11/15 12:30:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.1.2.1  2005/11/14 17:37:56  martius
 *   moved to selforg
 *
 *   Revision 1.9  2005/11/07 17:03:30  martius
 *   class PlotOption added
 *   agent can be constructed with a list of PlotOptions
 *   tracking file gets controller parameters as well
 *
 *   Revision 1.8  2005/10/24 13:32:07  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.7  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.6  2005/08/03 20:34:58  martius
 *   use if Inspectable interface
 *
 *   Revision 1.5  2005/07/26 17:01:47  martius
 *   flushing every 10
 *   guilogger is opened with nice -2
 *
 *   Revision 1.4  2005/07/18 10:13:46  martius
 *   noise moved to wiring
 *
 *   Revision 1.3  2005/07/14 15:57:53  fhesse
 *   now agent contains controller, robot and wiring, plotting ability included, therefore plotagent can be removed; ono2onewiring replaces one2oneagent
 *
 *   Revision 1.2  2005/06/15 14:02:47  martius
 *   revised and basicly tested
 *                                                                 *
 ***************************************************************************/
#ifndef __AGENT_H
#define __AGENT_H

#include <stdio.h>
#include <list>
#include <string>

#include "wiredcontroller.h"
#include "randomgenerator.h"

class AbstractRobot;

#include "types.h"
#include "trackrobots.h"

/** The Agent contains a controller, a robot and a wiring, which connects robot and controller.
    Additionally there are some ways to keep track of internal information.
    You have the possibility to keep track of sensor values,
     motor values and internal parameters of the controller with PlotOptions.
    The name PlotOptions is a bit missleaded, it should be "OutputOptions",
     however you can write the data into a file or send it to visialisation tools like
     guilogger or neuronviz.

    If want to log the position, speed and orienation of your robot
    you can use setTrackOptions().
    Please be aware that the Agent inherits from WiredController. You
     might also find useful functions there.
 */
class Agent : public WiredController {
public:
  /** constructor. PlotOption as output setting.
      noisefactor is used to set the relative noise strength of this agent
   */
  Agent(const PlotOption& plotOption = PlotOption(NoPlot), double noisefactor = 1);
  /** constructor. A list of PlotOption can given.
      noisefactor is used to set the relative noise strength of this agent
   */
  Agent(const std::list<PlotOption>& plotOptions, double noisefactor = 1);

  /** destructor
   */
  virtual ~Agent();

  /** initializes the object with the given controller, robot and wiring
      and initializes the output options.
      It is also possible to provide a random seed, 
       if not given (0) rand() is used to create one
  */
  virtual bool init(AbstractController* controller, AbstractRobot* robot, 
		    AbstractWiring* wiring, long int seed=0);

  /** Performs an step of the agent, including sensor reading, pushing sensor values through the wiring,
      controller step, pushing controller outputs (= motorcommands) back through the wiring and sent
      resulting motorcommands to robot.
      @param noise Noise strength.
      @param time (optional) current simulation time (used for logging)
  */
  virtual void step(double noise, double time=-1);

  /** Sends only last motor commands again to robot.  */
  virtual void onlyControlRobot();

  /** Returns a pointer to the robot.
   */
  virtual AbstractRobot* getRobot() { return robot;}

  /// sets the trackoptions which enable tracking of a robot
  virtual void setTrackOptions(const TrackRobot& trackrobot);

  /** adds the PlotOptions to the list of plotoptions
      If a plotoption with the same Mode exists, then the old one is
      deleted first (overwritten from WiredController, to add robot)
   */
  virtual void addPlotOption(const PlotOption& plotoption);

protected:

  AbstractRobot* robot;

  sensor *rsensors;
  motor  *rmotors;

  RandGen randGen; // random generator for this agent
protected:
  TrackRobot trackrobot;
  int t; // access to this variable is needed from OdeAgent

};

#endif
