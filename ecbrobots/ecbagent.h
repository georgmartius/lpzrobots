/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    marcoeckert@web.de                                                   *
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
 *   Revision 1.5  2009-08-11 15:49:05  guettler
 *   Current development state:
 *   - Support of communication protocols for XBee Series 1, XBee Series 2 and cable mode
 *   - merged code base from ecb_robots and Wolgang Rabes communication handling;
 *     ECBCommunicator (almost) entirely rewritten: Use of Mediator (MediatorCollegues: ECB),
 *     Callbackble (BackCaller: SerialPortThread)
 *   - New CThread for easy dealing with threads (is using pthreads)
 *   - New TimerThreads for timed event handling
 *   - SerialPortThread now replaces the cserialthread
 *   - GlobalData, ECBCommunicator is now configurable
 *   - ECBAgent rewritten: new PlotOptionEngine support, adapted to new WiredController structure
 *   - ECBRobot is now Inspectables (uses new infoLines functionality)
 *   - ECB now supports dnsNames and new communication protocol via Mediator
 *   - Better describing command definitions
 *   - SphericalRobotECB: adapted to new ECB structure, to be tested
 *
 *   Revision 1.4  2009/03/25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.3  2008/08/12 11:45:30  guettler
 *   plug and play update, added some features for the ECBRobotGUI
 *
 *   Revision 1.2  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#include <selforg/agent.h>
#include <selforg/configurable.h>
#include "ecbrobot.h"

namespace lpzrobots {


  class ECBAgent : public Agent , public Configurable {

    public:

      /**
       * Constructor.
       * @param plotOption as output setting.
       * @param noisefactor is used to set the relative noise strength of this agent
       * @return
       */
      ECBAgent(const PlotOption& plotOption = PlotOption(NoPlot), double noisefactor = 1);

      /**
       * Constructor.
       * @param plotOptions A list of PlotOption can given.
       * @param noisefactor is used to set the relative noise strength of this agent.
       * @return
       */
      ECBAgent(const std::list<PlotOption>& plotOptions, double noisefactor = 1);

      /**
       * Returns a pointer to the robot.
       */
      virtual ECBRobot* getRobot();

      /**
       * Initializes the object with the given controller, robot and wiring
       * and initializes the output options.
       * The initialisation is internally delayed until the ECBRobot is fully initialised
       * (all ECBs are initialised).
       */
      virtual bool init(AbstractController* controller, ECBRobot* robot, AbstractWiring* wiring);


      /** Performs an step of the agent, including sensor reading, pushing sensor values through the wiring,
          controller step, pushing controller outputs (= motorcommands) back through the wiring and sent
          resulting motorcommands to robot.
          @param noise Noise strength.
          @param time (optional) current simulation time (used for logging)
      */
      virtual void step(double noise, double time=-1);


    protected:

    private:
      Configurable::parambool restartPlotEngine;
      bool internalInitialised;
  };

}

