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
 *   Revision 1.1.2.2  2005-12-06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.1.2.1  2005/11/15 12:29:18  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __ODEAGENT_H
#define __ODEAGENT_H

#include <selforg/agent.h>
#include "oderobot.h"

namespace lpzrobots {

/** Specialised agent for ode robots
 */
class OdeAgent : public Agent {
public:
  /** constructor
   */
  OdeAgent(const PlotOption& plotOption)  : Agent(plotOption) {}
  OdeAgent(const list<PlotOption>& plotOptions) : Agent(plotOptions) {}

  /** destructor
   */
  virtual ~OdeAgent() {}

  /** initializes the object with the given controller, robot and wiring
      and initializes pipe to guilogger
  */
  virtual bool init(AbstractController* controller, OdeRobot* robot, AbstractWiring* wiring){
    return Agent::init(controller, robot, wiring);
  }

  /** Returns a pointer to the robot.
   */
  virtual OdeRobot* getRobot() { return (OdeRobot*)robot;}

};

}

#endif
