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
 *   Revision 1.1.2.3  2006-03-28 14:14:44  fhesse
 *   tracing of a given primitive (in the osg window) added
 *
 *   Revision 1.1.2.2  2005/12/06 10:13:23  martius
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
#include "osgprimitive.h"
#include "primitive.h"

namespace lpzrobots {

/** Specialised agent for ode robots
 */
  class OdeAgent : public Agent {
  public:
  /** constructor
   */
  OdeAgent(const PlotOption& plotOption)  : Agent(plotOption) {tracing_activated=false;}
  OdeAgent(const list<PlotOption>& plotOptions) : Agent(plotOptions) {tracing_activated=false;}

  /** destructor
   */
  virtual ~OdeAgent() {}

  /** initializes the object with the given controller, robot and wiring
      and initializes pipe to guilogger
  */
  virtual bool init(AbstractController* controller, OdeRobot* robot, AbstractWiring* wiring){
    return Agent::init(controller, robot, wiring);
  }

  /** Performs an step of the agent, including sensor reading, pushing sensor values through wiring, 
      controller step, pushing controller outputs (= motorcommands) back through wiring and sent 
      resulting motorcommands to robot.
      @param noise Noise strength.
  */
  virtual void step(double noise);

  void internInit(){
    Agent::internInit();
    trace_length=0; // number of past robot positions shown in osg
  }


  /** Returns a pointer to the robot.
   */
  virtual OdeRobot* getRobot() { return (OdeRobot*)robot;}

  /// gives the number of past robot positions shown as trace in osg
  virtual int getTraceLength(){return trace_length;}

  /// sets the primitive for tracing and the number of past positions shown as trace in osg
  virtual void trace(const OsgHandle& osgHandle, Primitive* body_to_follow, 
		     int tracelength=10, double tracethickness=0.003);

 private:
  OsgHandle osgHandle;
  int trace_length;
  double trace_thickness;
  int counter;
  Primitive* body_to_trace;
  bool tracing_activated;

  OSGPrimitive** segments; // stores segments(cylinders) of the trace
  osg::Vec3 lastpos;
};

}

#endif
