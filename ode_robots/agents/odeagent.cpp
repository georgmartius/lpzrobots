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
 *   Revision 1.9  2008-09-16 14:48:05  martius
 *   Code indentation
 *
 *   Revision 1.8  2008/04/29 08:44:20  guettler
 *   #include <assert> added
 *
 *   Revision 1.7  2008/04/18 09:50:24  guettler
 *   Implemented step functions for multiple threads of the class Simulation
 *
 *   Revision 1.6  2007/08/30 09:46:29  martius
 *   simulation time
 *
 *   Revision 1.5  2007/03/28 07:16:37  martius
 *   drawing of tracking trace fixed
 *
 *   Revision 1.4  2006/08/11 15:40:27  martius
 *   removed to do
 *
 *   Revision 1.3  2006/08/04 15:06:24  martius
 *   TrackRobot tracing changed
 *
 *   Revision 1.2  2006/07/14 12:23:31  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/03/31 16:16:58  fhesse
 *   changed trace() to init_tracing()
 *   and check for init at beginning of step
 *
 *   Revision 1.1.2.2  2006/03/30 12:32:46  fhesse
 *   trace via trackrobot
 *
 *   Revision 1.1.2.1  2006/03/28 14:14:44  fhesse
 *   tracing of a given primitive (in the osg window) added
 *                                                *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include "odeagent.h"
#include "oderobot.h"
#include "pos.h"
#include <assert.h>

namespace lpzrobots {


  void OdeAgent::init_tracing(int tracelength,double tracethickness){
    trace_length=tracelength;
    trace_thickness=tracethickness;

    segments = (OSGPrimitive**) malloc(sizeof(OSGPrimitive*) * trace_length);
    for (int i=0; i<trace_length; i++){
      segments[i]=0;
    }
    // init lastpos with position of robot to follow
    Pos pos(robot->getPosition());
    lastpos=pos;

    counter=0;

    tracing_initialized=true;
  }



  void OdeAgent::step(double noise, double time){
    Agent::step(noise, time);
    if (trackrobot.isDisplayTrace() && t%10==0){
      if (!tracing_initialized) {
	init_tracing();
      }
      Pos pos(robot->getPosition());
      /* if construct used to draw cylinder only when length between actual
	 and last point is larger then a specific value
      */
      //if(counter==0 || ((pos - lastpos).length2() > 0.00005)  ) {
      double len = (pos - lastpos).length();
      if(segments[counter%trace_length]) delete segments[counter%trace_length];
      OSGPrimitive* s = new OSGCylinder(trace_thickness, len);
      s->init(((OdeRobot*)robot)->osgHandle, OSGPrimitive::Low);
      s->setMatrix(osg::Matrix::rotate(osg::Vec3(0,0,1), (pos - lastpos)) *
		   osg::Matrix::translate(lastpos+(pos - lastpos)/2));
      segments[counter%trace_length] = s;
      lastpos = pos;
      counter++;
      //}
    }
  }


  void OdeAgent::stepOnlyWiredController(double noise, double time) {
    WiredController::step(rsensors,rsensornumber, rmotors, rmotornumber, noise, time);
    trackrobot.track(robot, time);
    if (trackrobot.isDisplayTrace() && t%10==0){
      if (!tracing_initialized) {
	init_tracing();
      }
      Pos pos(robot->getPosition());
      /* if construct used to draw cylinder only when length between actual
	 and last point is larger then a specific value
      */
      //if(counter==0 || ((pos - lastpos).length2() > 0.00005)  ) {
      double len = (pos - lastpos).length();
      if(segments[counter%trace_length]) delete segments[counter%trace_length];
      OSGPrimitive* s = new OSGCylinder(trace_thickness, len);
      s->init(((OdeRobot*)robot)->osgHandle, OSGPrimitive::Low);
      s->setMatrix(osg::Matrix::rotate(osg::Vec3(0,0,1), (pos - lastpos)) *
		   osg::Matrix::translate(lastpos+(pos - lastpos)/2));
      segments[counter%trace_length] = s;
      lastpos = pos;
      counter++;
      //}
    }
  }

  void OdeAgent::setMotorsGetSensors() {
    robot->setMotors(rmotors, rmotornumber);

    assert(robot && rsensors && rmotors);

    int len =  robot->getSensors(rsensors, rsensornumber);
    if(len != rsensornumber){
      fprintf(stderr, "%s:%i: Got not enough sensors, expected %i, got %i!\n", __FILE__, __LINE__,
	      rsensornumber, len);
    }
  }

}


