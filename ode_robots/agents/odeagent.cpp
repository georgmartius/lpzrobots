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
 *   Revision 1.1.2.3  2006-03-31 16:16:58  fhesse
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

namespace lpzrobots {
  

  void OdeAgent::init_tracing(int tracelength/*=10*/,double tracethickness/*=0.003*/){
    trace_length=tracelength;
    trace_thickness=tracethickness;

    segments = (OSGPrimitive**) malloc(sizeof(OSGPrimitive*) * trace_length);
    for (int i=0; i<trace_length; i++){
      segments[i]=0;
    }
    // init lastpos with position of body_to_follow
    Pos pos(robot->getPosition());
    lastpos=pos;

    counter=0;

    tracing_initialized=true;
  }



  void OdeAgent::step(double noise){
    Agent::step(noise);
    // todo: do this (trackrobot.trace()) with friend class OdeAgent or the like
    // to be able to directly use trackrobot.tracePos
    if (trackrobot.trace()){
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
//       OsgHandle osgHandle_white=((OdeRobot*)robot)->osgHandle;
//       osgHandle_white.changeColor(Color(255, 255, 255));
//       s->init(osgHandle_white, OSGPrimitive::Low);
      s->init(((OdeRobot*)robot)->osgHandle, OSGPrimitive::Low);
      s->setMatrix(osg::Matrix::rotate(osg::Vec3(0,0,1), (pos - lastpos)) * 
		   osg::Matrix::translate(pos+(pos - lastpos)/2));
      segments[counter%trace_length] = s;
      lastpos = pos;
      counter++;
     //}
    }
  }
}


