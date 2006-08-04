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
 *   Revision 1.4  2006-08-04 15:06:38  martius
 *   documentation
 *
 *   Revision 1.3  2006/07/20 17:19:43  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:30  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2006/06/12 13:50:21  robot8
 *   -added atomodeagent
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

#include <selforg/agent.h>
#include <selforg/abstractwiring.h>
#include "atomodeagent.h"
#include "odeagent.h"

using namespace std;

namespace lpzrobots
{

/// initializes the object with the given controller, robot and wiring
//  and initializes pipe to guilogger
//overwrites the init function of agent.cpp
AtomOdeAgent::AtomOdeAgent(const PlotOption& plotOption) : OdeAgent(plotOption)
{

	      
}

AtomOdeAgent::AtomOdeAgent(const std::list<PlotOption>& plotOptions) : OdeAgent(plotOptions)
{

	      
}

AtomOdeAgent::~AtomOdeAgent ()
{
}


bool AtomOdeAgent::init ( AbstractController* controller, OdeRobot* robot, AbstractWiring* wiring )
{


  this->controller = controller;
  this->robot      = robot;
  this->wiring     = wiring;
  if(!controller || !robot || !wiring) return false;
  else{
    rsensornumber = 10;
    rmotornumber  = 10;

    wiring->init(rsensornumber, rmotornumber);

    csensornumber = wiring->getControllerSensornumber();
    cmotornumber  = wiring->getControllerMotornumber();

    controller->init(csensornumber, cmotornumber);

    rsensors      = (sensor*) malloc(sizeof(sensor) * rsensornumber);
    rmotors       = (motor*)  malloc(sizeof(motor)  * rmotornumber);
    csensors      = (sensor*) malloc(sizeof(sensor) * csensornumber);
    cmotors       = (motor*)  malloc(sizeof(motor)  * cmotornumber);

/*    // open the plotting pipe (and file logging) if configured
    for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
      // this prevents the simulation to terminate if the child  closes
      // or if we fail to open it.
      signal(SIGPIPE,SIG_IGN); 
      (*i).open();
    }    
    // init the plotting pipe 
    initPlottingPipe();*/

    return true;
  }
}




}
