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
 *   Revision 1.8  2008-04-18 09:49:41  guettler
 *   Added the OdeAgent as a friend class
 *
 *   Revision 1.7  2007/08/29 11:33:20  martius
 *   simulation time enters logfile
 *
 *   Revision 1.6  2007/06/21 16:18:57  martius
 *   do not free scene -> Segfault
 *
 *   Revision 1.5  2007/04/05 15:14:15  martius
 *   angular speed tracking
 *
 *   Revision 1.4  2006/08/04 15:16:13  martius
 *   documentation
 *
 *   Revision 1.3  2006/08/02 09:33:21  martius
 *   Todo updated
 *
 *   Revision 1.2  2006/07/14 12:24:02  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/03/31 16:18:32  fhesse
 *   tracing in oderagent via trackrobots
 *
 *   Revision 1.1.2.2  2006/03/30 12:33:15  fhesse
 *   trace via trackrobot
 *
 *   Revision 1.1.2.1  2005/11/16 11:24:27  martius
 *   moved to selforg
 *
 *   Revision 1.4  2005/11/10 09:08:26  martius
 *   trace has a name
 *
 *   Revision 1.3  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef __TRACKROBOTS_H
#define __TRACKROBOTS_H

#include <stdio.h>
#include <string.h>

class AbstractRobot;
class Agent;

namespace lpzrobots {
  class OdeAgent;
}

/**
   This class provides tracking possibilies of a robot.
   The position, speed, and orientation can be logged.
*/
class TrackRobot {
public:

  friend class Agent;
  friend class lpzrobots::OdeAgent;

  /// constructor for no tracking at all
  TrackRobot(){
    trackPos = false;
    trackSpeed = false;
    trackOrientation = false;
    displayTrace = false;
    interval = 1;
    file=0;
    cnt=0;
    scene=0;
  }

  /** Constructor that allows individial setting of tracking options.
      The tracked data is written into a file with the current date and time appended by a name given by scene.
      @param trackPos if true the trace (position vectors) of the robot are logged
      @param trackSpeed if true the speed vectors (linear and angular) of the robot are logged
      @param trackOrientation if true the orientation matrices  of the robot are logged
      @param displayTrace if true the trace of the robot should be displayed (used in ODE simulations)
      @param scene name of the scene (is appended to log file name)
      @param interval timesteps between consequent logging events (default 1)
   */
  TrackRobot(bool trackPos, bool trackSpeed, bool trackOrientation, bool displayTrace,
	     const char* scene, int interval = 1){
    this->trackPos     = trackPos;
    this->trackSpeed   = trackSpeed;
    this->trackOrientation = trackOrientation;
    this->displayTrace     = displayTrace;
    this->interval = interval;
    this->scene = strdup(scene);
    file=0;
    cnt=0;
  }

  ~TrackRobot(){
    // if(scene) free(scene);
  }

  /// returns whether tracing is activated
  bool isDisplayTrace() const {return displayTrace;};

 protected:
  bool open(const AbstractRobot* robot);
  void track(AbstractRobot* robot, double time);
  void close();

 protected:
  bool trackPos;
  bool trackSpeed;
  bool trackOrientation;
  bool displayTrace;

  int interval;
  FILE* file;
  char* scene;
  long cnt;
};



#endif
