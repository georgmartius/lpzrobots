/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
   This is used by the agent class, @see Agent::setTrackOptions()
*/
class TrackRobot {
public:

  friend class Agent;
  friend class lpzrobots::OdeAgent;

  /// constructor for no tracking at all
  TrackRobot()
  {
    trackPos          = false;
    trackSpeed        = false;
    trackOrientation  = false;
    displayTrace      = false;
    interval          = 1;
    scene             = 0;
    file              = 0;
    cnt               = 1;
    memset(filename, 0, 256);

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
	     const char* scene = "", int interval = 1)
  {
    this->trackPos         = trackPos;
    this->trackSpeed       = trackSpeed;
    this->trackOrientation = trackOrientation;
    this->displayTrace     = displayTrace;
    this->interval         = interval;
    this->scene            = strdup(scene);
    file = 0;
    cnt  = 1;
    memset(filename, 0, 256);
  }


  TrackRobot(const TrackRobot &rhs)
  {
    deepcopy(*this, rhs);
  }

  const TrackRobot& operator=(const TrackRobot &rhs)
  {
    if ( this != &rhs )
    {
      if (scene)
        free(scene);
      scene = 0;

      if (file)
        fclose(file);
      file=0;

      deepcopy(*this, rhs);
    }

    return *this;
  }

  ~TrackRobot()
  {
    if (file)
      fclose(file);
    file = 0;

    //Something goes wrong here, but what? -> also with copy constructor no improvement, see above
    if (scene)
      free(scene);
    scene = 0;
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

 private:
   char filename[256];
   static void deepcopy (TrackRobot &lhs, const TrackRobot &rhs);

};

#endif
