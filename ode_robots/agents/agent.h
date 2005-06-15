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
 *   Revision 1.2  2005-06-15 14:02:47  martius
 *   revised and basicly tested
 *                                                                 *
 ***************************************************************************/
#ifndef __AGENT_H
#define __AGENT_H

#include "abstractrobot.h"
#include "abstractcontroller.h"

/// Abstract glue-object between controller and robot. 
//   Implements wireing of sensors from robot to inputs of the controller and
//   controller outputs to motors. 
class Agent {
public:
  /// default constructor
  Agent(){
    controller = 0;
    robot      = 0;
  }

  /// initializes the object with the given controller and robot
  // should be called from overloaded classes!
  virtual bool init(AbstractController* controller, AbstractRobot* robot){
    this->controller = controller;
    this->robot      = robot;
    if(!controller || !robot) return false;
    else return true;
  }

  /// Performs an step of the agent. 
  //   Must be overloaded in order to implement the appropriate mapping 
  //   of the robot sensors to the controller inputs and so on.
  // @param noise Noise strength.
  virtual void step(double noise) = 0;

  ///
  AbstractController* getController() { return controller;}
  ///
  AbstractRobot* getRobot() { return robot;}

protected:
  AbstractController* controller;
  AbstractRobot* robot;
};

#endif
