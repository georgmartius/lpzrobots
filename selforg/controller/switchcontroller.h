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
#ifndef __SWITCHCONTROLLER_H
#define __SWITCHCONTROLLER_H

#include "abstractcontroller.h"

/**
 * meta controller for switching control between
 * different subcontrollers.
 */
class SwitchController : public AbstractController {
public:

  SwitchController(const std::list<AbstractController*>& controllers, const std::string& name = "SwitchController", const std::string& revision = "1.0");

  virtual ~SwitchController();

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber);
  virtual void stepNoLearning(const sensor* sensors , int sensornumber,
                              motor* motors, int motornumber);

  virtual int getSensorNumber() const { return controllers.front()->getSensorNumber();};

  virtual int getMotorNumber() const { return controllers.front()->getMotorNumber();};


  virtual bool store(FILE* f) const { return controllers.front()->store(f);};
  virtual bool restore(FILE* f)     { return controllers.front()->restore(f);};

protected:
  int activecontroller;
  std::list<AbstractController*> controllers;
};

#endif
