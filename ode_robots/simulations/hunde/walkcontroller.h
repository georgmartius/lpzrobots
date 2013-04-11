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
 *   Revision 1.1  2008-01-29 10:01:37  der
 *   first version
 *
 *   Revision 1.1  2007/12/13 17:00:02  martius
 *   walk controller
 *
 *                                            *
 *                                                                         *
 ***************************************************************************/
#ifndef __WALKCONTROLLER_H
#define __WALKCONTROLLER_H


#include <stdio.h>
#include <selforg/abstractcontroller.h>

/**
 * robot controller for vierbeiner walk (hard coded)
 */
class WalkController : public AbstractController {
public:

  WalkController();

  virtual void init(int sensornumber, int motornumber);
  virtual int getSensorNumber() const {return number_sensors;}
  virtual int getMotorNumber() const {return number_motors;}
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber);
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  virtual std::list<iparamkey> getInternalParamNames()const  { return std::list<iparamkey>(); }

  virtual std::list<iparamval> getInternalParams() const { return std::list<iparamval>(); }

  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    Configurable::parse(f);
    return true;
  }


protected:

  int t;
  std::string name;
  int number_sensors;
  int number_motors;

  paramval speed;
  paramval phase;
  paramval kneeamplitude;
  paramval hipamplitude;
};

#endif
