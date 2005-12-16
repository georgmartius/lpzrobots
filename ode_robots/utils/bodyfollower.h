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
 *   Revision 1.1.2.1  2005-12-16 16:22:57  fhesse
 *   draws a line connecting the last n positions of a body
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __BODYFOLLOWER_H
#define __BODYFOLLOWER_H

#include <ode/ode.h>
#include <vector>

class BodyFollower{
 public:
  BodyFollower(){};
  virtual BodyFollower::~BodyFollower();
  virtual void init(int horizon_, dBodyID body_to_follow);
  virtual void draw();

 private:
  dBodyID body;
  std::vector < dReal* > old_pos;
  int horizon;
  int counter;
  int filled;
};


#endif
