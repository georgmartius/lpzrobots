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

#include "bodyfollower.h" 

#include <drawstuff/drawstuff.h>
#include "simulation.h" 

void BodyFollower::init(int horizon_, dBodyID body_to_follow){
  std::cout<<"1 \n";
  horizon=horizon_;
  old_pos.resize(horizon);
  for (int i=0; i<horizon; i++)
    old_pos[i]=new dReal[3];
  counter=0;
  std::cout<<"2 \n";
  body=body_to_follow;
  filled=0;
}

void BodyFollower::draw(){
   const  dReal* p = dBodyGetPosition(body);
  memcpy(old_pos[counter%horizon], p, sizeof(dReal[3]));
  filled++;
  if (filled>horizon){
    for (int i=-(horizon-1); i<0; i++){
      dsDrawLine(old_pos[(counter+i)%horizon], old_pos[(counter+i+1)%horizon]);
    }
  }
  counter=counter+1;
}

BodyFollower::~BodyFollower(){
  for (int i=0; i<horizon; i++)
    delete(old_pos[i]);
}
