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
#include "switchcontroller.h"

#include <assert.h>


SwitchController::SwitchController(const std::list<AbstractController*>& controllers,
                                   const std::string& name, const std::string& revision)
  : AbstractController(name, revision), controllers(controllers) {

  addParameterDef("activecontroller",&activecontroller,0,0,10,"index of active controller");
  assert(controllers.size()>0);
  for(auto &c: controllers){
    addConfigurable(c);
    addInspectable(c);
  }
}

SwitchController::~SwitchController() {}

void SwitchController::init(const int sensornumber, const int motornumber,
                                           RandGen* randGen) {
  for(auto &c: controllers){
    c->init(sensornumber, motornumber, randGen);
  }
}

void SwitchController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
  motor* dummy = (motor*) malloc(sizeof(motor) * motornumber);

  activecontroller = std::max(0,std::min(activecontroller,(int)controllers.size()-1));

  FOREACHIa(controllers, c, i){
    if(activecontroller==i){
      (*c)->step(sensors,sensornumber,motors,motornumber);
    }else{
      (*c)->step(sensors,sensornumber,dummy,motornumber);
    }
  }
  free(dummy);
}

void SwitchController::stepNoLearning(const sensor* sensors , int sensornumber, motor* motors, int motornumber){
  motor* dummy = (motor*) malloc(sizeof(motor) * motornumber);

  activecontroller = std::max(0,std::min(activecontroller,(int)controllers.size()-1));

  FOREACHIa(controllers, c, i){
    if(activecontroller==i){
      (*c)->stepNoLearning(sensors,sensornumber,motors,motornumber);
    }else{
      (*c)->stepNoLearning(sensors,sensornumber,dummy,motornumber);
    }
  }
  free(dummy);
}

