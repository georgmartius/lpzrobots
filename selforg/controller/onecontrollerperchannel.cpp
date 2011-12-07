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
#include "onecontrollerperchannel.h"

#include <assert.h>
#include <string.h>


OneControllerPerChannel::OneControllerPerChannel(ControllerGenerator* controllerGenerator,
                                                 std::string controllerName, 
                                                 int numCtrlCreateBeforeInit,
                                                 int numContextSensors)
  : AbstractController(controllerName, "1"), controllerGenerator(controllerGenerator),
    numCtrlCreateBeforeInit (numCtrlCreateBeforeInit), numContextSensors(numContextSensors) {
  for(int i=0; i<numCtrlCreateBeforeInit; i++){
    AbstractController* c = (*controllerGenerator)(i);
    ctrl.push_back(c);    
    addConfigurable(c);
  } 
  sensorbuffer = new sensor[1+numContextSensors];

}

OneControllerPerChannel::~OneControllerPerChannel() {
  FOREACH(std::vector<AbstractController*>, ctrl, c){
    delete *c;
  }
  delete[] sensorbuffer;
}

void OneControllerPerChannel::init(const int sensornumber, const int motornumber, 
                                   RandGen* randGen) {
  
  if(sensornumber-numContextSensors!=motornumber){
    fprintf(stderr,"sensor%i, context %i, motor: %i", sensornumber, numContextSensors, motornumber);
    assert(sensornumber-numContextSensors==motornumber);
  }

  this->sensornumber=sensornumber;
  this->motornumber=motornumber;
  
  for(int i=0; i<motornumber; i++){
    if(i<numCtrlCreateBeforeInit){
      ctrl[i]->init(1 + numContextSensors, 1);
    }else{
      AbstractController* c = (*controllerGenerator)(i);
      c->init(1 + numContextSensors, 1);      
      ctrl.push_back(c);
      addConfigurable(c);
    }
  }
}

void OneControllerPerChannel::step(const sensor* sensors, int sensornumber, 
                                   motor* motors, int motornumber) {
  assert((int)ctrl.size()==motornumber);
  if(numContextSensors==0){
    for(int i=0; i<motornumber; i++){        
      ctrl[i]->step(sensors+i,1,motors+i,1);    
    }
  }else{
    memcpy(sensorbuffer+1, sensors+sensornumber-numContextSensors, 
           sizeof(sensor)*numContextSensors);
    for(int i=0; i<motornumber; i++){        
      sensorbuffer[0]=sensors[i];
      ctrl[i]->step(sensorbuffer,1+numContextSensors,motors+i,1);
    }    
  }
}

void OneControllerPerChannel::stepNoLearning(const sensor* sensors , int sensornumber, 
                                             motor* motors, int motornumber){
  assert((int)ctrl.size()==motornumber);

  if(numContextSensors==0){
    for(int i=0; i<motornumber; i++){        
      ctrl[i]->stepNoLearning(sensors+i,1,motors+i,1);    
    }
  }else{
    memcpy(sensorbuffer+1, sensors+sensornumber-numContextSensors, 
           sizeof(sensor)*numContextSensors);
    for(int i=0; i<motornumber; i++){        
      sensorbuffer[0]=sensors[i];
      ctrl[i]->stepNoLearning(sensorbuffer,1+numContextSensors,motors+i,1);
    }    
  }
}

bool OneControllerPerChannel::store(FILE* f) const {
  bool rv=true;
  FOREACHC(std::vector<AbstractController*>, ctrl, c){
    rv &= (*c)->store(f);
  }
  return rv;
}
  
bool OneControllerPerChannel::restore(FILE* f){
  bool rv=true;
  FOREACH(std::vector<AbstractController*>, ctrl, c){
    rv &= (*c)->restore(f);
  }
  return rv;
}
