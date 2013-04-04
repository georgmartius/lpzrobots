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
#include "splitcontrol.h"

#include <assert.h>
#include <string.h>


SplitControl::SplitControl(ControllerGenerator* controllerGenerator,
                           const Assoziations& assoziations,
                           std::string controllerName,
                           int numCtrlCreateBeforeInit,
                           int numContextSensors)
  : AbstractController(controllerName, "1"), controllerGenerator(controllerGenerator),
    assoz(assoziations),
    numCtrlCreateBeforeInit (numCtrlCreateBeforeInit), numContextSensors(numContextSensors) {

  for(int i=0; i<numCtrlCreateBeforeInit; i++){
    AbstractController* c = (*controllerGenerator)(i);
    ctrl.push_back(c);
    addConfigurable(c);
  }

}

SplitControl::~SplitControl() {
  FOREACH(std::vector<AbstractController*>, ctrl, c){
    delete *c;
  }
  delete[] sensorbuffer;
  delete[] motorbuffer;
}

void SplitControl::init(const int sensornumber, const int motornumber,
                                   RandGen* randGen) {

  if(sensornumber-numContextSensors!=motornumber){
    fprintf(stderr,"sensor%i, context %i, motor: %i", sensornumber, numContextSensors, motornumber);
    assert(sensornumber-numContextSensors==motornumber);
  }

  this->sensornumber=sensornumber;
  this->motornumber=motornumber;

  sensorbuffer = new sensor[sensornumber+numContextSensors];
  motorbuffer = new motor[motornumber];

  int k=0;
  FOREACHC(Assoziations, assoz, a){
    FOREACHC(std::list<int>, a->sensors, s){
      assert(*s < sensornumber);
    }
    FOREACHC(std::list<int>, a->motors, m){
      assert(*m < motornumber);
    }
    int sensornum = a->sensors.size();
    int motornum = a->motors.size();
    assert(sensornum > 0 && motornum > 0);
    if(k<numCtrlCreateBeforeInit){
      ctrl[k]->init(sensornum + numContextSensors, motornum);
    }else{
      AbstractController* c = (*controllerGenerator)(k);
      c->init(sensornum + numContextSensors, motornum);
      ctrl.push_back(c);
      addConfigurable(c);
    }
    k++;
  }
}

void SplitControl::step(const sensor* sensors, int sensornumber,
                                   motor* motors, int motornumber) {

  int k=0;
  FOREACH(Assoziations, assoz, a){
    int i=0;
    FOREACHC(std::list<int>, a->sensors, s){
      sensorbuffer[i] = sensors[*s];
      i++;
    }
    if(numContextSensors!=0){
      memcpy(sensorbuffer+i, sensors+sensornumber-numContextSensors,
             sizeof(sensor)*numContextSensors);
    }
    ctrl[k]->step(sensorbuffer,i+numContextSensors,motorbuffer,a->motors.size());
    i=0;
    FOREACHC(std::list<int>, a->motors, m){
      motors[*m] = motorbuffer[i];
      i++;
    }
    k++;
  }
}

void SplitControl::stepNoLearning(const sensor* sensors , int sensornumber,
                                             motor* motors, int motornumber){
  int k=0;
  FOREACH(Assoziations, assoz, a){
    int i=0;
    FOREACHC(std::list<int>, a->sensors, s){
      sensorbuffer[i] = sensors[*s];
      i++;
    }
    if(numContextSensors!=0){
      memcpy(sensorbuffer+i, sensors+sensornumber-numContextSensors,
             sizeof(sensor)*numContextSensors);
    }
    ctrl[k]->stepNoLearning(sensorbuffer,i+numContextSensors,motorbuffer,a->motors.size());
    i=0;
    FOREACHC(std::list<int>, a->motors, m){
      motors[*m] = motorbuffer[i];
      i++;
    }
    k++;
  }
}



