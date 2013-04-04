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

#include "wiringsequence.h"
#include "controller_misc.h"
#include <assert.h>

using namespace std;

/// constructor
WiringSequence::WiringSequence(std::list<AbstractWiring*> ws)
  : AbstractWiring(0), wirings(ws.begin(), ws.end()) {
  initialised=false;
}

WiringSequence::WiringSequence(AbstractWiring* w1,AbstractWiring* w2)
  : AbstractWiring(0) {
  initialised=false;
  addWiring(w1);
  addWiring(w2);
}

WiringSequence::~WiringSequence(){
  FOREACH(vector<AbstractWiring*>, wirings, w){
    if(*w) delete *w;
  }
}

void WiringSequence::addWiring(AbstractWiring* wiring){
  assert(!initialised);
  assert(wiring);
  wirings.push_back(wiring);
}


// initializes the number of sensors and motors from robot, calculate
//  number of sensors and motors on controller side
bool WiringSequence::initIntern(){
  assert(wirings.size());
  int snum = rsensornumber;
  int mnum = rmotornumber;
  FOREACH(vector<AbstractWiring*>, wirings, w){
    // initialize the wiring with the output of the last wiring
    (*w)->init(snum,mnum, randGen);
    snum  = (*w)->getControllerSensornumber();
    mnum  = (*w)->getControllerMotornumber();
  }
  csensornumber = snum;
  cmotornumber  = mnum;
  initialised=true;
  return true;
}

bool WiringSequence::wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                       sensor* csensors, int csensornumber,
                                       double noiseStrength){
  assert(rsensornumber==this->rsensornumber);
  assert(csensornumber==this->csensornumber);

  const sensor* inp =  rsensors;
  int inp_s = rsensornumber;
  sensor* sensorbuf;
  int num = wirings.size();
  for(int i=0; i< num; i++){
    int d = wirings[i]->getControllerSensornumber();
    if(i==num-1){
      sensorbuf = csensors;
      assert(d == csensornumber);
    }else{
      sensorbuf = new sensor[d];
    }
    wirings[i]->wireSensors(inp, inp_s, sensorbuf, d, noiseStrength);
    if(i!=0) delete[] inp; // delete buffer, but not in first round
    inp   = sensorbuf;
    inp_s = d;
  }
  return true;
}

bool WiringSequence::wireMotorsIntern(motor* rmotors, int rmotornumber,
                                      const motor* cmotors, int cmotornumber){
  assert(rmotornumber==this->rmotornumber);
  assert(cmotornumber==this->cmotornumber);

  const motor* inp =  cmotors;
  int inp_s = cmotornumber;
  motor* motorbuf;
  int num = wirings.size();
  for(int i=num-1; i>=0; i--){
    int d = wirings[i]->getRobotMotornumber();
    if(i==0){
      motorbuf = rmotors;
      assert(d == rmotornumber);
    }else{
      motorbuf = new motor[d];
    }
    wirings[i]->wireMotors(motorbuf, d, inp, inp_s);
    if(i!=num-1) delete[] inp; // delete buffer, but not in first round
    inp   = motorbuf;
    inp_s = d;
  }
  return true;

}

