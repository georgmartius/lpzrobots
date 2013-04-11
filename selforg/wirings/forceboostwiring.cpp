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

#include "forceboostwiring.h"
#include <assert.h>
#include <cstring>
#include <selforg/matrix.h>
#include <cmath>
#include <selforg/controller_misc.h>

using namespace matrix;

/// constructor
ForceBoostWiring::ForceBoostWiring(NoiseGenerator* noise, double boost, bool exportBoostError, int plotMode, const std::string& name)
  : AbstractWiring(noise, plotMode, name), Configurable(name,"1.1"), boost(boost){

  addParameter("booster",&this->boost, 0, 1, "force boosting rate");
  if(exportBoostError)
    addInspectableMatrix("errorForce",&error,false,"ForceBoosting error");
}

ForceBoostWiring::~ForceBoostWiring(){
}


bool ForceBoostWiring::initIntern(){
  csensornumber = rsensornumber;
  cmotornumber  = rmotornumber;
  noisenumber   = csensornumber;
  assert(cmotornumber<=csensornumber);
  error.set(cmotornumber,1);
  sens.set(cmotornumber,1);

  return true;
}

bool ForceBoostWiring::wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                         sensor* csensors, int csensornumber,
                                         double noiseStrength){
  assert(rsensornumber == this->rsensornumber);
  assert(csensornumber == this->csensornumber);
  // the noisevals are set in abstractwiring
  for(int i=0; i< rsensornumber; i++){
    csensors[i] = rsensors[i] + noisevals[i];
  }
  sens.set(rsensors);
  return true;
}

bool ForceBoostWiring::wireMotorsIntern(motor* rmotors, int rmotornumber,
                                     const motor* cmotors, int cmotornumber){
  assert(rmotornumber == this->rmotornumber);
  assert(cmotornumber == this->cmotornumber);
  if(boost>0){
    ///
    Matrix mot(cmotornumber ,1,  cmotors);
    error += (mot-sens)*boost - error*.001 - (error.map(power3))*0.1;
    error.toMapP(1.0,clip); // limit error to [-1,1]
    Matrix rmot = (mot + error).mapP(1.0,clip);
    rmot.convertToBuffer(rmotors, rmotornumber);
  }else{
    memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
  }
  return true;
}


void ForceBoostWiring::reset(){
  error.toZero();
  sens.toZero();
}

