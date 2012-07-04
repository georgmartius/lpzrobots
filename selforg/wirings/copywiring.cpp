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

#include "copywiring.h"
#include <assert.h>
#include <cstring>
#include <selforg/matrix.h>
#include <cmath>
#include <selforg/controller_misc.h>

using namespace matrix;

/// constructor
CopyWiring::CopyWiring(const Assignment& sensor_assignment,
                       const Assignment& motor_assignment,
                       NoiseGenerator* noise, int plotMode, const std::string& name)
  : AbstractWiring(noise, plotMode, name),
    s_assign(sensor_assignment),m_assign(motor_assignment){

}

CopyWiring::~CopyWiring(){
}

/// returns the inverse Assignment of the given sensor assignemnt (typically the right thing)
CopyWiring::Assignment CopyWiring::motorFromSensorAssignment(const Assignment& sensor_assignment){
  Assignment ma;
  FOREACHCI(Assignment, sensor_assignment, sa, k){
    FOREACHC(std::list<int>, *sa, s){
      if((signed)ma.size()<=*s) ma.resize(*s+1);
      ma[*s].push_back(k);
    }
  }
  return ma;
}

bool CopyWiring::initIntern(){
  csensornumber = s_assign.size();
  //maximal motor index
  int maxidx=0;
  FOREACHC(Assignment, m_assign, ma) {
    FOREACHC(std::list<int>, *ma, m) {
      maxidx = max(maxidx,*m);
    }
  }
  cmotornumber  = maxidx+1;
  assert((signed int)m_assign.size() <= rmotornumber);
  noisenumber   = csensornumber;
  return true;
}

bool CopyWiring::wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                  sensor* csensors, int csensornumber,
                                  double noiseStrength){
  assert(rsensornumber == this->rsensornumber);
  assert(csensornumber == this->csensornumber);

  FOREACHCI(Assignment, s_assign, sa, k) {
    csensors[k]=0;
    FOREACHC(std::list<int>, *sa, s) {
      if ( *s < 0  || *s >= rsensornumber ) {
        fprintf(stderr, "access to sensor out of range: %i", *s);
      } else {
        csensors[k]+= rsensors[*s];
      }
    }
    csensors[k]/=max(1.0, (double)sa->size());
  }

  // the noisevals are set in abstractwiring
  for(int i=0; i< csensornumber; i++){
    csensors[i] += noisevals[i];
  }
  return true;
}

bool CopyWiring::wireMotorsIntern(motor* rmotors, int rmotornumber,
                                 const motor* cmotors, int cmotornumber){
  assert(rmotornumber == this->rmotornumber);
  assert(cmotornumber == this->cmotornumber);

  FOREACHCI(Assignment, m_assign, ma, k) {
    rmotors[k]=0;
    FOREACHC(std::list<int>, *ma, m) {
      if ( *m < 0  || *m >= cmotornumber ) {
        fprintf(stderr, "access to motor value (controller) out of range: %i", *m);
      } else {
        rmotors[k]+= cmotors[*m];
      }
    }
    rmotors[k]/=max(1.0, (double)ma->size());
  }
  return true;
}


void CopyWiring::reset(){
}

