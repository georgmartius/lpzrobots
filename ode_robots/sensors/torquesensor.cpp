/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 ***************************************************************************/

#include <selforg/matrix.h>

#include "torquesensor.h"
#include "joint.h"
// #include "mathutils.h"

namespace lpzrobots {

  TorqueSensor::TorqueSensor(Joint* joint, double maxtorque)
    : joint(joint), maxtorque(maxtorque) {
    assert(joint);
  }

  void TorqueSensor::init(Primitive* own){    
    // set joint to feedback mode
    dJointSetFeedback (joint->getJoint(), &feedback);    
  }

  int TorqueSensor::getSensorNumber() const{
    return get().size();
  }
  
  bool TorqueSensor::sense(const GlobalData& globaldata) { return true; }

  std::list<sensor> TorqueSensor::get() const {
    // todo: check axis 
    Pos t1(feedback.t1);
    Pos t2(feedback.t2);
    std::list<sensor> l;
    l.push_back((t1.length()+t2.length())/maxtorque);
    return l;
  }

  int TorqueSensor::get(sensor* sensors, int length) const{
    assert(length>=1);
    Pos t1(feedback.t1);
    Pos t2(feedback.t2);
    sensors[0] = (t1.length()+t2.length())/maxtorque;    
    return 1;
  }

}

