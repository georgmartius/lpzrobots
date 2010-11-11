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
    : joint(joint), maxtorque(maxtorque), feedback(0), allocatedfb(false) {
    
    assert(joint);
  }


  TorqueSensor::~TorqueSensor(){
    if(allocatedfb){      
      // this would require the sensor to be deleted before the joint, which is not enshured.
      // dJointSetFeedback (joint->getJoint(), 0); 
      free(feedback);
    }
  }

  void TorqueSensor::init(Primitive* own){    
    // set joint to feedback mode
    if((feedback=dJointGetFeedback(joint->getJoint()))==0){
      feedback = (dJointFeedback*)malloc(sizeof(dJointFeedback));
      allocatedfb = true;
      dJointSetFeedback (joint->getJoint(), feedback);          
    }
  }

  int TorqueSensor::getSensorNumber() const{
    return get().size();
  }
  
  bool TorqueSensor::sense(const GlobalData& globaldata) { return true; }

  std::list<sensor> TorqueSensor::get() const {
    // todo: check axis 
    dJointFeedback* fb = dJointGetFeedback(joint->getJoint());
    assert(fb);
    Pos t1(fb->t1);
    Pos t2(fb->t2);
    std::list<sensor> l;
    //    t1.print();
    //    t2.print();
    l.push_back((t1.length()+t2.length())/maxtorque);
    return l;
  }

}

