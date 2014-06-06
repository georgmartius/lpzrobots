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

#include <selforg/matrix.h>

#include "torquesensor.h"
#include "joint.h"
// #include "mathutils.h"

namespace lpzrobots {

  TorqueSensor::TorqueSensor(double maxtorque, int avg)
    : joint(0), maxtorque(maxtorque) {
    tau = 1.0/std::max(1.0,(double)avg);
  }


  TorqueSensor::~TorqueSensor(){
  }

  void TorqueSensor::init(Primitive* own, Joint* joint){
    this->joint=joint;
    assert(this->joint);
    // set joint to feedback mode
    this->joint->setFeedBackMode(true);
    // assert(getSensorNumber()>0); // the joint may be a fixed joint and then this is a dummy sensor

  }

  int TorqueSensor::getSensorNumber() const{
    return joint->getNumberAxes();
  }

  bool TorqueSensor::sense(const GlobalData& globaldata) {
    int num = getSensorNumber();
    if((signed)values.size()<num){
      values.resize(num,0);
    }
    Pos t1;
    Pos t2;
    joint->getTorqueFeedback(t1,t2);
    for(int i=0; i<num; i++){
      const Pos& a = joint->getAxis(i);
      // scalar product of axis and force gives the resulting torque
      double p1 = t1 * a;
      double p2 = t2 * a;
      if(tau<1.0)
        values[i] = values[i]*(1-tau) + (p1+p2)*(-tau/maxtorque);
      else
                                values[i] = (p1+p2)/(-maxtorque);
    }
    // debugging:
    // std::cout << "T1:"; t1.print();
    // std::cout << "T2:"; t2.print();
    // std::cout << "\t\tT1+T2:"; Pos(t1+t2).print();
    //    Pos f1;
    //    Pos f2;
    // joint->getForceFeedback(f1,f2);
    // std::cout << "F1:"; f1.print();
    // std::cout << "F2:"; f2.print();
    // std::cout << "\t\tF1+F2:"; Pos(f1+f2).print();
    return true;
  }


  int TorqueSensor::get(sensor* sensors, int length) const {
    // we assume sense was called before.
    int num = getSensorNumber();
    assert(length >= num);
    for(int i=0; i<num; i++){
      sensors[i]= values[i];
    }
    return num;
  }


  std::list<sensor> TorqueSensor::getList() const {
    return getListOfArray();
  }

}

