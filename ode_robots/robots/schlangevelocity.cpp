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

#include "schlangevelocity.h"
using namespace std;


namespace lpzrobots {

  SchlangeVelocity::SchlangeVelocity ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                                       const SchlangeConf& conf, const std::string& name)
    : Schlange(odeHandle, osgHandle, conf, name, "$Id$")
  {
  }


  SchlangeVelocity::~SchlangeVelocity() { }


  /**
   *Reads the actual motor commands from an array, and sets all motors (forces) of the snake to this values.
   *It is an linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1]
   *@param motornumber length of the motor array
   **/
  void SchlangeVelocity::setMotorsIntern( const double* motors, int motornumber )
  {
    assert(created);
    // there will always be an even number of motors
    // (two sensors/motors per joint)
    int len = min(motornumber/2, (int)joints.size());
    // controller output as torques; friction added
    for (int i = 0; i < len; i++){
      // motorcommand
      ((UniversalJoint*)joints[i])->setParam ( dParamVel, factor_motors * motors[2*i]);
      ((UniversalJoint*)joints[i])->setParam ( dParamFMax , conf.motorPower );
      //friction
      ((UniversalJoint*)joints[i])->addForces
         (- conf.frictionJoint * ((UniversalJoint*)joints[i])->getPosition1Rate(),
          - conf.frictionJoint * ((UniversalJoint*)joints[i])->getPosition2Rate());
    }
  }

  /**
   *Writes the sensor values to an array in the memory.
   *@param sensors pointer to the arrays
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  int SchlangeVelocity::getSensorsIntern( sensor* sensors, int sensornumber )
  {
    assert(created);
    // there will always be an even number of senors
    // (two sensors/motors per joint)
    int len = min(sensornumber/2, (int)joints.size());
    // read angle of joints
    /*
      for (int n = 0; n < len; n++) {
      sensors[2*n]   = joints[n]->getPosition1();
      sensors[2*n+1] = joints[n]->getPosition2();
      }
    */
    // or read anglerate of joints
    for (int n = 0; n < len; n++) {
      sensors[2*n]   = conf.sensorFactor * ((UniversalJoint*)joints[n])->getPosition1Rate();
      sensors[2*n+1] = conf.sensorFactor * ((UniversalJoint*)joints[n])->getPosition2Rate();
    }
    return len*2;
  }


  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void SchlangeVelocity::create(const osg::Matrix& pose){
    Schlange::create(pose);

    //*****************joint definition***********
    for ( int n = 0; n < conf.segmNumber-1; n++ ) {

      Pos p1(objects[n]->getPosition());
      Pos p2(objects[n]->getPosition());
      UniversalJoint* j = new UniversalJoint(objects[n], objects[n+1],
                                             (objects[n]->getPosition() + objects[n+1]->getPosition())/2,
                                             Axis(0,0,1)* pose, Axis(0,1,0)* pose);
      j->init(odeHandle, osgHandle, true, conf.segmDia * 1.02);

      // setting stops at universal joints
      j->setParam(dParamLoStop, -conf.jointLimit*1.5);
      j->setParam(dParamHiStop,  conf.jointLimit*1.5);
      j->setParam(dParamLoStop2, -conf.jointLimit*1.5);
      j->setParam(dParamHiStop2,  conf.jointLimit*1.5);

      // making stops bouncy
          j->setParam (dParamBounce, 0.9 );
          j->setParam (dParamBounce2, 0.9 ); // universal

      joints.push_back(j);
    }
  }


  /** destroys vehicle and space
   */
  void SchlangeVelocity::destroy(){
    if (created){
      Schlange::destroy();
    }
  }

}


