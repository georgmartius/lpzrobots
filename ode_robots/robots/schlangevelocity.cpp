/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.3  2006-07-20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:41  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.4  2006/06/25 17:00:33  martius
 *   Id
 *
 *   Revision 1.1.2.3  2006/06/25 16:57:16  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.2  2006/02/01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.1.2.1  2006/01/10 13:55:12  fhesse
 *   snake powered by directly setting angular velocities
 *                                        *
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
  void SchlangeVelocity::setMotors ( const motor* motors, int motornumber )
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
      ((UniversalJoint*)joints[i])->addTorques
 	(- conf.frictionJoint * ((UniversalJoint*)joints[i])->getPosition1Rate(), 
 	 - conf.frictionJoint * ((UniversalJoint*)joints[i])->getPosition2Rate());
    }
  }	

  /**
   *Writes the sensor values to an array in the memory.
   *@param sensor* pointer to the arrays
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  int SchlangeVelocity::getSensors ( sensor* sensors, int sensornumber )
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


