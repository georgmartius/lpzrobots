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
 *   Revision 1.15  2006-09-20 12:56:17  martius
 *   Snakes have CreateSegment
 *
 *   Revision 1.14  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.13  2006/07/14 12:23:41  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.12.4.8  2006/06/25 17:00:32  martius
 *   Id
 *
 *   Revision 1.12.4.7  2006/06/25 16:57:15  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.12.4.6  2006/04/25 09:06:16  robot3
 *   *** empty log message ***
 *
 *   Revision 1.12.4.5  2006/02/23 18:05:05  martius
 *   friction with angularmotor
 *
 *   Revision 1.12.4.4  2006/02/01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.12.4.3  2006/01/10 14:24:32  fhesse
 *   second motor config
 *
 *   Revision 1.12.4.2  2006/01/04 14:46:00  fhesse
 *   inherits from Schlange; moved to osg
 *
 *   Revision 1.12.4.1  2005/12/12 22:36:28  martius
 *   indentation
 *
 *   Revision 1.12  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/

#include "schlangeforce.h"
using namespace std;

namespace lpzrobots {

  SchlangeForce::SchlangeForce ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
				 const SchlangeConf& conf, const string& name,
				 const std::string& revision) 
    : Schlange(odeHandle, osgHandle, conf, name, 
	       revision.empty() ? "$Id$" : revision)
  {
  }
	
  SchlangeForce::~SchlangeForce() { }
	

  /**
   *Reads the actual motor commands from an array, and sets all motors (forces) of the snake to this values.
   *It is an linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1] 
   *@param motornumber length of the motor array
   **/
  void SchlangeForce::setMotors ( const motor* motors, int motornumber )
  {
    assert(created);
    // there will always be an even number of motors
    // (two sensors/motors per joint)
    int len = min(motornumber/2, (int)joints.size());
    // controller output as torques; friction added
    for (int i = 0; i < len; i++){
      // motorcommand
      // use all motors
      ((UniversalJoint*)joints[i])->addTorques(conf.motorPower * motors[2*i], 
      					       conf.motorPower * motors[2*i+1]);
      
      // or use only one motor at a joint (alternating between motor 1 and motor 2)      
      // http://www.novell.com/linux/      if (i%2==0){
      // 	((UniversalJoint*)joints[i])->addTorques(conf.motorPower * motors[2*i],0);
      //       }
      //       else{
      // 	((UniversalJoint*)joints[i])->addTorques(0, conf.motorPower * motors[2*i+1]);
      //       }
    }
  }	

  /**
   *Writes the sensor values to an array in the memory.
   *@param sensor* pointer to the arrays
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  int SchlangeForce::getSensors ( sensor* sensors, int sensornumber )
  {
    assert(created);
    // there will always be an even number of senors
    // (two sensors/motors per joint)
    int len = min(sensornumber/2, (int)joints.size()); 
    // reading angle of joints
    /*
      for (int n = 0; n < len; n++) {
      sensors[2*n]   = joints[n]->getPosition1();
      sensors[2*n+1] = joints[n]->getPosition2();
      }
    */
    // or reading anglerate of joints
    for (int n = 0; n < len; n++) {
      sensors[2*n]   = conf.sensorFactor * ((UniversalJoint*)joints[n])->getPosition1Rate();
      //      sensors[2*n]   = factor_sensors * ((UniversalJoint*)joints[n])->getPosition1();
      sensors[2*n+1] = conf.sensorFactor * ((UniversalJoint*)joints[n])->getPosition2Rate();
      //      sensors[2*n+1] = factor_sensors * ((UniversalJoint*)joints[n])->getPosition2();
    }
    return len*2;
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void SchlangeForce::create(const osg::Matrix& pose){
    Schlange::create(pose);
    
    //*****************joint definition***********
    for ( int n = 0; n < conf.segmNumber-1; n++ ) {		

      Pos p1(objects[n]->getPosition());
      Pos p2(objects[n+1]->getPosition());
      
      UniversalJoint* j = new UniversalJoint(objects[n], objects[n+1],
 					     (p1+p2)/2,
 					     Axis(0,0,1)*pose, Axis(0,1,0)*pose);
      j->init(odeHandle, osgHandle, true, conf.segmDia * 1.02);
      
      // setting stops at universal joints		
      j->setParam(dParamLoStop, -conf.jointLimit);
      j->setParam(dParamHiStop,  conf.jointLimit);
      j->setParam(dParamLoStop2, -conf.jointLimit);
      j->setParam(dParamHiStop2,  conf.jointLimit);
      
      // making stops bouncy
      j->setParam (dParamBounce, 0.9 );
      j->setParam (dParamBounce2, 0.9 ); // universal

      joints.push_back(j); 

      frictionmotors.push_back(new AngularMotor2Axis(odeHandle, j, 
						     conf.frictionJoint, conf.frictionJoint)
			       );
    }	  
  }


  /** destroys vehicle and space
   */
  void SchlangeForce::destroy(){  
    Schlange::destroy();  
  }

}
