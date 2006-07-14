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
 *   Revision 1.2  2006-07-14 12:23:41  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.4  2006/06/25 17:00:33  martius
 *   Id
 *
 *   Revision 1.1.2.3  2006/06/25 16:57:15  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.2  2006/02/23 18:05:05  martius
 *   friction with angularmotor
 *
 *   Revision 1.1.2.1  2006/02/01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *
 ***************************************************************************/

#include "schlangeservo2.h"

namespace lpzrobots {

SchlangeServo2::SchlangeServo2 ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			       const SchlangeConf& conf, const std::string& name) 
  : Schlange(odeHandle, osgHandle, conf, name, "$Id$") 
{

}
	
SchlangeServo2::~SchlangeServo2() { }
	

/**
 *Reads the actual motor commands from an array, an sets all motors (forces) of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 **/
void SchlangeServo2::setMotors ( const motor* motors, int motornumber )
{
  assert(created);
  int len = min(motornumber, getMotorNumber())/2;
  // controller output as torques 
  for (int i = 0; i < len; i++){
    servos[i]->set(motors[2*i], motors[2*i+1]);
  }
}	

/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the arrays

 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 **/
int SchlangeServo2::getSensors ( sensor* sensors, int sensornumber )
{
  assert(created);
  int len = min(sensornumber, getSensorNumber())/2;
  
  for (int n = 0; n < len; n++) {
    sensors[2*n] = servos[n]->get1();
    sensors[2*n+1] = servos[n]->get2();
  }
	
  return 2*len;
}


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void SchlangeServo2::create(const osg::Matrix& pose){
    Schlange::create(pose);
    
    //*****************joint definition***********
    for ( int n = 0; n < conf.segmNumber-1; n++ ) {		

      const Pos& p1(objects[n]->getPosition());
      const Pos& p2(objects[n+1]->getPosition());
      UniversalJoint* j = new UniversalJoint(objects[n], objects[n+1],
					     (p1 + p2)/2,
					     Axis(0,0,1)* pose, Axis(0,1,0)* pose);
      j->init(odeHandle, osgHandle, true, conf.segmDia * 1.02);
        
      // setting stops at universal joints		
      j->setParam(dParamLoStop, -conf.jointLimit*1.5);
      j->setParam(dParamHiStop,  conf.jointLimit*1.5);
    
      // making stops bouncy
      //    j->setParam (dParamBounce, 0.9 );
      //    j->setParam (dParamBounce2, 0.9 ); // universal

      joints.push_back(j); 
      
      UniversalServo* servo =  new UniversalServo(j, -conf.jointLimit, conf.jointLimit, conf.motorPower,
					          -conf.jointLimit, conf.jointLimit, conf.motorPower);
      servos.push_back(servo);
      
      frictionmotors.push_back(new AngularMotor2Axis(odeHandle, j, 
						     conf.frictionJoint, conf.frictionJoint)
			       );
    }	  
  }


bool SchlangeServo2::setParam(const paramkey& key, paramval val){
  bool rv = Schlange::setParam(key, val);
  for (vector<UniversalServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
    if(*i) (*i)->setPower(conf.motorPower, conf.motorPower);
  }
  return rv;
}


/** destroys vehicle and space
 */
void SchlangeServo2::destroy(){  
  if (created){
    Schlange::destroy();
    for (vector<UniversalServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
      if(*i) delete *i;
    }
    servos.clear();
  }
}

}
