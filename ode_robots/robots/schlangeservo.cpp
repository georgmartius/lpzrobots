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
 *   Revision 1.10  2007-02-12 13:28:38  martius
 *   twoaxiservos
 *
 *   Revision 1.9  2007/01/26 12:05:04  martius
 *   servos combinied into OneAxisServo
 *
 *   Revision 1.8  2006/09/20 12:56:17  martius
 *   Snakes have CreateSegment
 *
 *   Revision 1.7  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.6  2006/07/14 12:23:41  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.5.4.8  2006/06/25 17:00:32  martius
 *   Id
 *
 *   Revision 1.5.4.7  2006/06/25 16:57:15  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.5.4.6  2006/02/23 18:05:05  martius
 *   friction with angularmotor
 *
 *   Revision 1.5.4.5  2006/02/01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.5.4.4  2005/12/30 22:52:52  martius
 *   joint axis have right size
 *
 *   Revision 1.5.4.3  2005/12/29 16:46:52  martius
 *   inherits from Schlange
 *   moved to osg
 *
 *   Revision 1.5.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.5.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.5  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/

#include "schlangeservo.h"
using namespace std;

namespace lpzrobots {

SchlangeServo::SchlangeServo ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			       const SchlangeConf& conf, const std::string& name, 
			       const std::string& revision) 
  : Schlange(odeHandle, osgHandle, conf, name, 
	     revision.empty() ? "$Id$" : revision) 
{

}
	
SchlangeServo::~SchlangeServo() { }
	

/**
 *Reads the actual motor commands from an array, an sets all motors (forces) of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 **/
void SchlangeServo::setMotors ( const motor* motors, int motornumber )
{
  assert(created);
  int len = min(motornumber, (int)servos.size());
  // controller output as torques 
  for (int i = 0; i < len; i++){
    servos[i]->set(motors[i]);
  }
}	

/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the arrays

 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 **/
int SchlangeServo::getSensors ( sensor* sensors, int sensornumber )
{
  assert(created);
  int len = min(sensornumber, getSensorNumber());
  
  for (int n = 0; n < len; n++) {
    sensors[n] = servos[n]->get();
  }
	
  return len;
}


/** creates vehicle at desired position 
    @param pos struct Position with desired position
*/
void SchlangeServo::create(const osg::Matrix& pose){
  Schlange::create(pose);
  
  //*****************joint definition***********
  for ( int n = 0; n < conf.segmNumber-1; n++ ) {		
    
    Pos p1(objects[n]->getPosition());
    Pos p2(objects[n]->getPosition());
    HingeJoint* j = new HingeJoint(objects[n], objects[n+1],
				   (objects[n]->getPosition() + objects[n+1]->getPosition())/2,
				   Axis(0,0,1)* pose);
    j->init(odeHandle, osgHandle, true, conf.segmDia * 1.02);
    
    // setting stops at hinge joints		
    j->setParam(dParamLoStop, -conf.jointLimit*1.5);
    j->setParam(dParamHiStop,  conf.jointLimit*1.5);
    
    // making stops bouncy
    //    j->setParam (dParamBounce, 0.9 );
      //    j->setParam (dParamBounce2, 0.9 ); // universal
    
    joints.push_back(j); 
    
    HingeServo* servo =  new HingeServo(j, -conf.jointLimit, conf.jointLimit, conf.motorPower,conf.frictionJoint,0);
    servos.push_back(servo);

    //    frictionmotors.push_back(new AngularMotor1Axis(odeHandle, j, conf.frictionJoint) );
  }	  
}

bool SchlangeServo::setParam(const paramkey& key, paramval val){
  bool rv = Schlange::setParam(key, val);
  for (vector<HingeServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
    if(*i) (*i)->setPower(conf.motorPower);
  }
  for (vector<HingeServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
    if(*i) (*i)->damping()=conf.frictionJoint;
  }

  return rv;    
}


/** destroys vehicle and space
 */
void SchlangeServo::destroy(){  
  if (created){
    Schlange::destroy();
    for (vector<HingeServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
      if(*i) delete *i;
    }
    servos.clear();
  }
}

}
