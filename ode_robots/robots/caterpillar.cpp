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
 *                                                                         *
 *   $Log$
 *   Revision 1.7  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.6  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.5  2007/02/23 15:14:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2007/01/26 12:05:04  martius
 *   servos combinied into OneAxisServo
 *
 *   Revision 1.3  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:39  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.13  2006/07/05 13:12:54  der
 *   fixed sliderServos.size() bug being used in min-function
 *   (compiler said that sliderServos.size() returned size_t
 *    instead of unsigned int)
 *    this problem occured on x86_64 suse linux 10.0 (der)
 *
 *   Revision 1.1.2.12  2006/06/25 17:00:31  martius
 *   Id
 *
 *   Revision 1.1.2.11  2006/06/25 16:57:11  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.10  2006/06/13 04:24:22  robot5
 *   Separated annular and linear placement in different robots.
 *   See "wheelie" for annular segment placement.
 *
 *   Revision 1.1.2.9  2006/05/29 20:28:43  robot5
 *   Annular placement of segments.
 *
 *   Revision 1.1.2.8  2006/05/22 14:19:11  robot5
 *   Added variable conf.firstJoint to represent the first slider type in the alternating sequence.
 *   Code cleaning.
 *
 *   Revision 1.1.2.7  2006/05/16 07:57:48  robot3
 *   fixed getSensor() and getMotor() bug
 *
 *   Revision 1.1.2.6  2006/05/15 21:11:12  robot5
 *   Using slider and universal joints now (alternating)
 *
 *   Revision 1.1.2.5  2006/05/09 08:46:37  robot3
 *   getSensors() and getMotors() modified
 *
 *   Revision 1.1.2.4  2006/05/09 04:24:34  robot5
 *   Modified for testing purposes
 *
 *   Revision 1.1.2.3  2006/04/25 09:03:52  robot3
 *   caterpillar is now represented by a box
 *
 *   Revision 1.1.2.2  2006/04/11 13:26:46  robot3
 *   caterpillar is using now methods from schlangeservo2
 *
 *   Revision 1.1.2.1  2006/04/11 08:09:47  robot3
 *   first version
 *
 *
 ***************************************************************************/

#include "caterpillar.h"
#include "mathutils.h"

using namespace std;

namespace lpzrobots {

  CaterPillar::CaterPillar ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			     const CaterPillarConf& conf, const std::string& n) 
    : DefaultCaterPillar(odeHandle, osgHandle, conf, n, "$Id$")
  {
  }
	
  CaterPillar::~CaterPillar() {
  }
	

  /**
   *Reads the actual motor commands from an array, and sets all motors (forces) of the snake to this values.
   *It is a linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1] 
   *@param motornumber length of the motor array
   **/
  void CaterPillar::setMotors(const motor* motors, int motornumber) {
   assert(created);
   unsigned int len = min(motornumber, getMotorNumber())/2;
   // controller output as torques 
   for(unsigned int i=0; (i<len) && (i<sliderServos.size()); i++) {
    sliderServos[i]->set(motors[i]);
   }
   unsigned int sssize = sliderServos.size();
   unsigned int usedSliders=min(len,sssize);
   for(unsigned int i=0; (i<len) && (i<universalServos.size()); i++) {
    universalServos[i]->set(motors[usedSliders+2*i], motors[usedSliders+2*i+1]);
   }
  }

  /**
   *Writes the sensor values to an array in the memory.
   *@param sensor* pointer to the arrays

   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  int CaterPillar::getSensors(sensor* sensors, int sensornumber) {
   assert(created);
   unsigned int len=min(sensornumber,getSensorNumber());
   // get the SliderServos
   for(unsigned int n=0; (n<len) && (n<sliderServos.size()); n++) {
    sensors[n] = sliderServos[n]->get();
   }
   unsigned int sssize = sliderServos.size();
   unsigned int usedSliders=min(len,sssize);
   // get the universalServos (2 sensors each!)
   for(unsigned int n=0; (n<len) && (n<universalServos.size()); n++) {
    sensors[usedSliders+2*n] = universalServos[n]->get1();
    sensors[usedSliders+2*n+1] = universalServos[n]->get2();
   }
   return len;
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void CaterPillar::create(const osg::Matrix& pose) {
   DefaultCaterPillar::create(pose);
   //*****************joint definition***********
   for(int n=0; n<conf.segmNumber-1; n++) {
    const Pos& p1(objects[n]->getPosition());
    const Pos& p2(objects[n+1]->getPosition());

    if(n%2==0) {
     // new slider joints //
     SliderJoint *s=new SliderJoint(objects[n], objects[n+1], osg::Vec3((0), (conf.segmDia), (0)),Axis(1,0,0)*pose);

     s->init(odeHandle, osgHandle);
     s->setParam(dParamLoStop, -0.2*conf.segmLength);
     s->setParam(dParamHiStop,  0.2*conf.segmLength);
     s->setParam(dParamCFM, 0.1);
     s->setParam(dParamStopCFM, 0.1);
     s->setParam(dParamStopERP, 0.9);
     joints.push_back(s);

     SliderServo *ss = new SliderServo(s,-conf.segmDia,conf.segmDia,conf.segmMass*100,0.01,0);
     sliderServos.push_back(ss);
     // end of slider joints //

    } else {

     // normal servos creating //
     UniversalJoint* j = new UniversalJoint(objects[n], objects[n+1],
					    (p1 + p2)/2,
					    Axis(0,0,1)*pose, Axis(0,1,0)*pose);
     j->init(odeHandle, osgHandle, true, conf.segmDia/2 * 1.02);

     // setting stops at universal joints
     j->setParam(dParamLoStop, -conf.jointLimit*1.5);
     j->setParam(dParamHiStop,  conf.jointLimit*1.5);
     joints.push_back(j);

     UniversalServo* servo =  new UniversalServo(j, -conf.jointLimit, conf.jointLimit, conf.motorPower,
					         -conf.jointLimit, conf.jointLimit, conf.motorPower);
     universalServos.push_back(servo);
     frictionmotors.push_back(new AngularMotor2Axis(odeHandle, j, conf.frictionJoint, conf.frictionJoint));
     // end of normal servos //
    }

   }
  }

  void CaterPillar::notifyOnChange(const paramkey& key) {
   for(vector<UniversalServo*>::iterator i=universalServos.begin(); i!=universalServos.end(); i++) {
    if(*i) (*i)->setPower(conf.motorPower, conf.motorPower);
   }
  }

  /** destroys vehicle and space
   */
  void CaterPillar::destroy() {
   if(created) {
    DefaultCaterPillar::destroy();  
    for (vector<UniversalServo*>::iterator i = universalServos.begin(); i!= universalServos.end(); i++) {
     if(*i) delete *i;
    }
    universalServos.clear();
   }
  }
}
