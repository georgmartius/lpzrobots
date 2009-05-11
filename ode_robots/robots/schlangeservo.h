/************************************************************************/
/*schlangeservo.h							*/
/*Snake with PID Servo motors (just one motor per joint)     		*/
/*@author Georg Martius 						*/
/*									*/
/************************************************************************/
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
 *   Revision 1.10  2009-05-11 15:44:30  martius
 *   new velocity servos used
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
 *   Revision 1.5.4.7  2006/06/25 16:57:15  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.5.4.6  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.5.4.5  2006/02/01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.5.4.4  2005/12/29 16:46:24  martius
 *   inherits from Schlange
 *   moved to osg
 *
 *   Revision 1.5.4.3  2005/11/16 11:26:53  martius
 *   moved to selforg
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
 *                                                                 *
 ***************************************************************************/
#ifndef __SCHLANGESERVO_H
#define __SCHLANGESERVO_H


#include "schlange.h"
#include "oneaxisservo.h"

namespace lpzrobots {

  /**
   * This is a class, which models a snake like robot. 
   * It consists of a number of equal elements, each linked 
   * by a joint hinge powered by 1 servos
   **/
  class SchlangeServo: public Schlange
  {
  private:
    std::vector <HingeServo*> servos;

  public:
    SchlangeServo ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		    const SchlangeConf& conf, 
		    const std::string& name, const std::string& revision="");

    
    virtual ~SchlangeServo();
	
    /**
     *Reads the actual motor commands from an array, 
     *an sets all motors of the snake to this values.
     *It is an linear allocation.
     *@param motors pointer to the array, motor values are scaled to [-1,1] 
     *@param motornumber length of the motor array
     **/
    virtual void setMotors ( const motor* motors, int motornumber );

    /**
     *Writes the sensor values to an array in the memory.
     *@param sensors pointer to the array
     *@param sensornumber length of the sensor array
     *@return number of actually written sensors
     **/
    virtual int getSensors ( sensor* sensors, int sensornumber );
	
    /** returns number of sensors
     */
    virtual int getSensorNumber() { assert(created); return servos.size(); }

    /** returns number of motors
     */
    virtual int getMotorNumber(){ assert(created); return servos.size(); }

    virtual bool setParam(const paramkey& key, paramval val);

  private:
    virtual void create(const osg::Matrix& pose);
    virtual void destroy();
  };

}

/*
Parameters for invertmotornstep controller:
motorpower=5
sensorfactor=5

controller->setParam("adaptrate", 0.000);
controller->setParam("epsC", 0.01);
controller->setParam("epsA", 0.01);
controller->setParam("dampA", 0.0001);
controller->setParam("rootE", 1);
controller->setParam("steps", 1);
controller->setParam("s4avg", 1);
controller->setParam("s4del", 5);
global.odeConfig.setParam("controlinterval",4);


High frequency modes and then low-dim modes occur with
controller->setParam("adaptrate", 0.0001);
controller->setParam("nomupdate", 0.0007);
controller->setParam("epsC", 0.01);
controller->setParam("epsA", 0.01);
controller->setParam("dampA", 0.0001);
controller->setParam("rootE", 1);
controller->setParam("steps", 1);
controller->setParam("s4avg", 1);
controller->setParam("s4del", 1);
global.odeConfig.setParam("controlinterval",1);
    

*/


#endif

