/************************************************************************/
/*schlangeservo.h							*/
/*Snake with PID Servo motors (just motor per joint)     		*/
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
 *   Revision 1.5  2007-02-12 13:28:38  martius
 *   twoaxiservos
 *
 *   Revision 1.4  2006/09/20 12:56:17  martius
 *   Snakes have CreateSegment
 *
 *   Revision 1.3  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:41  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/06/25 16:57:16  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.2  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.1.2.1  2006/02/01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __SCHLANGESERVO2_H
#define __SCHLANGESERVO2_H


#include "schlange.h"
#include "twoaxisservo.h"

namespace lpzrobots {

  /**
   * This is a class, which models a snake like robot. 
   * It consists of a number of equal elements, each linked 
   * by a joint powered by 2 servos
   **/
  class SchlangeServo2: public Schlange
  {
  private:
    std::vector <UniversalServo*> servos;

  public:
    SchlangeServo2 ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		     const SchlangeConf& conf, 
		     const std::string& name, 
		     const std::string& revision = "" );
    
    virtual ~SchlangeServo2();
	
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
    virtual int getSensorNumber() { assert(created); return 2*servos.size(); }

    /** returns number of motors
     */
    virtual int getMotorNumber(){ assert(created); return 2*servos.size(); }

    virtual bool setParam(const paramkey& key, paramval val);

  private:
    virtual void create(const osg::Matrix& pose);
    virtual void destroy();
  };

}

#endif
