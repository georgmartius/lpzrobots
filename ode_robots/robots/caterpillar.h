/**************************************************************************/
/*schlangeservo.h							  */
/*Snake with PID Servo motors (just motor per joint)     		  */
/*@author Georg Martius 						  */
/*								     	  */
/**************************************************************************/
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
 *   Revision 1.6  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.5  2007/02/12 13:28:38  martius
 *   twoaxiservos
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
 *   Revision 1.1.2.3  2006/05/09 08:46:47  robot3
 *   getSensors() and getMotors() modified
 *
 *   Revision 1.1.2.2  2006/04/11 13:27:00  robot3
 *   caterpillar is using now methods from schlangeservo2
 *
 *   Revision 1.1.2.1  2006/04/11 08:07:43  robot3
 *   first version
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __CATERPILLAR_H
#define __CATERPILLAR_H

#include "defaultCaterpillar.h"
#include "twoaxisservo.h"
#include "oneaxisservo.h"

namespace lpzrobots {

  /**
   * This is a class, which models a snake like robot. 
   * It consists of a number of equal elements, each linked 
   * by a joint powered by 2 servos
   **/
  class CaterPillar : public DefaultCaterPillar
    {
  private:
    std::vector <UniversalServo*> universalServos;
    std::vector <SliderServo*> sliderServos;

  public:
      CaterPillar ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      const CaterPillarConf& conf, const std::string& name);
    
      virtual ~CaterPillar();
	
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
    virtual int getSensorNumber() { assert(created); return 2*universalServos.size()+sliderServos.size(); }

    /** returns number of motors
     */
    virtual int getMotorNumber(){ assert(created); return 2*universalServos.size()+sliderServos.size(); }

    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);

  private:
    virtual void create(const osg::Matrix& pose);
    virtual void destroy();
  };

}

#endif
