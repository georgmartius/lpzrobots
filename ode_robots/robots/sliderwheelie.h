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
 *   Revision 1.2  2006-07-14 12:23:42  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/06/25 21:57:20  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.2  2006/06/20 07:18:29  robot3
 *   -added cvs log
 *   -changed some behaviour of wheelie
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __SLIDERWHEELIE_H
#define __SLIDERWHEELIE_H

#include "defaultSliderWheelie.h"
#include "hingeservo.h"
#include "sliderservo.h"

namespace lpzrobots {

  /**
   * This is a class, which models an annular robot. 
   * It consists of a number of equal elements, each linked 
   * by a joint powered by 1 servo
   **/
  class SliderWheelie : public DefaultSliderWheelie
    {
  private:
    vector <HingeServo*> hingeServos;
    vector <SliderServo*> sliderServos;

  public:
      SliderWheelie(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      const SliderWheelieConf& conf, const std::string& name);
    
      virtual ~SliderWheelie();
	
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
    virtual int getSensorNumber() { assert(created); return hingeServos.size()+sliderServos.size(); }

    /** returns number of motors
     */
    virtual int getMotorNumber(){ assert(created); return hingeServos.size()+sliderServos.size(); }

    virtual bool setParam(const paramkey& key, paramval val);

  private:
    virtual void create(const osg::Matrix& pose);
    virtual void destroy();
  };

}

#endif
