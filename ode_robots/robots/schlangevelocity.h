/************************************************************************/
/*schlangevelocity.h                                                            */
/*Snake with powered by setting angular velocities                      */
/*                                                                        */
/************************************************************************/
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
#ifndef __SCHLANGEVELOCITY_H
#define __SCHLANGEVELOCITY_H

#include "schlange.h"

namespace lpzrobots {

  /**
   * This is a class, which models a snake like robot.
   * It consists of a number of equal elements, each linked
   * by a joint powered by directly setting the angular velocities of the joints
   **/
  class SchlangeVelocity: public Schlange
    {
    private:
      paramval factor_motors;
      paramval factor_sensors;
      paramval friction_joint;


    public:
      SchlangeVelocity ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      const SchlangeConf& conf, const std::string& name);

      static SchlangeConf getDefaultConf(){
        SchlangeConf conf;
        conf.segmNumber = 10;      //  number of snake elements
        conf.segmLength = 0.8;     // length of one snake element
        conf.segmDia    = 0.2;     //  diameter of a snake element
        conf.segmMass   = 0.4;     //  mass of one snake element
        conf.motorPower = 0.3;     //  power of motors
        conf.jointLimit =  M_PI/4;
        return conf;
      }

      virtual ~SchlangeVelocity();

      /**
       *Reads the actual motor commands from an array,
       *an sets all motors of the snake to this values.
       *It is an linear allocation.
       *@param motors pointer to the array, motor values are scaled to [-1,1]
       *@param motornumber length of the motor array
       **/
      virtual void setMotorsIntern( const double* motors, int motornumber );

      /**
       *Writes the sensor values to an array in the memory.
       *@param sensors pointer to the array
       *@param sensornumber length of the sensor array
       *@return number of actually written sensors
       **/
      virtual int getSensorsIntern( sensor* sensors, int sensornumber );

      /** returns number of sensors
       */
      virtual int getSensorNumberIntern() { assert(created); return joints.size() * 2; }

      /** returns number of motors
       */
      virtual int getMotorNumberIntern(){ assert(created); return joints.size() * 2; }

    private:
      virtual void create(const osg::Matrix& pose);
      virtual void destroy();
    };

}

#endif
