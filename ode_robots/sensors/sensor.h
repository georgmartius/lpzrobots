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
#ifndef __SENSOR_H
#define __SENSOR_H

#include <list>
#include <selforg/types.h>
#include <selforg/stl_adds.h>
#include <selforg/matrix.h>
#include "globaldata.h"
#include "pos.h"

namespace lpzrobots {

  // forward declaration
  class Primitive;

  /** Abstract class for sensors that can be plugged into a robot
  */
  class Sensor {
  public:
    /// defines which dimensions should be sensed. The meaning is sensor specific.
    enum Dimensions { X = 1, Y = 2, Z = 4, XY = X | Y, XZ = X | Z, YZ = Y | Z, XYZ = X | Y | Z };

    Sensor() {}
    virtual ~Sensor() {}

    /** initialises sensor with body of robot. This is usually done by the robot itself.
    */
    virtual void init(Primitive* own) = 0;

    /** performs sense action
     */
    virtual bool sense(const GlobalData& globaldata) = 0;

    /** returns the number of sensors values produced by this sensor
     */
    virtual int getSensorNumber() const  = 0;

    /** returns a list of sensor values (usually in the range [-1,1] )
	This function should be overloaded.
     */
    virtual std::list<sensor> get() const  = 0;

    /** to update any visual appearance
     */
    virtual void update() {};

    /** writes the sensor values (usually in the range [-1,1] )
	into the given sensor array and returns the number of sensors written.
	A default implementation based on get() is provided. Only if performance
	matters overwrite this function.
	@param sensors call by refernce array which received the values
	@param length capacity of sensors array
	@return number of sensor values written
     */
    virtual int get(sensor* sensors, int length) const {
      const std::list<sensor>& l = get();
      assert(length>=(int)l.size());
      int n=0;
      FOREACHC(std::list<sensor>,l,s)
	sensors[n++] = *s;
      return l.size();
    };

    /// selects the rows specified by dimensions (X->0, Y->1, Z->2)
    static std::list<sensor> selectrows(const matrix::Matrix& m, short dimensions) {
      std::list<sensor> l;
      for(int i=0; i<3; i++){
	if(( 1 <<i ) & dimensions) l += m.row(i).convertToList();
      }
      return l;
    }
    /// selects the rows specified by dimensions (X->0, Y->1, Z->2)
    static int selectrows(sensor* sensors, int length, const matrix::Matrix& m, short dimensions) {
      int len=0;
      for(int i=0; i<3; i++){
	if(( 1 << i) & dimensions)
	  len+=m.row(i).convertToBuffer(sensors+len, length-len);
      }
      return len;
    }

    // parse a string with "xyz" or "XYZ" for specifing the sensors dimension
    static Dimensions parseSensorDimension(char* str){
      int val=0;
      for(unsigned int i=0; i<strlen(str); i++){
        switch(str[i]){
        case 'X':
        case 'x': val|=X; break;
        case 'Y':
        case 'y': val|=Y; break;
        case 'Z':
        case 'z': val|=Z; break;
        }
      }
      if(val==0) {
        fprintf(stderr,"parseSensorDimension:Sensor must have at least one dimension");
        val = X;
      }
      return (Dimensions)val;
    }

  };

}

#endif
