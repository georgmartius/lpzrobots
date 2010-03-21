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
 *   Revision 1.6  2010-03-21 21:48:59  martius
 *   camera sensor bugfixing (reference to osghandle)
 *   twowheeled robot added (nimm2 with camera)
 *   sense function added to robots (before control): sensors (type Sensor) are checked here
 *   position and optical flow camera sensors added
 *
 *   Revision 1.5  2010/03/19 17:46:21  martius
 *   camerasensors added
 *   camera works great now. Near and far plane fixed by hand and optimal positioning
 *   many image processings added
 *
 *   Revision 1.4  2007/11/07 13:22:47  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2007/08/23 15:39:05  martius
 *   new IR sensor schema which uses substances and callbacks, very nice
 *
 *   Revision 1.2  2006/12/21 11:42:10  martius
 *   sensors have dimension to sense
 *   axissensors have finer settings
 *
 *   Revision 1.1  2006/08/08 11:59:01  martius
 *   new abstract class for sensors
 *
 *   Revision 1.1  2005/11/22 10:24:04  martius
 *   abstract class for position sensor
 *
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

    /** returns a list of sensor values (usually in the range [0,1] )
     */
    virtual std::list<sensor> get() const  = 0;

    /** to update any visual appearance       
     */
    virtual void update() {};

    /** writes the sensor values (usually in the range [0,1] ) 
	into the giben sensor array and returns the number of sensors written
	@param sensors call by refernce array which received the values
	@param length capacity of sensors array
	@return number of sensor values written
     */
    virtual int get(sensor* sensors, int length) const  = 0;

    /// selects the rows specified by dimensions (X->0, Y->1, Z->2)
    static std::list<sensor> selectrows(const matrix::Matrix& m, short dimensions) {
      std::list<sensor> l;
      for(int i=0; i<2; i++){
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

  };

}

#endif
