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
 *                                                                 *
 ***************************************************************************
 *                                                                         *
 * Spherical Robot magically driven                                        *
 *                                                                         *
 *   $Log$
 *   Revision 1.6  2006-12-21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.5  2006/08/11 15:44:35  martius
 *   has conf now and arbitrary sensors
 *
 *   Revision 1.4  2006/08/08 17:04:46  martius
 *   added new sensor model
 *
 *   Revision 1.3  2006/07/14 12:23:40  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.2.4.5  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.2.4.4  2006/01/10 22:25:09  martius
 *   moved to osg
 *
 *
 *                                                                 *
 ***************************************************************************/

#ifndef __FORCESSPHERE_H
#define __FORCESSPHERE_H

#include "oderobot.h"
#include "sensor.h"

namespace lpzrobots {

  class Primitive;

  class ForcedSphereConf {
  public:
    ForcedSphereConf();
    ~ForcedSphereConf();
    /// deletes sensors
    void destroy();

    double radius; //< radius of the sphere
    double max_force; ///< maximal force applied to the sphere
    /// bit mask for selecting the dimensions for the forces (see ForcedSphere::Dimensions)
    short drivenDimensions; 
    /// list of sensors that are mounted at the robot. (e.g.\ AxisOrientationSensor)
    std::list<Sensor*> sensors; 
    /// adds a sensor to the list of sensors
    void addSensor(Sensor* s) { sensors.push_back(s); }    
  };

  class ForcedSphere : public OdeRobot
  {
  protected:
    Primitive* object[1];
    bool created;
    ForcedSphereConf conf;

  public:
  
    enum Dimensions { X = 1, Y = 2, Z = 4 };

    /**
     * constructor
     * 
     * use getDefaultConf() to obtain a configuration with default values, which can be altered 
     *  to personal needs.
     **/ 
    ForcedSphere ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		   const ForcedSphereConf& ForcedSphereConf, const std::string& name);
  
    virtual ~ForcedSphere();
	
    static ForcedSphereConf getDefaultConf(){
      ForcedSphereConf c;
      c.radius = 1;
      c.max_force = 1;     
      c.drivenDimensions = X | Y;
      return c;      
    }

    virtual void update();

    virtual void place(const osg::Matrix& pose);
  
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
    virtual void doInternalStuff(const GlobalData& globalData);
	
    virtual int getSensors ( sensor* sensors, int sensornumber );
    virtual void setMotors ( const motor* motors, int motornumber );
    virtual int getMotorNumber();
    virtual int getSensorNumber();
	 
    virtual Primitive* getMainPrimitive() const { return object[0]; }

  protected:

    virtual void create(const osg::Matrix& pose); 
    virtual void destroy(); 


  };

}

#endif
