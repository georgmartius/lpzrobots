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
 *   Revision 1.3  2010-03-09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.2  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.1  2007/09/06 18:49:40  martius
 *   Sphere reinforcement
 *
 *
 *                                                                         *
 ***************************************************************************/
#include <ode-dbl/ode.h>
#include <math.h>
#include <assert.h>
#include <selforg/position.h>
#include <osg/Matrix>
#include <osg/Vec3>
#include <list>

#include <ode_robots/primitive.h>
#include <ode_robots/osgprimitive.h>
#include "irsensor_wall.h"

namespace lpzrobots {

  using namespace std;

  // this function is called if the ray has a collision. In the userdata we get the
  //  irsensor and the depth is in the contact information
  int irwallCollCallback(dSurfaceParameters& params, GlobalData& globaldata, void *userdata,
                         dContact* contacts, int numContacts,
                         dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2){

    IRSensorWall* sensor = (IRSensorWall*)userdata;
    list<dGeomID>::iterator result = find(sensor->avoids.begin(),sensor->avoids.end(),o2);
    if(result==sensor->avoids.end())
      sensor->setLength(contacts[0].geom.depth);
    return 0;
  }

  IRSensorWall::IRSensorWall(float exponent, const list<dGeomID>& avoids)
    : IRSensor(exponent), avoids(avoids){
  }

  RaySensor* IRSensorWall::clone() const {
    IRSensorWall* w = new IRSensorWall(exponent, avoids);
    return (RaySensor*)w;
  }


  void IRSensorWall::init(const OdeHandle& odeHandle,
                          const OsgHandle& osgHandle,
                          Primitive* body,
                          const osg::Matrix pose, float range,
                          rayDrawMode drawMode){
    IRSensor::init(odeHandle, osgHandle, body, pose,range,  drawMode);
    transform->substance.setCollisionCallback(irwallCollCallback,this);
  };

}
