/***************************************************************************
 *   Copyright (C) 2005-2012 LpzRobots development team                    *
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
#include <ode-dbl/ode.h>
#include <cmath>
#include <assert.h>
#include <selforg/position.h>
#include <osg/Matrix>
#include <osg/Vec3>

#include <ode_robots/primitive.h>
#include <ode_robots/osgprimitive.h>
#include <ode_robots/osghandle.h>
#include <ode_robots/contactsensor.h>

namespace lpzrobots {

  // this function is called if the sensor object has a collision. In the userdata we get the 
  //  contactsensor and the depth is in the contact information
  int contactCollCallback(dSurfaceParameters& params, GlobalData& globaldata, void *userdata, 
                          dContact* contacts, int numContacts,
                          dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2){
    
    ContactSensor* sensor = (ContactSensor*)userdata;
    sensor->setDepth(contacts[0].geom.depth);
    return 0;    
  }



  ContactSensor::ContactSensor(bool binary /*=true*/,
                               double forcescale /*= 1*/, double radius /*= 0.05*/)
    : binary(binary), forcescale(forcescale), size(radius) {
    reference = 0;
    value = 0;  
    lastvalue=-1;
    initialised = false;
    sensorBody = 0;
    transform=0;
  }

  ContactSensor::~ContactSensor(){
    if(transform) {
      delete(transform);
    } else {
      reference->substance.setCollisionCallback(0,this); // remove collision callback
    }
    if(sensorBody) delete sensorBody;        
  }


  void ContactSensor::init(const OdeHandle& odeHandle,
                           const OsgHandle& osgHandle, 
                           Primitive* reference, 
                           bool createSphere,
                           const osg::Matrix pose, 
                           bool colorObject){
    assert(reference);
    this->colorObject = colorObject;
    this->reference   = reference;
    
    value = 0;
    lastvalue = -1;
    if(createSphere){
      sensorBody = new Sphere(size);
      transform = new Transform(reference, sensorBody, pose);
      origColor = osgHandle.getColor("joint");
      transform->init(odeHandle, 0, osgHandle.changeColor(origColor));
      transform->substance.setCollisionCallback(contactCollCallback,this);
    }else{
      reference->substance.setCollisionCallback(contactCollCallback,this);
      if(reference->getOSGPrimitive())
        origColor = reference->getOSGPrimitive()->getColor();
      else
        colorObject = false;
    }
  
    update();
    initialised = true;
  }; 

  void ContactSensor::reset(){
    value=0;
  }  
  
  void ContactSensor::setDepth(float depth){
    if(binary && depth>0) 
      value=1.0;
    else
      value = std::max(value,depth*forcescale);
    //    printf("depth= %f, value: %f, \n",depth, value);
  }

  double ContactSensor::get(){
    return value;
  }

  void ContactSensor::update(){  
    if(value!=lastvalue){
      if(colorObject){
        const Color& col = value > 0 ? Color(value,0,0) : origColor;
        if(sensorBody){
          sensorBody->setColor(col);          
        }else{
          reference->setColor(col);          
        }
      }
      lastvalue=value;
    }  
    if(sensorBody) {    
      sensorBody->update();
    }
  }
  
}
