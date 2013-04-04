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
#include <ode-dbl/ode.h>
#include <cmath>
#include <assert.h>
#include <selforg/position.h>
#include <osg/Matrix>
#include <osg/Vec3>

#include "primitive.h"
#include "osgprimitive.h"
#include "irsensor.h"

namespace lpzrobots {

  // this function is called if the ray has a collision. In the userdata we get the
  //  irsensor and the depth is in the contact information
  int irCollCallback(dSurfaceParameters& params, GlobalData& globaldata, void *userdata,
                     dContact* contacts, int numContacts,
                     dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2){

    IRSensor* sensor = (IRSensor*)userdata;
    sensor->setLength(contacts[0].geom.depth);
    return 0;
  }


  IRSensor::IRSensor(float exponent/* = 1*/, double size /*= 0.05*/){
    value = 0;
    len=0;
    lastvalue=-1;
    ray=0;
    this->exponent = exponent;
    this->size = size;
    initialised = false;
    sensorBody = 0;
    //  sensorRay  = 0;

    ray=0;
    transform=0;
  }

  IRSensor::~IRSensor(){
    if(transform) delete(transform);

    //   dGeomDestroy(transform);
    //   if(sensorRay) delete sensorRay;
    if(sensorBody) delete sensorBody;
  }

  RaySensor* IRSensor::clone() const {
    IRSensor* w = new IRSensor(exponent);
    return (RaySensor*)w;
  }

  void IRSensor::init(const OdeHandle& odeHandle,
                      const OsgHandle& osgHandle,
                      Primitive* body,
                      const osg::Matrix pose, float range,
                      rayDrawMode drawMode){
    this->range = range;
    this->osgHandle = osgHandle;
    value = 0;
    len   = range;
    lastvalue = -1;

    ray = new Ray(range, 0.005, len);
    transform = new Transform(body, ray, pose);
    OdeHandle myOdeHandle(odeHandle);
    transform->init(odeHandle, 0, osgHandle,
                    (drawMode==drawAll || drawMode==drawRay) ? (Primitive::Draw | Primitive::Geom) : Primitive::Geom );
    transform->substance.setCollisionCallback(irCollCallback,this);


    // transform = dCreateGeomTransform (odeHandle.space);
    //   dGeomTransformSetInfo(transform, 0);
    //   dGeomTransformSetCleanup(transform, 1); // destroy ray geom of transform is deleted

    //   ray = dCreateRay ( 0, range);
    //   osg::Vec3 p = pose.getTrans();
    //   dGeomSetPosition (ray, p.x(), p.y(), p.z());
    //   dMatrix3 rot;
    //   odeRotation(pose, rot);
    //   dGeomSetRotation(ray, rot);

    //   dGeomTransformSetGeom(transform, ray);
    //   dGeomSetBody ( transform, body->getBody() );
    //   // disable transform geom,
    //   //  so that it is not treated by normal collision detection.
    //   dGeomDisable (transform);

    switch(drawMode){
    case drawAll:
      //   case drawRay:
      //     sensorRay = new OSGBox(0.002, 0.002, len);
      //     sensorRay->init(osgHandle);
      //     if( drawMode != drawAll) break;
    case drawSensor:
      sensorBody = new OSGCylinder(size, size / 5.0);
      sensorBody->init(osgHandle);
      break;
    default:
      break;
    }
    update();
    initialised = true;
  };

  void IRSensor::reset(){
    setLength(range);
  }

  void IRSensor::setLength(float len){
    this->len = len;
    value     = characteritic(len);
    ray->setLength(len);
    // printf("len= %f, value: %f, \n",len, value);
  }

  void IRSensor::setRange(float range){
    this->range = range;
  }

  double IRSensor::get(){
    return value;
  }

  void IRSensor::update(){
    if(value!=lastvalue){
      ray->setColor(Color(value*1.5, 0.0, 0.0));
      if(sensorBody) {
        sensorBody->setColor(Color(value*2.0, 0.0, 0.0));
      }
      lastvalue=value;
    }
    ray->update();

    if(sensorBody) {
      sensorBody->setMatrix(osg::Matrix::translate(0,0,0.005) *
                            ray->getPose() * transform->getPose());
    }

  }

  float IRSensor::characteritic(float len){
    float v = (range - len)/range;
    return v < 0 ? 0 : pow(v, exponent);
  }

}
