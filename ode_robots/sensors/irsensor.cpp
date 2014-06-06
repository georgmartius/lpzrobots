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
    sensor->setLength(contacts[0].geom.depth, globaldata.sim_step);
    return 0;
  }


  IRSensor::IRSensor(float exponent/* = 1*/, double size /*= 0.05*/,
                     float range /*= 2*/, rayDrawMode drawMode /*= drawSensor*/)
    : exponent(exponent), size(size), range(range), drawMode(drawMode)
  {
    value = 0;
    len=0;
    time=-10;
    lastvalue=-1;
    initialised = false;

    sensorBody = 0;
    ray=0;
    transform=0;
    setBaseInfo(SensorMotorInfo("IR").changequantity(SensorMotorInfo::Distance).changemin(0));

  }

  IRSensor::~IRSensor(){
    if(transform) delete(transform);

    //   dGeomDestroy(transform);
    if(sensorBody) delete sensorBody;
  }

  RaySensor* IRSensor::clone() const {
    IRSensor* w = new IRSensor(exponent, size, range, drawMode);
    w->setInitData(odeHandle, osgHandle, pose);
    return w;
  }

  void IRSensor::setPose(const osg::Matrix& pose) {
    this->pose=pose;
    if(transform) {
      // Transform SetPose not implemented. it is easy to do...
      assert("not yet implemented" == 0);
    }
  }

  osg::Matrix IRSensor::getPose() { return pose;};


  void IRSensor::init(Primitive* own, Joint* joint) {
    assert(isInitDataSet);

    value = 0;
    len   = range;
    lastvalue = -1;

    ray = new Ray(range, 0 /*0.005*/, len); // 0 means that we use a line
    transform = new Transform(own, ray, pose);
    OdeHandle myOdeHandle(odeHandle);
    transform->init(odeHandle, 0, osgHandle,
                    (drawMode==drawAll || drawMode==drawRay) ? (Primitive::Draw | Primitive::Geom) : Primitive::Geom );
    transform->substance.setCollisionCallback(irCollCallback,this);

    switch(drawMode){
    case drawAll:
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

  void IRSensor::setLength(float len, long int time){
    this->len = len;
    this->time = time;
    // printf("len= %f, value: %f, \n",len, value);
  }

  void IRSensor::setRange(float range){
    assert(!initialised);
    this->range = range;
  }

  void IRSensor::setDrawMode(rayDrawMode drawMode){
    assert(!initialised);
    this->drawMode = drawMode;
  }


  bool IRSensor::sense(const GlobalData& globaldata){
    if(time<globaldata.sim_step - globaldata.odeConfig.controlInterval){
      len     = range;
    }
    value     = characteritic(len);
    ray->setLength(len);
    return true;
  }


  double IRSensor::getValue() const {
    return value;
  }

  int IRSensor::get(sensor* sensors, int length) const {
    assert(length>0);
    sensors[0]=value;
    return 1;
  }

  std::list<sensor> IRSensor::getList() const {
    return {value};
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
