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
#include "raysensor.h"

namespace lpzrobots {

  // this function is called if the ray has a collision. In the userdata we get the
  //  RaySensor and the depth is in the contact information
  int rayCollCallback(dSurfaceParameters& params, GlobalData& globaldata, void *userdata,
                     dContact* contacts, int numContacts,
                     dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2){

    RaySensor* sensor = (RaySensor*)userdata;
    sensor->setLength(contacts[0].geom.depth, globaldata.sim_step);
    return 0;
  }

  void RaySensor::defaultInit(){
    initialised = false;
    len=0;
    lastlen=-1;
    sensorBody = 0;
    ray=0;
    transform=0;
    detection=10^5;
    lasttimeasked=-1;

    setBaseInfo(SensorMotorInfo("Ray Sensor").changequantity(SensorMotorInfo::Distance).changemin(0));
  }

  RaySensor::RaySensor(){
    this->defaultInit();
  }

  RaySensor::RaySensor(double size , double range, rayDrawMode drawMode)
    : size(size), range(range), drawMode(drawMode)
  {
    this->defaultInit();
  }

  RaySensor::~RaySensor(){
    if(transform) delete(transform);

    //   dGeomDestroy(transform);
    if(sensorBody) delete sensorBody;
  }

  RaySensor* RaySensor::clone() const {
    RaySensor* w = new RaySensor(size, range, drawMode);
    w->setInitData(odeHandle, osgHandle, pose);
    return w;
  }

  int RaySensor::getSensorNumber() const{
    return 1;
  }

  void RaySensor::setPose(const osg::Matrix& pose) {
    this->pose=pose;
    if(transform) {
      // Transform SetPose not implemented. it is easy to do...
      assert("not yet implemented" == 0);
    }
  }

  void RaySensor::init(Primitive* own, Joint* joint) {
    assert(isInitDataSet);
    len   = range;
    ray = new Ray(range, 0 /*0.005*/, len); // 0 means that we use a line
    transform = new Transform(own, ray, pose);
    OdeHandle myOdeHandle(odeHandle);
    transform->init(odeHandle, 0, osgHandle,
                    (drawMode==drawAll || drawMode==drawRay) ? (Primitive::Draw | Primitive::Geom) : Primitive::Geom );
    transform->substance.setCollisionCallback(rayCollCallback,this);

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

  void RaySensor::setLength(double len, long int time){
    detection = std::min(this->detection,len);
  }

  void RaySensor::setRange(double range){
    assert(!initialised);
    this->range = range;
  }

  void RaySensor::setDrawMode(rayDrawMode drawMode){
    assert(!initialised);
    this->drawMode = drawMode;
  }


  bool RaySensor::sense(const GlobalData& globaldata){
    if(globaldata.sim_step != lasttimeasked) {
      len     = std::max(0.0,std::min(detection,range));
      detection=range;
    }
    lasttimeasked=globaldata.sim_step;
    return true;
  }

  int RaySensor::get(sensor* sensors, int length) const {
    assert(length>0);
    sensors[0]=len;
    return 1;
  }

  std::list<sensor> RaySensor::getList() const {
    return {len};
  }


  void RaySensor::update(){
    if(len!=lastlen){
      ray->setLength(len);
      ray->setColor(Color(len*1.5, 0.0, 0.0));
      if(sensorBody) {
        sensorBody->setColor(Color(len*2.0, 0.0, 0.0));
      }
      lastlen=len;
    }
    ray->update();

    if(sensorBody) {
      sensorBody->setMatrix(osg::Matrix::translate(0,0,0.005) *
                            ray->getPose() * transform->getPose());
    }

  }

}
