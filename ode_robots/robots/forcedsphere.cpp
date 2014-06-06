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

#include <assert.h>
#include <selforg/matrix.h>
#include "primitive.h"
#include "forcedsphere.h"
#include "mathutils.h"

using namespace osg;
using namespace std;

namespace lpzrobots {


  ForcedSphereConf::ForcedSphereConf() {}

  ForcedSphereConf::~ForcedSphereConf(){}

  void ForcedSphereConf::destroy(){
    for(list<Sensor*>::iterator i = sensors.begin(); i != sensors.end(); i++){
      if(*i) delete *i;
    }
    sensors.clear();
  }



  ForcedSphere::ForcedSphere ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                               const ForcedSphereConf& conf, const std::string& name)
    : OdeRobot( odeHandle, osgHandle,
                name, "$Id$" ), conf(conf)
  {
    created = false;
    object[0] = 0;
  }

  ForcedSphere::~ForcedSphere()
  {
    destroy();
    conf.destroy();
  }

  void ForcedSphere::update()
  {
    if(object[0]) object[0]->update();
  }

  int ForcedSphere::getSensorsIntern( sensor* sensors, int sensornumber )
  {
    assert(created);
    int len=0;
    FOREACH(list<Sensor*>, conf.sensors, i){
      len += (*i)->get(sensors+len, sensornumber-len);
    }
    return len;
  }

  void ForcedSphere::setMotorsIntern( const double* motors, int motornumber ) {
    assert(created);
    if (motornumber==getMotorNumber()){
      int i=0;
      double x = (conf.drivenDimensions & X) ? motors[i++] : 0;
      double y = (conf.drivenDimensions & Y) ? motors[i++] : 0;
      double z = (conf.drivenDimensions & Z) ? motors[i++] : 0;
      if(!conf.speedDriven)
        dBodyAddForce(object[0]->getBody(), x*conf.maxForce, y*conf.maxForce, z*conf.maxForce);
      else{
        Position nom;
        nom.x=x; nom.y=y; nom.z=z;
        Position diff = (nom*conf.maxSpeed-getSpeed())*0.5*conf.maxForce;
        dBodyAddForce(object[0]->getBody(), diff.x, diff.y, diff.z);

      }
      int len=motornumber-i;
      FOREACH(list<Motor*>, conf.motors, m){
        int l=(*m)->set(motors+i,len);
        i+=l;
        len-=l;
      }

    }
  }


  void ForcedSphere::placeIntern(const osg::Matrix& pose){
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.radius));
    create(p2);
  };


  void ForcedSphere::doInternalStuff(GlobalData& global){
    OdeRobot::doInternalStuff(global);
    FOREACH(list<Motor*>, conf.motors, i){
      (*i)->act(global);
    }
    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->sense(global);
    }

    // slow down rotation around z axis.
    dBodyID b = getMainPrimitive()->getBody();
    const double* vel = dBodyGetAngularVel( b);
    if(fabs(vel[2])>0.05){
      dBodyAddTorque ( b , 0 , 0 , -0.1*conf.maxForce*vel[2] );
    }





  }

  int ForcedSphere::getMotorNumberIntern(){
    int s = 0;
    FOREACHC(list<Motor*>, conf.motors, i){
      s += (*i)->getMotorNumber();
    }
    return s + (conf.drivenDimensions & X) + ((conf.drivenDimensions & Y) >> 1) +
      ((conf.drivenDimensions & Z) >> 2);
  }

  int ForcedSphere::getSensorNumberIntern() {
    int s = 0;
    FOREACHC(list<Sensor*>, conf.sensors, i){
      s += (*i)->getSensorNumber();
    }
    return s;
  }


  void ForcedSphere::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    Transform* f=0;
    if(conf.cylinderBody){
      object[0] = new Cylinder(conf.radius,conf.radius/2);
      Primitive* core = new Box(conf.radius/1.5,conf.radius/1.5,conf.radius/2.5);
      f = new Transform(object[0], core, osg::Matrix::translate(0,0,0));
    }
    else
      object[0] = new Sphere(conf.radius);
    object[0]->init(odeHandle, conf.radius*conf.radius, osgHandle);
    if(conf.cylinderBody){
      f->init(odeHandle, 0, osgHandle);
    }
    object[0]->setPose(pose);
    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->init(object[0]);
    }
    FOREACH(list<Motor*>, conf.motors, i){
      (*i)->init(object[0]);
    }
    created = true;
  }


  void ForcedSphere::destroy(){
    if (created){
      for (int i=0; i<1; i++){
        if(object[i]) delete object[i];
      }
    }
    created=false;
  }

}

