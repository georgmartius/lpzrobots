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
 *   Revision 1.14  2010-03-09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.13  2009/01/20 17:29:52  martius
 *   cvs commit
 *
 *   Revision 1.12  2008/09/16 14:53:59  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.11  2007/09/06 18:48:29  martius
 *   clone function (a bit like a factory)
 *
 *   Revision 1.10  2007/08/24 12:48:04  martius
 *   tranformation for sensor body fixed
 *
 *   Revision 1.9  2007/08/23 15:39:05  martius
 *   new IR sensor schema which uses substances and callbacks, very nice
 *
 *   Revision 1.8  2007/04/03 16:32:21  der
 *   thicker drawing
 *
 *   Revision 1.7  2006/12/11 18:25:08  martius
 *   memory freed
 *
 *   Revision 1.6  2006/09/20 12:56:28  martius
 *   setRange
 *
 *   Revision 1.5  2006/07/14 12:23:43  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.4.4.5  2006/01/26 18:37:20  martius
 *   *** empty log message ***
 *
 *   Revision 1.4.4.4  2005/12/14 15:37:19  martius
 *   sensors are working with osg
 *
 *   Revision 1.4.4.3  2005/12/14 12:43:07  martius
 *   moved to osg
 *
 *   Revision 1.4.4.2  2005/12/13 18:11:53  martius
 *   sensors ported, but not yet finished
 *
 *   Revision 1.4.4.1  2005/11/14 17:37:20  martius
 *   moved to selforg
 *
 *   Revision 1.4  2005/11/08 11:34:31  martius
 *   geom is only enabled in sense function
 *   there is no external collision detection anymore
 *
 *   Revision 1.3  2005/09/27 13:59:26  martius
 *   ir sensors are working now
 *
 *   Revision 1.2  2005/09/27 11:03:33  fhesse
 *   sensorbank added
 *
 *   Revision 1.1  2005/09/22 12:56:47  martius
 *   ray based sensors
 *
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
    value = 0;
    len   = range;
  }  
  
  void IRSensor::setLength(float len){
    this->len = len;
    value = characteritic(len);
    ray->setLength(len);
    //    printf("len= %f, value: %f, \n",len, value);
  }

  void IRSensor::setRange(float range){
    this->range = range;
  }

  double IRSensor::get(){
    return value;
  }

  void IRSensor::update(){  

    ray->setColor(Color(value*2.0, 0.0, 0.0));
    ray->update();
  
    if(sensorBody) {    
      sensorBody->setMatrix(osg::Matrix::translate(0,0,0.005) * ray->getPose() * transform->getPose());
      sensorBody->setColor(Color(value*2.0, 0.0, 0.0));
    }

  }
  
  float IRSensor::characteritic(float len){
    float v = (range - len)/range;
    return v < 0 ? 0 : pow(v, exponent);
  }

}
