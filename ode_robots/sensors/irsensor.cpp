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
 *   Revision 1.4.4.3  2005-12-14 12:43:07  martius
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
#include <ode/ode.h>
#include <math.h>
#include <assert.h>
#include <selforg/position.h>
#include <osg/Matrix>
#include <osg/Vec3>

#include "primitive.h"
#include "osgprimitive.h"
#include "irsensor.h"

namespace lpzrobots {

IRSensor::IRSensor(double exponent/* = 1*/){
  value = 0;  
  len=0;
  ray=0;
  this->exponent = exponent;
  initialised = false;
  sensorBody = 0;
  sensorRay  = 0;
}

IRSensor::~IRSensor(){
  dGeomDestroy(transform);     
}

void IRSensor::init(const OdeHandle& odeHandle,
		    const OsgHandle& osgHandle, 
		    Primitive* body, 
		    const osg::Matrix pose, double range,
		    rayDrawMode drawMode){
  this->range = range;
  this->osgHandle = osgHandle;
  value = 0;
  len   = range;

  transform = dCreateGeomTransform (odeHandle.space); 
  dGeomTransformSetInfo(transform, 0);   
  dGeomTransformSetCleanup(transform, 1); // destroy ray geom of transform is created

  ray = dCreateRay ( 0, range); 
  osg::Vec3 p = pose.getTrans();
  dGeomSetPosition (ray, p.x(), p.y(), p.z());
  dMatrix3 rot;
  odeRotation(pose, rot);
  dGeomSetRotation(ray, rot);

  dGeomTransformSetGeom(transform, ray);  
  dGeomSetBody ( transform, body->getBody() );
  dGeomDisable (transform); // disable transform geom, so that it is not treated by normal collision detection.


  switch(drawMode){
  case drawAll:
  case drawRay:     
    sensorRay = new OSGBox(0.002, 0.002, len);
    sensorRay->init(osgHandle);
    if( drawMode != drawAll) break;
  case drawSensor:
    sensorBody = new OSGCylinder(0.05, 0.01);
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
  
bool IRSensor::sense(dGeomID object){
  assert(initialised);
  int n; 
  bool rv = false;
  dContact contact;
  dGeomEnable (transform); // enable transform geom of this ray
  n = dCollide (object, transform, 1, &contact.geom, sizeof(dContact));
  if(n) {
    //     printf("ray: %x\n",ray);
    //     printf("coll between: %x  %x\n",contact.geom.g1,contact.geom.g2);
    len = contact.geom.depth;
    value = characteritic(len);
    //    printf("len= %f, value: %f, \n",len, value);
    rv = true;
  } 
  dGeomDisable (transform);// disable transform geom, so that it is not treated by normal collision detection.
  return rv;  
}


double IRSensor::get(){
  return value;
}

void IRSensor::update(){  
  const dReal* pos = dGeomGetPosition (transform);
  const dReal* R = dGeomGetRotation (transform);
  const dReal *pos2 = dGeomGetPosition (ray); // local position
  const dReal *R2 = dGeomGetRotation (ray);   // local rotation
  dVector3 actual_pos;
  dMatrix3 actual_R;
  dMULTIPLY0_331 (actual_pos,R,pos2);
  actual_pos[0] += pos[0];
  actual_pos[1] += pos[1];
  actual_pos[2] += pos[2];
  dMULTIPLY0_333 (actual_R,R,R2);

  if(sensorRay)  {
    sensorRay->remove(osgHandle);
    delete sensorRay;
    sensorRay = new OSGBox(0.002, 0.002, len);
    sensorRay->init(osgHandle);
    sensorRay->setMatrix(osg::Matrix::translate(osg::Vec3(0,0,len/2.0)) * osgPose(actual_pos,actual_R));
    sensorRay->setColor(Color(value*2.0, 0.0, 0.0));
  }
  if(sensorBody) {
    sensorBody->remove(osgHandle);
    delete sensorBody;
    sensorBody = new OSGCylinder(0.05, 0.01);
    sensorBody->init(osgHandle);
    sensorBody->setMatrix(osgPose(actual_pos,actual_R));
    sensorBody->setColor(Color(value*2.0, 0.0, 0.0));
  }
}

  
dGeomID IRSensor::getGeomID(){
  return ray;
}

double IRSensor::characteritic(double len){
  double v = (range - len)/range;
  return v < 0 ? 0 : pow(v, exponent);
}

}
