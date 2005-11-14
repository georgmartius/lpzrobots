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
 *   Revision 1.4.4.1  2005-11-14 17:37:20  martius
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
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <math.h>
#include <assert.h>
#include <selforg/position.h>

#include "simulation.h"
#include "irsensor.h"
#include "drawgeom.h"

IRSensor::IRSensor(double exponent/* = 1*/){
  value = 0;  
  len=0;
  ray=0;
  this->exponent = exponent;
  initialised = false;
}

IRSensor::~IRSensor(){
  dGeomDestroy(transform);     
}

void IRSensor::init(dSpaceID space, dBodyID body, const Position& pos, 
		    const dMatrix3 rotation, double range){
  this->range = range;
  value = 0;
  len   = range;

// from sphererobotTest:
  transform = dCreateGeomTransform (space); 
  dGeomTransformSetInfo(transform, 0);   
  dGeomTransformSetCleanup(transform, 1); // destroy ray geom of transform is created

  ray = dCreateRay ( 0, range); 
  dGeomSetRotation (ray, rotation);
  dGeomSetPosition (ray, pos.x, pos.y, pos.z);

  dGeomTransformSetGeom(transform, ray);  
  dGeomSetBody ( transform, body );
  dGeomDisable (transform); // disable transform geom, so that it is not treated by normal collision detection.

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

void IRSensor::draw(rayDrawMode drawMode){
  assert(initialised);
  const dReal* pos = dGeomGetPosition (transform);
  const dReal* R = dGeomGetRotation (transform);    
  const dReal *pos2 = dGeomGetPosition (ray);
  const dReal *R2 = dGeomGetRotation (ray);
  dVector3 actual_pos;
  dMatrix3 actual_R;
  dMULTIPLY0_331 (actual_pos,R,pos2);
  actual_pos[0] += pos[0];
  actual_pos[1] += pos[1];
  actual_pos[2] += pos[2];
  dMULTIPLY0_333 (actual_R,R,R2);
  dVector3 end_pos,end; 

  dsSetColor(value*2,0.0,0.0);
  dsSetTexture(DS_NONE);
  switch(drawMode){
  case drawAll:
  case drawRay: 
    // endposition in the local coordinate system (just length in z-direction)
    end[0]=0; end[1]=0; end[2]=len;  
    // rotate endposition in local coordinate system with rotation matrix R
    dMULTIPLY0_331 (end_pos,actual_R,end);
    // add actual position (of transform object) to get global coordinates
    end_pos[0] += actual_pos[0];
    end_pos[1] += actual_pos[1];
    end_pos[2] += actual_pos[2];     
    dsDrawLine(actual_pos, end_pos);  
    if( drawMode != drawAll) break;
  case drawSensor:
    dsDrawCylinder(actual_pos,actual_R, 0.01, 0.05);  
    break;
  default:
    break;
  }
}

  
dGeomID IRSensor::getGeomID(){
  return ray;
}

double IRSensor::characteritic(double len){
  double v = (range - len)/range;
  return v < 0 ? 0 : pow(v, exponent);
}

