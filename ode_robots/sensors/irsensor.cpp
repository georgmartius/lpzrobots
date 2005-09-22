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
 *   Revision 1.1  2005-09-22 12:56:47  martius
 *   ray based sensors
 *
 *                                                                         *
 ***************************************************************************/
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <math.h>

#include "position.h"
#include "irsensor.h"
#include "drawgeom.h"

IRSensor::IRSensor(dSpaceID space, dBodyID body, const Position& pos, dMatrix3 rotation, double range){
  this->range = range;
  value = 0;
  
  transform = dCreateGeomTransform(space);
  // on collisions the contact structure contains the ray reference and not the transform
  dGeomTransformSetInfo(transform, 0);   
  dGeomTransformSetCleanup(transform, 1); // destroy ray geom of transform is created
  ray = dCreateRay (0, range);   // the ray is not plugged into any space
  dGeomTransformSetGeom (transform, ray);
  
  dGeomSetRotation (ray, rotation);
  dGeomSetPosition (ray, pos.x, pos.y, pos.z);
  dGeomSetBody (transform, body);
}

IRSensor::~IRSensor(){
  dGeomDestroy(transform);     
}

void IRSensor::reset(){
  value = 0;
}  
  
bool IRSensor::sense(dGeomID object){
  int n;  
  dContact contact;
  n = dCollide (object, ray, 1, &contact.geom, sizeof(dContact));
  if(n) {
    double len = contact.geom.depth;
    value = characteritic(len);
    return true;
  } else {
    return false;
  }
}

double IRSensor::get(){
  return value;
}

void IRSensor::draw(){
  drawGeom(transform,0,0);
}

  
dGeomID IRSensor::getGeomID(){
  return ray;
}

double IRSensor::characteritic(double len){
  double v = (range - len)/range;
  return v < 0 ? 0 : v;
}



