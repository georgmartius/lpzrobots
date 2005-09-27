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
 *   Revision 1.2  2005-09-27 11:03:33  fhesse
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

#include "position.h"
#include "irsensor.h"
#include "drawgeom.h"

IRSensor::IRSensor(){
  value = 0;  
  // TODO initialised
}

IRSensor::~IRSensor(){
  //  dGeomDestroy(transform);     
}

void IRSensor::init(dSpaceID space, dBodyID body, const Position& pos, 
		    const dMatrix3 rotation, double range){
  this->range = range;
  value = 0;

// from sphererobotTest:
  transform = dCreateGeomTransform (space); 
  dGeomTransformSetInfo(transform, 0);   
  dGeomTransformSetCleanup(transform, 1); // destroy ray geom of transform is created

  ray = dCreateRay ( 0, range); 
  dGeomSetRotation (ray, rotation);
  dGeomSetPosition (ray, pos.x, pos.y, pos.z);

  dGeomTransformSetGeom(transform, ray);  
  dGeomSetBody ( transform, body );

}; 

void IRSensor::reset(){
  value = 0;
}  
  
bool IRSensor::sense(dGeomID object){
  int n;  
  dContact contact;
  n = dCollide (object, transform, 1, &contact.geom, sizeof(dContact));
  if(n) {
//     printf("ray: %x\n",ray);
//     printf("coll between: %x  %x\n",contact.geom.g1,contact.geom.g2);
    double len = contact.geom.depth;
    value = characteritic(len);
    printf("len= %f, value: %f, \n",len, value);

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
//   dsSetColor (0,1,0);
//   drawGeom(transcc,0,0);
}

//   enum rayDrawMode {ray, sphere, no};
// void drawGeom (dGeomID g, const dReal *pos, const dReal *R )
// {
//   if (!g) return;
//   if (!pos) pos = dGeomGetPosition (g);
//   if (!R) R = dGeomGetRotation (g);

//   int type = dGeomGetClass (g);
//   if  (type == dRayClass) {
    
//     dReal length;
//     dVector3 start, dir;
//     length=dGeomRayGetLength (g);
//     dGeomRayGet(g, start, dir);

//     dVector3 end_pos,end; 
//     // endposition in the local coordinate system (just length in z-direction)
//     end[0]=0;
//     end[1]=0;
//     end[2]=length;  

//     // rotate endposition in local coordinate system with rotation matrix R
//     dMULTIPLY0_331 (end_pos,R,end);
//     // add actual position (of transform object) to get global coordinates
//     end_pos[0] += pos[0];
//     end_pos[1] += pos[1];
//     end_pos[2] += pos[2];
//     // draw line from start(pos) to end
//     dsDrawLine(pos, end_pos);


//   }

// /*
//   // cylinder option not yet implemented
//   else if (type == dCylinderClass) {
//     dReal radius,length;
//     dGeomCylinderGetParams (g,&radius,&length);
//     dsDrawCylinder (pos,R,length,radius);
//   }
// */
//   else if (type == dGeomTransformClass) {
//     dGeomID g2 = dGeomTransformGetGeom (g);
//     const dReal *pos2 = dGeomGetPosition (g2);
//     const dReal *R2 = dGeomGetRotation (g2);
//     dVector3 actual_pos;
//     dMatrix3 actual_R;
//     dMULTIPLY0_331 (actual_pos,R,pos2);
//     actual_pos[0] += pos[0];
//     actual_pos[1] += pos[1];
//     actual_pos[2] += pos[2];
//     dMULTIPLY0_333 (actual_R,R,R2);
//     drawGeom (g2,actual_pos,actual_R);
//   }
// }

  
dGeomID IRSensor::getGeomID(){
  return ray;
}

double IRSensor::characteritic(double len){
  double v = (range - len)/range;
  return v < 0 ? 0 : v;
}



