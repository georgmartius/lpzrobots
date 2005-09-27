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
 *   Revision 1.6  2005-09-27 12:22:52  robot3
 *   now changing camView by mouse movement really works :)
 *
 *   Revision 1.5  2005/09/27 11:04:18  fhesse
 *   drawing for ray added
 *
 *   Revision 1.4  2005/08/08 11:06:47  martius
 *   camera is a module for camera movements
 *   includes cleaned
 *
 *   Revision 1.3  2005/08/02 13:26:35  fhesse
 *   unused int i removed
 *                                                *
 *   Revision 1.2  2005/08/02 13:20:10  fhesse                             *
 *   head added                                                            *
 *                                                                         *
 *   Revision 1.1  2005/08/02 13:18:33  fhesse                             *
 *   function for drawing geoms                                            *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include "drawgeom.h" 

#include <drawstuff/drawstuff.h>
#include "simulation.h" 


// draws a geom

void drawGeom (dGeomID g, const dReal *pos, const dReal *R)
{
  if (!g) return;
  if (!pos) pos = dGeomGetPosition (g);
  if (!R) R = dGeomGetRotation (g);

  int type = dGeomGetClass (g);
  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths (g,sides);
    dsDrawBox (pos,R,sides);
  }
  else if (type == dSphereClass) {
    dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
  }
  else if (type == dCCylinderClass) {
    dReal radius,length;
    dGeomCCylinderGetParams (g,&radius,&length);
    dsDrawCappedCylinder (pos,R,length,radius);
  }
  else if (type == dRayClass) {
    
    dReal length;
    dVector3 start, dir;
    length=dGeomRayGetLength (g);
    dGeomRayGet(g, start, dir);

    dVector3 end_pos,end; 
    // endposition in the local coordinate system (just length in z-direction)
    end[0]=0;
    end[1]=0;
    end[2]=length;  

    // rotate endposition in local coordinate system with rotation matrix R
    dMULTIPLY0_331 (end_pos,R,end);
    // add actual position (of transform object) to get global coordinates
    end_pos[0] += pos[0];
    end_pos[1] += pos[1];
    end_pos[2] += pos[2];
    // draw line from start(pos) to end
    //    dsDrawLine(pos, end_pos);


  }

/*
  // cylinder option not yet implemented
  else if (type == dCylinderClass) {
    dReal radius,length;
    dGeomCylinderGetParams (g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }
*/
  else if (type == dGeomTransformClass) {
    dGeomID g2 = dGeomTransformGetGeom (g);
    const dReal *pos2 = dGeomGetPosition (g2);
    const dReal *R2 = dGeomGetRotation (g2);
    dVector3 actual_pos;
    dMatrix3 actual_R;
    dMULTIPLY0_331 (actual_pos,R,pos2);
    actual_pos[0] += pos[0];
    actual_pos[1] += pos[1];
    actual_pos[2] += pos[2];
    dMULTIPLY0_333 (actual_R,R,R2);
    drawGeom (g2,actual_pos,actual_R);
  }
}
