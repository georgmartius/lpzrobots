
#include "drawgeom.h" 


// draws a geom

void drawGeom (dGeomID g, const dReal *pos, const dReal *R)
{
  int i;
	
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
