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
 *   Revision 1.2  2005-11-22 13:03:15  robot3
 *   bugfixing
 *
 *   Revision 1.1  2005/11/15 14:29:25  robot3
 *   first version
 *
 *                                                                         *
 ***************************************************************************/
#include "degreesegment.h"
#include "mathutils.h"
#include "stl_adds.h"


void DegreeSegment::setProperties() {
  radius=10.0f; // INTERNAL radius
  width=5.0f; // INTERNAL width
  widthWall=0.1f;
  heightWall=0.7f;
  angle=90.0f/180.0f*M_PI;
  left=1;
  color=Color(226 / 255.0, 103 / 255.0, 66 / 255.0);
}

void DegreeSegment::setCurveAngle(const double& alpha) {
  if (alpha>=0)
    left=1;
  else
    left=0;
  angle=alpha;
}

void DegreeSegment::setRadius(const double& rad) {
  radius=rad;
}


/**
 * returns the inner coordinates that are responsible for the segment of the
 * given length and alpha. 
 */
Position DegreeSegment::getInnerCoordinates(double length, double alpha) {
  Matrix l,r,t;
  if (left==1) {
    l = ::getPositionMatrix(Position(0.0f,-length,0.0f));
    // now rotate the l with alpha
    r = getRotationMatrix(alpha);
    // then translate it with (0,-radius,0) if right curve
    t = getTranslationMatrix(Position(0.0f,radius,0.0f));
  } else {
    l = ::getPositionMatrix(Position(0.0f,length,0.0f));
    // now rotate the l with alpha
    r = getRotationMatrix(alpha);
    // then translate it with (0,-radius,0) if right curve
    t = getTranslationMatrix(Position(0.0f,-radius,0.0f));
  }
  // do it all in one step
  Matrix p = t * ( r * l);
  return getPosition4x1(p);
}


/**
 * returns the outer coordinates (world) that are responsible for the
 * segment of the given length and alpha. 
 */
Position DegreeSegment::getOuterCoordinates(double length, double alpha) {
  Matrix l,r,t;
  if (left==1) {
    l = ::getPositionMatrix(Position(0.0f,-length,0.0f));
    // now rotate the l with alpha
    r = getRotationMatrix(alpha);
    // then translate it with (0,-radius,0) if right curve
    t = getTranslationMatrix(Position(0.0f,radius,0.0f));
  } else {
    l = ::getPositionMatrix(Position(0.0f,length,0.0f));
    // now rotate the l with alpha
    r = getRotationMatrix(alpha);
    // then translate it with (0,-radius,0) if right curve
    t = getTranslationMatrix(Position(0.0f,-radius,0.0f));
  }
  // do it all in one step
  Matrix p = posMatrix * (t * ( r * l));
  return getPosition4x1(p);
}


/**
 * Constructor
 */
DegreeSegment::DegreeSegment() : posMatrix(4,4) { // GLOBAL
  // now position and rotation is all 0
  setProperties();
  posMatrix.toId();
};

/**
 * Constructor
 */
DegreeSegment::DegreeSegment(const Position& p,const double& angle) { // GLOBAL
  // 2 matrices are calculated, the sum is the matrix to store
  setProperties();
  posMatrix + getTranslationRotationMatrix(p,angle);
};

/**
 * Constructor
 */
DegreeSegment::DegreeSegment(const Matrix& pose) : posMatrix(pose) { // GOBAL
  setProperties();

  assert (pose.getM()==4 && pose.getN()==4);
  std::cout << "Pos of DEGREEseg:\t" << pose;
};


Position DegreeSegment::getPosition() {
  return ::getPosition(posMatrix);
}

  /** 
   * gives the position and rotation(angle) of the segment
   * in the real world.
   * returns a matrix.
   */
Matrix DegreeSegment::getPositionMatrix() { // GLOBAL
  return posMatrix;
};


  /** 
   * sets the position and rotation(angle) of the segment
   * in the real world.
   */
void DegreeSegment::setPositionMatrix(const Matrix& pose) { // GLOBAL
  assert (pose.getM()==4 && pose.getN()==4);
  posMatrix=pose;
};

  /** 
   * sets the position of the segment
   * in the real world.
   */
void DegreeSegment::setPosition(const Position& pos) { // GLOBAL
  // the new posMatrix is the old rotation plus the new translation part
  // therefore the old translation part must be removed first
  posMatrix=getTranslationRotationMatrix(pos,getAngle(posMatrix));
};

void DegreeSegment::setRotation(const double& angle) { // GLOBAL
  // the new posMatrix is the old position plus the new translation part
  // therefore the old rotation part must be removed first
  posMatrix = getTranslationRotationMatrix(::getPosition(posMatrix),angle);
}

void DegreeSegment::setRotation(const Matrix& rot) { // GLOBAL
  // the new posMatrix is the old position plus the new translation part
  // therefore the old rotation part must be removed first
  posMatrix=removeRotationInMatrix(posMatrix)*rot;
}

Matrix DegreeSegment::getRotation() { // GLOBAL
  assert (posMatrix.getM()==4 && posMatrix.getN()==4);
  return posMatrix;
}


  /**
   * gives the position and rotation(angle) of the segment at the
   * end of the segment so that a new segment could be placed there
   */
Matrix DegreeSegment::getTransformedEndMatrix(){ // INTERNAL
  /*
  // this is the translation matrix, initialized with length
  Matrix t = getTranslationMatrix(Position(length,0,0));
  // this is the rotation matrix, initialized with angle,
  // in straightline the angle is zero!
  Matrix r = getRotationMatrix(angle);
  return (r * t);
  */

  //  Matrix p0 = getTranslationMatrix(Position(0.0f,0.0f,0.0f));
  // this is the translation matrix, initialized with length
  /*  Matrix t1 = getTranslationMatrix(Position(0.0f,-radius,0.0f));

  Matrix r = getRotationMatrix(-angle);

  Matrix t2 = getTranslationMatrix(Position(0.0f,radius,0.0f));

  Matrix t = t2 * (r * t1);*/
  Position e= getInnerCoordinates(radius,angle);
  Matrix end = getTranslationRotationMatrix(e,angle);
  // this was all, nice homogen coordinates :)
  return end;
}


/**
 * returns true if the real coordinates lay inside of the segment
 */
bool DegreeSegment::isInside(const Position& p) { // must be inner coordinates
  // the point lays inside of the segment if two conditions are true:
  // 1. the distance between (0,0,0) and the position is between
  //    0 and length
  // 2. the width between is between -width/2 and width/2
  if (getSectionIdValue(p)==-1 || getWidthIdValue(p)==-1)
    return 0;
  else return 1;
}

/**
 * returns a value between 0 and 100 that tells at which section
 * you are on the segment.
 * 0 means you are on the beginning
 * 100 means you are at the end
 * returns -1 if no IdValue can be given
 */
double DegreeSegment::getSectionIdValue(const Position& p) { // must be inner coordinates
  // first get a 4x1 matrix for p
  Matrix p0 = ::getPositionMatrix(p);
  Matrix t;
  // now get (0,-r,0) as a translation Matrix
  if (left==1)
    t = getTranslationMatrix(Position(0.0f,radius,0.0f));
  else
    t = getTranslationMatrix(Position(0.0f,-radius,0.0f));
  // now calculate the endmatrix
  Matrix p1 = t * p0;
  // now get the Position from p1
  Position pos1 = getPosition4x1(p1);
  // now calculate the angle
  double alpha;
  if (left==1)
    alpha = getAngle(pos1,Position(0.0f,-radius,0.0f));
  else
    alpha = getAngle(pos1,Position(0.0f,radius,0.0f));

  if ((alpha>=0.0f) && (alpha <=angle)) {
    // if check ok, return the value
    return (alpha/angle)*100.0f;
  }
  else return -1;
}

/**
 * returns a value between 0 and 100 that tells at which width
 * you are on the segment, more to right or more to the left.
 * 0 means you are on the left
 * 50 means you are in the middle
 * 100 means you are on the right
 * returns -1 if no WidthValue can be given
 */
double DegreeSegment::getWidthIdValue(const Position& p) { // must be inner coordinates
  // first get a 4x1 matrix for p
  Matrix p0 = ::getPositionMatrix(p);
  // now get (0,-r,0) as a translation Matrix
  Matrix t;
  if (left==1)
    t = getTranslationMatrix(Position(0.0f,-radius,0.0f));
  else
    t = getTranslationMatrix(Position(0.0f,radius,0.0f));
  // now calculate the endmatrix
  Matrix p1 = t * p0;
  // now get the Position from p1
  Position pos1 = getPosition4x1(p1);
  // now get the length
  double length = ::getLength(pos1)-radius;
  // now check if length is between -width/2 and width/2
  if (length>=-width/2.0f && length<=width/2.0f)
    return ((length+width/2.0f)/width)*100.0f;
  else return -1;
}





















  /**
   * draws the obstacle (4 boxes for the playground)
   */
  void DegreeSegment::draw(){
    dsSetTexture (DS_NONE);    
    dsSetColor (color.r, color.g, color.b);
    dVector3 dimensions;

    for(list<dGeomID>::iterator it = innerWalls.begin(); it!= innerWalls.end(); ++it) {
      // draws the wall
      dGeomBoxGetLengths ( (*it),  dimensions); // gets the length, width and height
      dsDrawBox ( dGeomGetPosition ( (*it) ) , dGeomGetRotation ( (*it) ) , dimensions );
    }
    dsSetColor (0.0f, 0.0f, 1.0f);
    for(list<dGeomID>::iterator it = outerWalls.begin(); it!= outerWalls.end(); ++it) {
      // draws the wall
      dGeomBoxGetLengths ( (*it),  dimensions); // gets the length, width and height
      dsDrawBox ( dGeomGetPosition ( (*it) ) , dGeomGetRotation ( (*it) ) , dimensions );
    }
 };

/**
 * returns the length of the segment,
 * here it is the length of the arc
 * formula is: radius * angle;
 */
double DegreeSegment::getLength() {
  if (angle>0)
    return (radius*angle);
  else
    return (-radius*angle);
}




void DegreeSegment::create(dSpaceID space)
{
  int numberCorners=16;
  for (int i=0;i<numberCorners;i++) {
    double lowerdivisor = ((float) i )/ ((float)numberCorners);
    double upperdivisor = ((float) i+1 )/ ((float)numberCorners);
    // get the first endpoint (beginning point)
    Position p1 = getOuterCoordinates(radius+width/2.0f,angle*lowerdivisor);
    // get the ending point
    Position p2 = getOuterCoordinates(radius+width/2.0f,angle*upperdivisor);
    // the length of the wall
    Position pdiff = getDifferencePosition(p2,p1);
    double length = ::getLength(getDifferencePosition(p1,p2));
    // now calculate the difference between the walls, because the length
    // of the box is in the middle and we must use the outer length
    double lengthdiff = sqrt((1-cos(angle*1.0f/numberCorners))*2.0f)*widthWall/2.0f;
    // now lets create one wall in space with the appropiate dimensions
    dGeomID innerWall = dCreateBox ( space,length +lengthdiff,widthWall,heightWall);
    // get the middle position of the two endpoints
     Position pm = ::getMiddlePosition(p1,p2);
    // now lets set the middle point of the wall set to mp
    dGeomSetPosition ( innerWall, pm.x,pm.y,pm.z+heightWall/2.0f);
    // now lets calculate the alpha between (0,r,0) and the vector p1-->p2
    double alpha=getAngle(pdiff,Position(1.0f,0.0f,0.0f));
    if (pdiff.y>0)
      alpha=-alpha;
    if (angle*upperdivisor>M_PI)
      alpha=-alpha;
    //    alpha = M_PI/4.0f;
    // now we must rotate the wall with alpha
    dMatrix3 Rl;
    dRFromEulerAngles(Rl, 0,0, alpha);
    dGeomSetRotation(innerWall, Rl);
    // now add him to the wall list
    innerWalls+=innerWall;
  }
  for (int i=0;i<numberCorners;i++) {
    double lowerdivisor = ((float) i )/ ((float)numberCorners);
    double upperdivisor = ((float) i+1 )/ ((float)numberCorners);
    // get the first endpoint (beginning point)
    Position p1 = getOuterCoordinates(radius-width/2.0f,angle*lowerdivisor);
    // get the ending point
    Position p2 = getOuterCoordinates(radius-width/2.0f,angle*upperdivisor);
    // the length of the wall
    Position pdiff = getDifferencePosition(p2,p1);
   double length = ::getLength(getDifferencePosition(p1,p2));
    // now calculate the difference between the walls, because the length
    // of the box is in the middle and we must use the outer length
    double lengthdiff = sqrt((1-cos(angle*1.0f/numberCorners))*2.0f)*widthWall/2.0f;
    // now lets create one wall in space with the appropiate dimensions
    dGeomID outerWall = dCreateBox ( space,length+lengthdiff,widthWall,heightWall);
    // get the middle position of the two endpoints
    Position pm = ::getMiddlePosition(p1,p2);
    // now lets set the middle point of the wall set to mp
    dGeomSetPosition ( outerWall, pm.x,pm.y,pm.z+heightWall/2.0f);
    // now lets calculate the alpha between (0,r,0) and the vector p1-->p2
    double alpha = getAngle(pdiff,Position(1.0f,0.0f,0.0f));
    if (angle*upperdivisor>M_PI)
      alpha=-alpha;
    if (pdiff.y>0)
      alpha=-alpha;
    // now we must rotate the wall with alpha
    dMatrix3 Rl;
    dRFromEulerAngles(Rl, 0,0, alpha);
    dGeomSetRotation(outerWall, Rl);
    // now add him to the wall list
    outerWalls+=outerWall;
  }
  obstacle_exists=true;
};


  void DegreeSegment::destroy(){
    //    dGeomDestroy(wallLeft);
    //dGeomDestroy(wallRight);
    obstacle_exists=false;
  };
