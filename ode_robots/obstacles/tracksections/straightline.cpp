/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.5  2005-12-11 12:06:35  robot3
 *   racegroundsensor testet
 *
 *   Revision 1.4  2005/12/03 16:57:12  martius
 *   setWidth is void
 *
 *   Revision 1.3  2005/11/22 15:51:23  robot3
 *   testing
 *
 *   Revision 1.2  2005/11/22 13:03:53  robot3
 *   bugfixing
 *
 *   Revision 1.1  2005/11/15 14:29:25  robot3
 *   first version
 *
 *                                                                         *
 ***************************************************************************/
#include "straightline.h"
#include "mathutils.h"


void StraightLine::setProperties() {
  length=10.0f; // INTERNAL length
  width=5.0f; // INTERNAL width
  widthWall=0.1f;
  heightWall=0.7f;
  angle=0.0f;
  color=Color(226 / 255.0, 103 / 255.0, 66 / 255.0);
}


/**
 * Constructor
 */
StraightLine::StraightLine() : posMatrix(4,4) { // GLOBAL
  // now position and rotation is all 0
  setProperties();
  posMatrix.toId();
};

/**
 * Constructor
 */
StraightLine::StraightLine(const Position& p,const double& angle) { // GLOBAL
  // 2 matrices are calculated, the sum is the matrix to store
  setProperties();
  posMatrix = getTranslationRotationMatrix(p,angle);
};

/**
 * Constructor
 */
StraightLine::StraightLine(const Matrix& pose) : posMatrix(pose) { // GOBAL
  setProperties();
  Position p = ::getPosition(pose);
  std::cout << "Pos of Line=(" << p.x << ", " << p.y << ", " << p.z << ")\n";
};


Position StraightLine::getPosition() {
  return ::getPosition(posMatrix);
}

  /** 
   * gives the position and rotation(angle) of the segment
   * in the real world.
   * returns a matrix.
   */
Matrix StraightLine::getPositionMatrix() { // GLOBAL
  return posMatrix;
};


  /** 
   * sets the position and rotation(angle) of the segment
   * in the real world.
   */
void StraightLine::setPositionMatrix(const Matrix& pose) { // GLOBAL
  posMatrix=pose;
};

  /** 
   * sets the position of the segment
   * in the real world.
   */
void StraightLine::setPosition(const Position& pos) { // GLOBAL
  // the new posMatrix is the old rotation plus the new translation part
  // therefore the old translation part must be removed first
  posMatrix=getTranslationRotationMatrix(pos,getAngle(posMatrix));
};

void StraightLine::setRotation(const double& angle) { // GLOBAL
  // the new posMatrix is the old position plus the new translation part
  // therefore the old rotation part must be removed first
  posMatrix=getTranslationRotationMatrix(::getPosition(posMatrix),angle);
}

void StraightLine::setRotation(const Matrix& rot) { // GLOBAL
  // the new posMatrix is the old position plus the new translation part
  // therefore the old rotation part must be removed first
  posMatrix=removeRotationInMatrix(posMatrix)*rot;
}

Matrix StraightLine::getRotation() { // GLOBAL
  return posMatrix;
}




  /**
   * gives the position and rotation(angle) of the segment at the
   * end of the segment so that a new segment could be placed there
   */
Matrix StraightLine::getTransformedEndMatrix(){ // INTERNAL
  // this is the translation matrix, initialized with length
  Matrix t = getTranslationMatrix(Position(length,0,0));
  // this is the rotation matrix, initialized with angle,
  // in straightline the angle is zero!
  Matrix r = getRotationMatrix(angle);
  return (r * t);
}

/**
 * returns true if the real coordinates lay inside of the segment
 */
bool StraightLine::isInside(const Position& p) { // must be inner coordinates0
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
double StraightLine::getSectionIdValue(const Position& p) { // must be inner coordinates
  // now the check
  if ((p.x>=0.0f) && (p.x<=length)) {
    // if check ok, return the value
    return p.x/length*100.0f;
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
double StraightLine::getWidthIdValue(const Position& p) { // must be inner coordinates
  double t= (p.y+width/2.0f)/width*100.0f;
  if ((t>=0.0f) && (t <=100.0f)) {
    // if check ok, return the value
    return t;
  }
  else return -1;
}

/**
 * returns the length of the segment,
 * here it is the length of the arc
 * formula is: radius * angle;
 */
double StraightLine::getLength() {
  return length;
}

/**
 * returns the width of the segment,
 */
double StraightLine::getWidth() {
  return width;
}

/**
 * sets the width of the segment,
 */
void StraightLine::setWidth(double w) {
  width=w;
}





  /**
   * draws the obstacle (4 boxes for the playground)
   */
  void StraightLine::draw(){
    dsSetTexture (DS_NONE);    
    
    dsSetColor (color.r, color.g, color.b);
    dVector3 dimensions;
    dGeomBoxGetLengths ( wallLeft,  dimensions); // gets the length, width and height
    dsDrawBox ( dGeomGetPosition ( wallLeft ) , dGeomGetRotation ( wallLeft ) , dimensions );
    
    dsSetColor (0.0f, 0.0f, 1.0f);
    dGeomBoxGetLengths ( wallRight,  dimensions); // gets the length, width and height
    dsDrawBox ( dGeomGetPosition ( wallRight ) , dGeomGetRotation ( wallRight ) , dimensions );
  };


void StraightLine::create(dSpaceID space)
{

  // create all walls bordering the segment to the left and right, this are only two

  // first calculate the points
  // wallLeft: position is:
  Matrix p = getPositionMatrix();
  Matrix pl = p * ::getTranslationMatrix(Position(length/2.0f,-width/2.0f,heightWall/2.0f));
  Matrix pr = p * ::getTranslationMatrix(Position(length/2.0f,width/2.0f,heightWall/2.0f));

  wallLeft = dCreateBox ( space,length,widthWall,heightWall);
  Position l = ::getPosition(pl);
  dGeomSetPosition ( wallLeft, l.x,l.y,l.z);

  wallRight = dCreateBox ( space,length,widthWall,heightWall);
  Position r = ::getPosition(pr);
  dGeomSetPosition ( wallRight, r.x, r.y, r.z);
  dMatrix3 Rl;
  dRFromEulerAngles(Rl, 0,0, -getAngle(p));
  std::cout << "angle from posMatrix = " << (getAngle(p)*180.0f/M_PI) << "\n";
  dGeomSetRotation(wallLeft, Rl);
  dGeomSetRotation(wallRight, Rl); // same angle
  
  obstacle_exists=true;
  };


  void StraightLine::destroy(){
    dGeomDestroy(wallLeft);
    dGeomDestroy(wallRight);
    obstacle_exists=false;
  };



