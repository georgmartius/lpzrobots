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
StraightLine::StraightLine(const Position& p,const double& angle)
  : AbstractTrackSection(p,angle) {
  // 2 matrices are calculated, the sum is the matrix to store
  setProperties();
};

/**
 * Constructor
 */
StraightLine::StraightLine(const Matrix& pose)
  : AbstractTrackSection(pose) { 
  setProperties();
};

  /**
   * gives the position and rotation(angle) of the segment at the
   * end of the segment so that a new segment could be placed there
   */
Matrix StraightLine::getTransformedEndMatrix(){ // INTERNAL
  // this is the translation matrix, initialized with length
  return  getTranslationMatrix(Position(length,0,0));
}

/**
 * returns true if the real coordinates lay inside of the segment
 */
bool StraightLine::isInside(const Position& p) { // must be inner coordinates0
  // the point lays inside of the segment if two conditions are true:
  // 1. the x - coordinate of the position is between 0 and length
  // 2. the width (y) is is between -width/2 and width/2
  if (getSectionIdValue(p)==-1 || getWidthIdValue(p)==-1)
    return 0;
  else return 1;
}

/**
 * returns a value between 0 and length that tells at which section
 * you are on the segment.
 * returns -1 if no IdValue can be given
 */
double StraightLine::getSectionIdValue(const Position& p) { // must be inner coordinates
  Position local = transformToLocalCoord(p);
  // now the check
  if ((local.x>=0.0f) && (local.x<=length)) {
    // if check ok, return the value
    return local.x; 
  }
  else return -1;
}

/**
 * returns a value between 0 and width that tells at which width
 * you are on the segment, more to right or more to the left.
 * returns -1 if no WidthValue can be given
 */
double StraightLine::getWidthIdValue(const Position& p) { // must be inner coordinates
  Position local = transformToLocalCoord(p);
  if ((local.y >= - width/2.0f) && (local.y <=width/2.0f)) {
    // if check ok, return the value
    return local.y + width/2.0;
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
  Matrix p = getPoseMatrix();
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



