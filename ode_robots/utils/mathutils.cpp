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
 *   Revision 1.4  2005-12-12 13:45:32  martius
 *   special inverse for 4x4 matrices in Pose form (can have diagonal zeros)
 *
 *   Revision 1.3  2005/11/15 14:26:32  robot3
 *   some new useful functions added
 *
 *   Revision 1.2  2005/10/27 14:16:11  martius
 *   some bugs fixed, module now works
 *   some functions from controller_misc.h are here now
 *
 *   Revision 1.1  2005/10/27 12:15:22  robot3
 *   several useful functions that provide mathematic operations
 *
 *                                                                         *
 ***************************************************************************/
#include <assert.h>
#include "mathutils.h"
#include <math.h>

/**
 * returns a rotation matrix with the given angle
 */
Matrix getRotationMatrix(const double& angle) {
  double data[16]={cos(angle),-sin(angle),0,0,
		   sin(angle),cos(angle),0,0,
		   0,0,1,0,
		   0,0,0,1};
 return Matrix(4,4,data);
}

/**
 * returns a rotation 3x3 rotation matrix part of the 4x4 pose matrix
 */
Matrix get3x3RotationMatrix(const Matrix& m) {
  double data[9]={m.val(0,0), m.val(0,1), m.val(0,2), 
		  m.val(1,0), m.val(1,1), m.val(1,2), 
		  m.val(2,0), m.val(2,1), m.val(2,2)};
 return Matrix(3,3,data);
}

/**
 * returns a rotation 4x4 pose  matrix from a 3x3 rotation matrix
 */
Matrix get4x4RotationMatrix(const Matrix& m) {
  double data[16]={m.val(0,0), m.val(0,1), m.val(0,2), 0, 
		   m.val(1,0), m.val(1,1), m.val(1,2), 0,
		   m.val(2,0), m.val(2,1), m.val(2,2), 0,
		   0 ,         0 ,         0 ,         1};
 return Matrix(4,4,data);
}

/**
 * returns a translation matrix with the given Position
 */
Matrix getTranslationMatrix(const Position& p) {
  double data[16]={1,0,0,p.x,
		   0,1,0,p.y,
		   0,0,1,p.z,
		   0,0,0,1};
 return Matrix(4,4,data);
}


/**
 * returns a translation and rotation matrix with the given Position and angle
 */
Matrix getTranslationRotationMatrix(const Position& p, double angle) {
  double data[16]={cos(angle),-sin(angle),0,p.x,
		   sin(angle),cos(angle),0,p.y,
		   0,0,1,p.z,
		   0,0,0,1};
 return Matrix(4,4,data);
}

/*
 * returns the difference vector of two positions as a Position
 */
Position getDifferencePosition(Position p, Position q) {
  return Position(p.x-q.x,p.y-q.y,p.z-q.z);
}

/**
 * removes the translation in the matrix
 */
Matrix removeTranslationInMatrix(const Matrix& pose){
  Matrix t(pose);
  // remove the three last values of the column 3
  t.val(0,3)=0.0f;
  t.val(1,3)=0.0f;
  t.val(2,3)=0.0f;
  return t;
}


Matrix getPositionMatrix(const Position& p) {
  double data[4]={p.x,p.y,p.z,1};
  return Matrix(4,1,data);
}


/**
 * removes the rotation in the matrix
 */
Matrix removeRotationInMatrix(const Matrix& pose){
  Matrix t(pose);
  t.val(0,0)=1.0f; t.val(0,1)=0.0f;   t.val(0,2)=0.0f;
  t.val(1,0)=0.0f; t.val(1,1)=1.0f;   t.val(1,2)=0.0f;
  t.val(2,0)=0.0f; t.val(2,1)=0.0f;   t.val(2,2)=1.0f;
  return t;
}

/**
 * returns the angle between two vectors
 */
double getAngle(Position a, Position b) {
  Matrix p(1,3,a.toArray()); // row wise
  Matrix q(3,1,b.toArray()); // column wise
  return  acos((p * q).val(0,0) / (a.length()*b.length()) );
}

/**
 * returns the angle stored in the Matrix
 */
double getAngle(Matrix& pose) {
  double acosinus = acos(pose.val(0,0));
  double asinus = asin(pose.val(1,0));
  std::cout << "asinus = " << asinus*180.0f/3.141536535965f << ", acosinus = " << acosinus*180.0f/3.141536535965f << "\n";
  if (asinus>0.0f)
    return acosinus;
  else return (2.0f*M_PI-acosinus); 
}


/**
 * returns the middle point of the two given positions
 */
Position getMiddlePosition(Position& p, Position& q) {
  return Position((p.x+q.x)/2.0f,(p.y+q.y)/2.0f,(p.z+q.z)/2.0f);
}


/**
 * returns the Position stored in the pose
 */
Position getPosition(const Matrix& pose) {
  assert(pose.getM()==4 && pose.getN()==4);
  return Position(pose.val(0,3),pose.val(1,3),pose.val(2,3));
}

/**
 * returns the Position stored in the 4x1-matrix
 */
Position getPosition4x1(const Matrix& pose) {
  return Position(pose.val(0,0),pose.val(1,0),pose.val(2,0));
}


/**
 * returns the length of a vector stored as Position
 */
double getLength(const Position& p) {
  return sqrt(sqr(p.x)+sqr(p.y)+sqr(p.z));
}

/**
 * returns inverse of a 4x4 Pose Matrix, which is of the following form:
 *
 */
Matrix invert_4x4PoseMatrix(const Matrix& m){
  Matrix rot = get3x3RotationMatrix(m);
  Matrix trans = removeRotationInMatrix(m);
  return get4x4RotationMatrix(rot^-1) * (trans^-1);
}
