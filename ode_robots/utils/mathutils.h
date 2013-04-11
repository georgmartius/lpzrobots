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
#ifndef __MATHUTILS_H
#define __MATHUTILS_H

#include <selforg/matrix.h>
#include <selforg/position.h>
#include "osgforwarddecl.h"
#include <osg/Math>

namespace lpzrobots {

  class Axis;

/*   template<typename T> */
/*   inline T clip(T v,T minimum, T maximum) */
/*     { return clampBelow(clampAbove(v,minimum),maximum); } */

/*template<typename T> */
/*        inline T abs(T v) */
/*{ return ((v>0)?v:-v); } */

  template<typename T>
  inline T normalize360(T v)
    { while (v>360) v-=360; while (v<360) v+=360; return v; }
  /*******************************************************************************/


  /**
   * returns a rotation matrix (osg) with the given angles
   * alpha, beta and gamma
   */
  osg::Matrix osgRotate(const double& alpha, const double& beta, const double& gamma);

  /**
     converts osg matrix to matrix of matrixlib
   */
  matrix::Matrix osgMatrix2Matrixlib(const osg::Matrix& m);


  /**
     returns a Rotation matrix that rotates the x-axis along with the given axis.
     The other 2 axis (y,z) are ambiguous.
  */
  osg::Matrix rotationMatrixFromAxisX(const Axis& axis);

  /**
     returns a Rotation matrix that rotates the z-axis along with the given axis.
     The other 2 axis (x,y) are ambiguous.
  */
  osg::Matrix rotationMatrixFromAxisZ(const Axis& axis);

  /**
   * returns the angle between two vectors (in rad)
   */
  double getAngle(const osg::Vec3& a, const osg::Vec3& b) ;

  /// converts an ode rotation matrix into a 3x3 rotation matrix (matrixlib)
  matrix::Matrix odeRto3x3RotationMatrixT ( const double R[12] );

  /// converts an ode rotation matrix into a 3x3 rotation matrix (matrixlib)
  matrix::Matrix odeRto3x3RotationMatrix ( const double R[12] );

  /*******************************************************************************/

  /**
     Multiplies 3x3 matrix with position
  */
  Position multMatrixPosition(const matrix::Matrix& r, Position& p);

  /**
   * returns a rotation matrix with the given angle
   */
  matrix::Matrix getRotationMatrix(const double& angle);


  /**
   * returns a translation matrix with the given Position
   */
  matrix::Matrix getTranslationMatrix(const Position& p) ;


  /**
   * removes the translation in the matrix
   */
  matrix::Matrix removeTranslationInMatrix(const matrix::Matrix& pose);


  /**
   * removes the rotation in the matrix
   */
  matrix::Matrix removeRotationInMatrix(const matrix::Matrix& pose) ;


  /**
   * returns the angle between two vectors
   */
  double getAngle(Position a, Position b) ;

}

#endif
