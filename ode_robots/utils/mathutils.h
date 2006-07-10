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
 *   Revision 1.3.4.10  2006-07-10 12:07:16  martius
 *   Matrixlib now in selforg
 *   optimised compilation
 *
 *   Revision 1.3.4.9  2006/06/27 14:14:30  robot3
 *   -optimized mesh and boundingshape code
 *   -other changes
 *
 *   Revision 1.3.4.8  2006/03/29 15:10:11  martius
 *   osgMatrix2matrixlib
 *
 *   Revision 1.3.4.7  2006/03/05 10:58:18  robot3
 *   added a template function normalize360
 *
 *   Revision 1.3.4.6  2006/03/04 16:57:36  robot3
 *   added a template function for abs
 *
 *   Revision 1.3.4.5  2006/02/07 15:48:56  martius
 *   axis
 *
 *   Revision 1.3.4.4  2005/12/15 17:04:32  martius
 *   getAngle
 *   min, max and so on are template functions now
 *
 *   Revision 1.3.4.3  2005/12/14 15:37:38  martius
 *   rotation matrix for axis
 *
 *   Revision 1.3.4.2  2005/11/24 16:21:45  fhesse
 *   multMatrixPosition added
 *
 *   Revision 1.3.4.1  2005/11/14 17:37:25  martius
 *   moved to selforg
 *
 *   Revision 1.3  2005/11/10 09:09:55  martius
 *   use defines for definitions of sqrt, min, max...
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
#ifndef __MATHUTILS_H
#define __MATHUTILS_H

#include <selforg/matrix.h>
#include <selforg/position.h>
#include "osgforwarddecl.h"
#include <osg/Math>

namespace lpzrobots {

  class Axis;

  template<typename T>
  inline T clip(T v,T minimum, T maximum)
    { return clampBelow(clampAbove(v,minimum),maximum); }

  template<typename T>
  inline T abs(T v)
    { return ((v>0)?v:-v); }

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
