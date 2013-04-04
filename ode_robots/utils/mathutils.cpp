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
#include <cmath>
#include <osg/Matrix>
#include "mathutils.h"
#include "axis.h"

using namespace matrix;

namespace lpzrobots {



  /*
   * returns a rotation matrix (osg) with the given angles
   * alpha, beta and gamma
   */
  osg::Matrix osgRotate(const double& alpha, const double& beta, const double& gamma) {
    return (osg::Matrix::rotate(alpha,1,0,0)
            *osg::Matrix::rotate(beta,0,1,0)
            *osg::Matrix::rotate(gamma,0,0,1));
  }

  /*
     converts osg matrix to matrix of matrixlib
   */
  Matrix osgMatrix2Matrixlib(const osg::Matrix& m){
    Matrix m2(4,4);
    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        m2.val(i,j) = m(i,j);
      }
    }
    return m2;
  }


  /*
     returns a Rotation matrix that rotates the x-axis along with the given axis.
     The other 2 axis (y,z) are ambiguous.
  */
  osg::Matrix rotationMatrixFromAxisX(const Axis& axis){
    return osg::Matrix::rotate(osg::Vec3(1,0,0), axis.vec3());
  }

  /*
     returns a Rotation matrix that rotates the z-axis along with the given axis.
     The other 2 axis (x,y) are ambiguous.
  */
  osg::Matrix rotationMatrixFromAxisZ(const Axis& axis){
    return osg::Matrix::rotate(osg::Vec3(0,0,1), axis.vec3());
  }


  /*
   * returns the angle between two vectors (in rad)
   */
  double getAngle(const osg::Vec3& a, const osg::Vec3& b) {
    // Cosinus Satz
    // here a*b is the dot product (Skalarprodukt)
    return acos(a*b / (a.length()*b.length()));
  }

  matrix::Matrix odeRto3x3RotationMatrixT ( const double R[12] ) {
    matrix::Matrix matrix(3,3);
    matrix.val(0,0)=R[0];
    matrix.val(0,1)=R[4];
    matrix.val(0,2)=R[8];
    matrix.val(1,0)=R[1];
    matrix.val(1,1)=R[5];
    matrix.val(1,2)=R[9];
    matrix.val(2,0)=R[2];
    matrix.val(2,1)=R[6];
    matrix.val(2,2)=R[10];
    return matrix;
  }

  matrix::Matrix odeRto3x3RotationMatrix ( const double R[12] ) {
    matrix::Matrix matrix(3,3);
    matrix.val(0,0)=R[0];
    matrix.val(1,0)=R[4];
    matrix.val(2,0)=R[8];
    matrix.val(0,1)=R[1];
    matrix.val(1,1)=R[5];
    matrix.val(2,1)=R[9];
    matrix.val(0,2)=R[2];
    matrix.val(1,2)=R[6];
    matrix.val(2,2)=R[10];
    return matrix;
  }



  /******************************************************************************/


  Position multMatrixPosition(const Matrix& r, Position& p){
    assert(r.getM()==3 && r.getN()==3);
    Matrix pm(3,1,p.toArray());
    Matrix rv = r*pm;
    return Position(rv.val(0,0), rv.val(1,0), rv.val(2,0));
  }

  /*
   * returns a rotation matrix for a rotation in x, y plane about the given angle
   */
  Matrix getRotationMatrix(const double& angle) {
    double data[16]={cos(angle),sin(angle),0,0,
                     -sin(angle),cos(angle),0,0,
                     0,0,1,0,
                     0,0,0,1};
    return Matrix(4,4,data);
  }

  /*
   * returns a translation matrix with the given Position
   */
  Matrix getTranslationMatrix(const Position& p) {
    double data[16]={0,0,0,p.x,
                     0,0,0,p.y,
                     0,0,1,p.z,
                     0,0,0,1};
    return Matrix(4,4,data);
  }

  /*
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

  /*
   * removes the rotation in the matrix
   */
  Matrix removeRotationInMatrix(const Matrix& pose){
    Matrix t(pose);
    t.val(0,0)=1.0f; t.val(0,1)=0.0f;
    t.val(1,0)=0.0f; t.val(1,1)=1.0f;
    return t;
  }

  /*
   * returns the angle between two vectors
   */
  double getAngle(Position a, Position b) {
    Matrix p(3,1,a.toArray()); // row wise
    Matrix q(1,3,b.toArray()); // column wise
    return acos((p * q).val(0,0) / (a.length()*b.length()) );
  }

}
