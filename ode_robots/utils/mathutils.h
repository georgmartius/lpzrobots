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
 *   Revision 1.6  2005-12-12 13:45:32  martius
 *   special inverse for 4x4 matrices in Pose form (can have diagonal zeros)
 *
 *   Revision 1.5  2005/11/15 13:38:27  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2005/11/15 14:26:32  robot3
 *   some new useful functions added
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

#include <matrix.h>
using namespace matrix;
#include "position.h"

#ifndef MIN_MAX_AND_SO_ON
#define MIN_MAX_AND_SO_ON
#define min(a,b) ( (a) < (b) ? a : b )
#define max(a,b) ( (a) > (b) ? a : b )
#define sign(x)  ( (x) < 0 ? -1 : 1 )
#define sqr(x)   ( (x)*(x) )
#define clip(x, lobound, highbound) ( (x)<(lobound) ? (lobound) : ( (x) > (highbound) ? (highbound) : (x) ) )
#endif

/*
 * returns the difference vector of two positions as a Position
 */
Position getDifferencePosition(Position p, Position q);


/**
 * returns a translation and rotation matrix with the given Position and angle
 */
Matrix getTranslationRotationMatrix(const Position& p, double angle);



/**
 * returns the middle point of the two given positions
 */
Position getMiddlePosition(Position& p, Position& q);

/**
 * returns a rotation matrix with the given angle
 */
Matrix getRotationMatrix(const double& angle);


/**
 * returns a translation (4x4) matrix with the given Position
 */
Matrix getTranslationMatrix(const Position& p) ;


/**
 * returns a position (4x1) matrix with the given Position
 */
Matrix getPositionMatrix(const Position& p);

Position getPosition4x1(const Matrix& position4x1);

/**
 * removes the translation in the matrix
 */
Matrix removeTranslationInMatrix(const Matrix& pose);


/**
 * removes the rotation in the matrix
 */
Matrix removeRotationInMatrix(const Matrix& pose) ;


/**
 * returns the angle between two vectors
 */
double getAngle(Position a, Position b) ;

/**
 * returns the Position stored in the pose
 */
Position getPosition(const Matrix& pose);

/**
 * returns the angle stored in the Matrix
 */
double getAngle(Matrix& pose);

/*  Matrix inversion technique:
Given a matrix mat, we want to invert it.
mat = [ r00 r01 r02 tx
        r10 r11 r12 ty
        r20 r21 r22 tz
        0  0  0  d ]
We note that this matrix can be split into two matrices.
mat = rot * trans, where rot is rotation part, trans is translation part
rot = [ r00 r01 r02 0
        r10 r11 r12 0
        r20 r21 r22 0
        0   0   0   1 ]
trans = [ 1 0 0 tx
         0 1 0 ty
         0 0 1 tz
         0 0 0 d ]
So the inverse is mat^-1 = (trans^-1 * rot^-1) 
******************************************/
Matrix invert_4x4PoseMatrix(const Matrix& m);

/**
 * returns the length of a vector stored as Position
 */
double getLength(const Position& p);

#endif
