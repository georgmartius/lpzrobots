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
 *   Revision 1.1  2005-10-27 12:15:22  robot3
 *   several useful functions that provide mathematic operations
 *
 *                                                                         *
 ***************************************************************************/
#include "controller_misc.h"
#include <math.h>
#include "matrix.h"


/**
 * returns a rotation matrix with the given angle
 */
Matrix getRotationMatrix(const double& angle) const=0:


/**
 * returns a translation matrix with the given Position
 */
Matrix getTranslationMatrix(const Position& p) const=0;


/**
 * removes the translation in the matrix
 */
Matrix removeTranslationInMatrix(const Matrix& pose) const=0l


/**
 * removes the rotation in the matrix
 */
Matrix removeRotationInMatrix(const Matrix& pose) const=0;


/**
 * returns the angle between two vectors
 */
double getAngle(const Position& a,const Position& b) const=0;


/**
 * returns the length of a vector stored as Position
 */
double getLength(const Position& p) const=0;
