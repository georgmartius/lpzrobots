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
#ifndef __CONTOLLER_MISC_H
#define __CONTOLLER_MISC_H

#include <selforg/matrix.h>
#include <assert.h>
#include <stdlib.h>
#include <cmath>

#include "stl_adds.h"
#include "noisegenerator.h"
#include "inspectable.h"


template<typename T>
inline T sign(T v) { return v<(T)0?(T)-1:(T)1; }

template<typename T>
inline T sqr(T v) { return v*v; }

template<typename T>
inline T clip(T x, T lobound, T highbound) {
  return (x)<(lobound) ? (lobound) : ( (x) > (highbound) ? (highbound) : (x) );
}


/// calculates 1/x
double one_over(double x);

/// returns c (useful for Matrix::mapP to fill matrix with constant value)
double constant(double c, double);

/// returns x the power c (as a double)
double power(void* c, double x);

/// power 3 for matrix::map
double power3(double x);

/// creates random number from -1 to 1
double random_minusone_to_one(double);

/// creates random number from -1 to 1
//  using the RandGen* which is give as a void pointer (Matrix::mapP)
double random_minusone_to_one(void *, double);

/// clipping function to the interval [-r, r] (for use with mapP)
double clip(double r,double);

/// cutof function for mapP
double lowercutof(void* theta, double);

/// returns -1 if probability is to low, otherwise 1 for mapP
double toBinaryWithProbability(void* r,double x);

/// returns -1 if below threshold, otherwise 1 for map2
double toBinaryWithThreshold(double x, double threshold);

/// plus function for mapP
double plus_(double b, double a);


/** stores at least left top 4x4 submatrix (row-wise) (if exists) and
   then the rest of the diagonal elements into a list
   @return list of values
*/
std::list<matrix::D> store4x4AndDiagonal(const matrix::Matrix& m);

/** stores at least left top 4x4 submatrix (row-wise) (if exists) and
   then the rest of the diagonal elements
  @param m matrix to be stored
  @param len Length of the provided buffer
  (should be min(getN(),4)*min(getM(),4)+ max(0,min(getM()-4,getN()-4)))
  @return number of actually written elements
*/
matrix::I store4x4AndDiagonal(const matrix::Matrix& m, matrix::D* buffer, matrix::I len);

/** returns the number of elements stored by store4x4AndDiagonal
  (should be min(getN(),4)*min(getM(),4)+ max(0,min(getM()-4,getN()-4)))
*/
matrix::I get4x4AndDiagonalSize(const matrix::Matrix& m);

/** writes the names of the fields stored by store4x4AndDiagonal into a list
  @param m matrix to be stored
  @param matrixName name of the matrix (prefix for all fields)
  @return list of keys
*/
std::list<Inspectable::iparamkey> store4x4AndDiagonalFieldNames(const matrix::Matrix& m,
                                                                const std::string& matrixName);

/** stores the names of the fields stored by store4x4AndDiagonal
  @param m matrix to be stored
  @param matrixName name of the matrix (prefix for all fields)
  @param keylist list for field names
  @param len Length of the provided buffer
  (should be min(getN(),4)*min(getM(),4)+ max(0,min(getM()-4,getN()-4)))
  @return number of actually written elements
*/
matrix::I store4x4AndDiagonalFieldNames(const matrix::Matrix& m,
                                        const std::string& matrixName,
                                        char** keylist, matrix::I len);

/** stores the names of all matrix fieldnames
  in the order produced by convertToBuffer (row-wise)
  @return list of names
*/
std::list<Inspectable::iparamkey> storeMatrixFieldNames(const matrix::Matrix& m, const std::string& matrixName);

/** stores the names of all vector (mx1 or 1xn matrix) fieldnames
  in the order produced by convertToBuffer (row-wise)
  @return list of names
*/
std::list<Inspectable::iparamkey> storeVectorFieldNames(const matrix::Matrix& m, const std::string& vectorName);

/** stores the names of all matrix fieldnames
   in the order produced by convertToBuffer (row-wise)
  @param m matrix to be stored
  @param matrixName name of the matrix (prefix for all fields)
  @param keylist list for field names
  @param len Length of the provided buffer
  (should be getN()*getM()
  @return number of actually written elements
*/
matrix::I storeMatrixFieldNames(const matrix::Matrix& m, const char* matrixName,
                        char** keylist, matrix::I len);

/** stores the names of all vector (mx1 or 1xn matrix) fieldnames
  in the order produced by convertToBuffer (row-wise)
  @param m vector to be stored
  @param vectorName name of the vector (prefix for all fields)
  @param keylist list for field names
  @param len Length of the provided buffer (should be getM())
  @return number of actually written elements
*/
matrix::I storeVectorFieldNames(const matrix::Matrix& m, const char* vectorName,
                                char** keylist, matrix::I len);

/** stores the Matrix into the given file stream (binary)
 */
bool storeMatrix(const matrix::Matrix& m, FILE* f);

/** reads a Matrix from the given file stream (binary)
 */
bool restoreMatrix(matrix::Matrix& m, FILE* f);

/// returns a Matrix with values generated by the given noise generator
matrix::Matrix noiseMatrix(matrix::I m, matrix::I n, NoiseGenerator& ng,
                           double strength,double unused = 0);

/** geneterates deterministically a new random generator from the given one.
    If null then rand() is used for the seed;
*/
RandGen* splitRandGen(RandGen* randGen);

/** calculates to linear matrix norm (sum of absolute values divided by number of values (m*n) )
*/
double matrixNorm1(const matrix::Matrix& m);

/** calculates to square matrix norm (sum of square values divided by number of values (m*n) )
*/
double matrixNorm2(const matrix::Matrix& m);

/** Matrix/matrixNorm2 ) */
matrix::Matrix matrixNormalized(const matrix::Matrix& m);


/** returns the k. largest element of the matrix
    Attention: it will detroy the given matrix! (sorting)
    Assumption: k>0 and k<=matrixsize
*/
double getKthLargestElement(matrix::Matrix& matrix, matrix::I k);

/** returns the k. smallest element of the matrix
    Attention: it will detroy the given matrix! (sorting)
    Assumption: k>0 and k<=matrixsize
*/
double getKthSmallestElement(matrix::Matrix& matrix, matrix::I k);

/// considers the matrix as vector (mx1) and returns the index of the smallest element
matrix::I argmin(const matrix::Matrix& v);

/// considers the matrix as vector (mx1) and returns the index of the largest element
matrix::I argmax(const matrix::Matrix& v);

/// returns the smallest element
double min(const matrix::Matrix& v);

/// minimum function for doubles without templates
double min(double a, double b);

/// returns the largest element
double max(const matrix::Matrix& v);

/// maximum function for doubles without templates
double max(double a, double b);


/// samples from the pdf (rowwise stored with sum = 1)
matrix::I sample(const matrix::Matrix& pdf);


/// parameter adaptation algorithm.
//   @param p current parameter value
//   @param actual actual value of some size controlled by p
//   @param nominal nominal value of some size controlled by p
//   @param up_rate adaptation rate for increasing p (<< 1)
//   @param down_rate adaptation rate for decreasing p (<< 1)
//   @return new value of p (there is no clipping done)
double adapt(double p, double actual, double nominal, double up_rate, double down_rate);

/// like adapt but that the adaption is just done if the actual value is outside the given interval (_min, _max)
double adaptMinMax(double p, double actual, double _min, double _max, double up_rate, double down_rate);

        /**
         * Helper function for converting an array with double values to a list.
         * Is used for the method getInternalParams() interface inspectable.
         */
std::list<Inspectable::iparamval> convertArrayToList(double* array,int arraySize);
        /**
         * Helper function for getting the array names of an array
         * Is used for the method getInternalParamNames() interface inspectable.
         */
std::list<Inspectable::iparamkey> getArrayNames(int arraySize,const char* name);

#endif
