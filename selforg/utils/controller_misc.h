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
 *   Revision 1.1  2008-04-16 12:44:16  martius
 *   moved to utils
 *
 *   Revision 1.24  2008/03/01 01:47:34  martius
 *   plus_ and lowercutof
 *
 *   Revision 1.23  2007/12/13 16:43:24  martius
 *   noiseMatrix adapted
 *
 *   Revision 1.22  2007/12/11 14:43:27  martius
 *   clipping for matrix mapP
 *
 *   Revision 1.21  2007/08/24 12:02:22  martius
 *   getKSmallestElement
 *
 *   Revision 1.20  2007/07/16 08:51:57  martius
 *   argmax
 *
 *   Revision 1.19  2007/06/21 16:27:43  martius
 *   min and max for matrices
 *
 *   Revision 1.18  2007/06/08 15:44:57  martius
 *   constant function
 *
 *   Revision 1.17  2007/05/22 16:01:45  martius
 *   argmin
 *
 *   Revision 1.16  2007/03/22 08:07:47  robot3
 *   added two methods for generating names and a list from arrays for the interface
 *   inspectable: getInternalParams() and getInternalParamNames()
 *
 *   Revision 1.15  2007/03/22 08:05:03  robot3
 *   this is an adapter class which implements all needed things. This class is
 *   used for example by the DiscreteControllerAdapter.
 *
 *   Revision 1.14  2007/02/20 15:41:06  martius
 *   big model stuff, elman and co
 *
 *   Revision 1.13  2007/02/02 11:16:18  martius
 *   getKthLargestElementc
 *
 *   Revision 1.12  2006/07/20 17:14:34  martius
 *   removed std namespace from matrix.h
 *   storable interface
 *   abstract model and invertablemodel as superclasses for networks
 *
 *   Revision 1.11  2006/07/14 12:23:57  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.9.6.4  2006/07/10 11:59:23  martius
 *   Matrixlib now in selforg
 *   no namespace std in header files
 *
 *   Revision 1.9.6.3  2006/01/17 16:58:39  martius
 *   loading and storing
 *
 *   Revision 1.9.6.2  2005/12/15 17:04:39  martius
 *   min, max and so on are template functions now
 *
 *   Revision 1.9.6.1  2005/11/15 12:30:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.9  2005/11/10 09:13:33  martius
 *   readded defines like min, max sqr,...
 *
 *   Revision 1.8  2005/10/27 14:17:17  martius
 *   some functions are now found in mathutils.h (dir: ../ode_robots/utils)
 *
 *   Revision 1.7  2005/10/21 11:50:33  martius
 *   adapt functions
 *   random number function for usage with Matrix::map
 *
 *   Revision 1.6  2005/10/06 17:06:57  martius
 *   switched to stl lists
 *
 *   Revision 1.5  2005/09/22 10:33:07  martius
 *   max and min more save
 *
 *   Revision 1.4  2005/08/29 09:05:31  fhesse
 *   removed weird bug in get4x4AndDiagonalSize, probably caused by some compiler optimization
 *
 *   Revision 1.3  2005/08/06 20:47:54  martius
 *   Commented
 *
 *   Revision 1.2  2005/07/29 14:26:56  martius
 *   fixed bug in length calculation of 4x4 diagonal routines
 *
 *   Revision 1.1  2005/07/28 11:14:52  martius
 *   helper functions for controller
 *    support for internal parameters more easy
 *
 *
 ***************************************************************************/
#ifndef __CONTOLLER_MISC_H
#define __CONTOLLER_MISC_H

#include <selforg/matrix.h>
#include <assert.h>
#include <stdlib.h>
#include <math.h>

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

/// returns c as a double (useful for Matrix::mapP to fill matrix with constant value)
double constant(void* c, double);

/// creates random number from -1 to 1
double random_minusone_to_one(double);

/// clipping function for mapP
double clip(void* r,double);

/// cutof function for mapP
double lowercutof(void* theta, double);

/// plus function for mapP
double plus_(void* b, double a);


/* stores at least left top 4x4 submatrix (row-wise) (if exists) and 
   then the rest of the diagonal elements into a list
   @return list of values
*/
std::list<matrix::D> store4x4AndDiagonal(const matrix::Matrix& m);

/* stores at least left top 4x4 submatrix (row-wise) (if exists) and 
   then the rest of the diagonal elements
  @param len Length of the provided buffer 
  (should be min(getN(),4)*min(getM(),4)+ max(0,min(getM()-4,getN()-4))) 
  @return number of actually written elements
*/
unsigned int store4x4AndDiagonal(const matrix::Matrix& m, matrix::D* buffer, unsigned int len);

/* returns the number of elements stored by store4x4AndDiagonal
  (should be min(getN(),4)*min(getM(),4)+ max(0,min(getM()-4,getN()-4))) 
*/
unsigned int get4x4AndDiagonalSize(const matrix::Matrix& m);

/* writes the names of the fields stored by store4x4AndDiagonal into a list
  @param matrixName name of the matrix (prefix for all fields)
  @param keylist list for field names
  @param len Length of the provided buffer 
  (should be min(getN(),4)*min(getM(),4)+ max(0,min(getM()-4,getN()-4))) 
  @return number of actually written elements
*/
std::list<Inspectable::iparamkey> store4x4AndDiagonalFieldNames(const matrix::Matrix& m, 
								const std::string& matrixName);

/* stores the names of the fields stored by store4x4AndDiagonal 
  @param matrixName name of the matrix (prefix for all fields)
  @param keylist list for field names
  @param len Length of the provided buffer 
  (should be min(getN(),4)*min(getM(),4)+ max(0,min(getM()-4,getN()-4))) 
  @return number of actually written elements
*/
unsigned int store4x4AndDiagonalFieldNames(const matrix::Matrix& m, const std::string& matrixName, 
					   char** keylist, unsigned int len);

/* stores the names of the all matrix fieldnames produces by convertToBuffer into a list
  @return list of names
*/
std::list<Inspectable::iparamkey> storeMatrixFieldNames(const matrix::Matrix& m, const std::string& matrixName);

/* stores the names of the all vector (mx1 matrix) fieldnames  produces by convertToBuffer into a list
  @return list of names
*/
std::list<Inspectable::iparamkey> storeVectorFieldNames(const matrix::Matrix& m, const std::string& vectorName);

/* stores the names of the all matrix fieldnames produces by convertToBuffer
  @param matrixName name of the matrix (prefix for all fields)
  @param keylist list for field names
  @param len Length of the provided buffer 
  (should be getN()*getM()
  @return number of actually written elements
*/
unsigned int storeMatrixFieldNames(const matrix::Matrix& m, const char* matrixName, 
				   char** keylist, unsigned int len);

/* stores the names of the all vector (mx1 matrix) fieldnames produces by convertToBuffer
  @param vectorName name of the vector (prefix for all fields)
  @param keylist list for field names
  @param len Length of the provided buffer (should be getM())
  @return number of actually written elements
*/
unsigned int storeVectorFieldNames(const matrix::Matrix& m, const char* vectorName, 
				   char** keylist, unsigned int len);

/** stores the Matrix into the given file stream (binary)
 */
bool storeMatrix(const matrix::Matrix& m, FILE* f);

/** reads a Matrix from the given file stream (binary)
 */
bool restoreMatrix(matrix::Matrix& m, FILE* f);

/// returns a Matrix with values generated by the given noise generator
matrix::Matrix noiseMatrix(unsigned int m, unsigned int n, NoiseGenerator& ng, 
			   double strength,double unused = 0);

/** calculates to linear matrix norm (sum of absolute values divided by number of values (m*n) )
*/
double matrixNorm1(const matrix::Matrix& m);

/** calculates to square matrix norm (sum of square values divided by number of values (m*n) )
*/
double matrixNorm2(const matrix::Matrix& m);


/** returns the k. largest element of the matrix 
    Attention: it will detroy the given matrix! (sorting)
    Assumption: k>0 and k<=matrixsize
*/
double getKthLargestElement(matrix::Matrix& matrix, unsigned int k); 

/** returns the k. smallest element of the matrix 
    Attention: it will detroy the given matrix! (sorting)
    Assumption: k>0 and k<=matrixsize
*/
double getKthSmallestElement(matrix::Matrix& matrix, unsigned int k); 

/// considers the matrix as vector (mx1) and returns the index of the smallest element
int argmin(const matrix::Matrix& v);

/// considers the matrix as vector (mx1) and returns the index of the largest element
int argmax(const matrix::Matrix& v);

/// returns the smallest element
double min(const matrix::Matrix& v);

/// returns the largest element
double max(const matrix::Matrix& v);

/// samples from the pdf (rowwise stored with sum = 1)
int sample(const matrix::Matrix& pdf);


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
