/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Rald Der       <ralfder at mis dot mpg dot de>                       *
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
#ifndef MATRICES_H
#define MATRICES_H

#include <vector>
#include "vector.h"
#include "exceptions.h"

namespace lpzrobots {


template <typename T> class TriDiagonalMatrix;

template <typename T>
class TriDiagonalMatrixPair {
 public:
  TriDiagonalMatrixPair(const TriDiagonalMatrix<T> &r_first,
			const TriDiagonalMatrix<T> &r_second);

  TriDiagonalMatrix<T> first;
  TriDiagonalMatrix<T> second;
};


template<typename T>
class AbstractMatrix {
 protected:
 public:
};


template <typename T>
class Matrix : public AbstractMatrix<T> {
 public:
  typedef std::vector<T> Data;

 protected:
  Data data;
  unsigned n;
  unsigned m;

 public:
  Matrix(unsigned n = 0, unsigned m = 0);

  unsigned get_column_count() const;
  unsigned get_row_count() const;
  
  T& operator() (unsigned i, unsigned j);
  const T& operator() (unsigned i, unsigned j) const;
};



/**
 * TriDiagonalMatrix
 * N: number of rows
 *
 */

template<typename T>
class TriDiagonalMatrix : public AbstractMatrix<T> {
 public:
  // using class AbstractMatrix<T>;
  typedef std::vector<T> Data;

 protected:
  Data data;

 public:
  TriDiagonalMatrix(unsigned dimension);

  unsigned get_dimension() const;
  

  T& operator() (unsigned i, unsigned j);
  const T& operator() (unsigned i, unsigned j) const;

  TriDiagonalMatrixPair<T> create_lu_decomposition() const;
};





template <typename T>
Vector<T> solve(const TriDiagonalMatrix<T> &r_mat, const Vector<T> &r_vector);


/*****************************************************************************/
/* IMPLEMENTATIONS                                                           */
/*****************************************************************************/



/*****************************************************************************/
/* Matrix                                                                    */
/*****************************************************************************/
template <typename T>
Matrix<T>::Matrix(unsigned _n, unsigned _m) :
  data(_n * _m),
  n(_n),
  m(_m)
{
}


template <typename T>
unsigned Matrix<T>::get_row_count() const
{
  return n;  
}


template <typename T>
unsigned Matrix<T>::get_column_count() const
{
  return m;
}


template <typename T>
T& Matrix<T>::operator() (unsigned i, unsigned j)
{
  if(i >= n || j >= m)
    IndexOutOfBoundsException().raise();

  return data[i * m + j];
}


template <typename T>
const T& Matrix<T>::operator() (unsigned i, unsigned j) const
{
  if(i >= n || j >= m)
    IndexOutOfBoundsException().raise();

  return data[i * m + j];
}

/*****************************************************************************/
/* TriDiagonalMatrix                                                         */
/*****************************************************************************/
// .) solve algorithm base on 
//    http://www.d-fine.de/pool/bibliothek/vl_jra_stoch_6.pdf
//    unfortunately the formulas for the result vector are wrong >_<

template <typename T>
TriDiagonalMatrixPair<T>::TriDiagonalMatrixPair(const TriDiagonalMatrix<T> &r_first, const TriDiagonalMatrix<T> &r_second) :
  first(r_first),
  second(r_second)
{
}


template <typename T>
TriDiagonalMatrix<T>::TriDiagonalMatrix(unsigned dimension) :
  data(3 * dimension - 2)
{
  // each row has at most 3 non-zero elements
  // the first and last row only have 2 non-zero elments
}


template <typename T>
unsigned TriDiagonalMatrix<T>::get_dimension() const
{
  // since: size = 3 * dimension - 2
  //        dimension = (size + 2) / 3
  return (data.size() + 2) / 3;
}


template <typename T>
T& TriDiagonalMatrix<T>::operator() (unsigned i, unsigned j)
{
  static T zero = 0;;

  // formula for addressing an element: adr = 2 * i + j
  // where |i - j| must be less equal 1

  // make sure i and j are in bounds
  if(i >= data.size() || j >= data.size())
    IndexOutOfBoundsException().raise();

  // check if a zero element is addressed
  if((i > j) && ((i - j) > 1) ||
     (j > i) && ((j - i) > 1))
    return zero;

  return data[2 * i + j];
}


template <typename T>
const T& TriDiagonalMatrix<T>::operator() (unsigned i, unsigned j) const
{
  static T zero = 0;;

  // formula for addressing an element: adr = 2 * i + j
  // where |i - j| must be less equal 1

  // make sure i and j are in bounds
  if(i >= data.size() || j >= data.size())
    IndexOutOfBoundsException().raise();

  // check if a zero element is addressed
  if((i > j) && ((i - j) > 1) ||
     (j > i) && ((j - i) > 1))
    return zero;

  return data[2 * i + j];
}


/**
 *
 *
 * status: tested! (high chance that this function yields correct results)
 */
template<class T>
TriDiagonalMatrixPair<T> TriDiagonalMatrix<T>::create_lu_decomposition() const
{
  unsigned n = get_dimension(); //data.size();
  
  TriDiagonalMatrix<T> mat_l(n);
  TriDiagonalMatrix<T> mat_u(n);
  
  // init diagonal of mat_l to 1
  for(unsigned i = 0; i < n; ++i)
    mat_l(i, i) = static_cast<T>(1);  
  
  // copy c(i) to mat_u
  for(unsigned i = 0; i < n - 1; ++i)
    mat_u(i, i + 1) = (*this)(i, i + 1);
  
  // set d0
  mat_u(0, 0) = (*this)(0, 0);
  
  for(unsigned i = 1; i < n; ++i) {
    // calculate l(i)
    mat_l(i, i - 1) = (*this)(i, i - 1) / mat_u(i - 1, i - 1);

    // calculate d's
    mat_u(i, i)     = (*this)(i, i) - mat_l(i, i - 1) * (*this)(i - 1, i);
  }

  return TriDiagonalMatrixPair<T>(mat_l, mat_u);
}



/**
 * solve
 *
 * special solve function for tridiagonal matrices
 *
 * status: tested! (high chance that this function yields correct results)
 */
template <typename T>
Vector<T> solve(const TriDiagonalMatrix<T> &r_mat, const Vector<T> &r_vector)
{
  // check that the dimensions match
  if(r_mat.get_dimension() != r_vector.get_dimension())
    DimensionMismatchException().raise();

  unsigned n = r_vector.get_dimension();

  TriDiagonalMatrixPair<T> lu_pair(r_mat.create_lu_decomposition());

  TriDiagonalMatrix<T> &r_mat_l = lu_pair.first;
  TriDiagonalMatrix<T> &r_mat_u = lu_pair.second;


  // determine the y's (the intermediate vector)
  Vector<T> y(n);

  y(0) = r_vector(0);
  for(unsigned i = 1; i < n; ++i)
    y(i) = r_vector(i) - r_mat_l(i, i - 1) * y(i - 1);

  // determine the x's (the result vector)
  Vector<T> x(n);

  x(n - 1) = y(n - 1) / r_mat_u(n - 1, n - 1);
  for(unsigned i = n - 2; i != static_cast<unsigned>(~0); --i)
    x(i) = (y(i) - r_mat_u(i, i + 1) * x(i + 1)) / r_mat_u(i, i);
    
  return x;
}


}


#endif
