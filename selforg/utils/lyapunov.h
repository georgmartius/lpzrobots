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
#ifndef __LYAPUNOV_H
#define __LYAPUNOV_H

#include "matrix.h"
#include "stl_map.h"

/**
 *  Class for calculating lyapunov exponents
 *   online, over several time horizons, from given Jacobi matrices
*/
class Lyapunov {
public:
  /// holds a matrix that is the result of a sliding window multiplication
  struct SlidingMatrix {
    /** @param dim dimension of the system (matrix is (dim x dim))
        @param horizon for sliding window
     */
    SlidingMatrix(int dim, int horizon);
    void step(int t, const matrix::Matrix* buffer,
              const matrix::Matrix* invbuffer, int buffersize);
    /** nominal size of sliding window
        (if <=0 then infinite and absolute value stands for the size so far) */
    int horizon;
    matrix::Matrix M;   ///<  accumulated Matrix
    matrix::Matrix Exp; ///< Lyapunov exponents
  };

  typedef HashMap< int, SlidingMatrix* > Horizons;

public:
  Lyapunov();
  ~Lyapunov();

  /** initializes with a set of horizons.
      @param horizons for each horizon # in steps. 0 means infinite
      @param dim # of dimensions (expect a (dim x dim) matrix in step)
   */
  void init(const std::list<int>& horizons, int dim);

  /** provides the current Jacobi matrix.
      Internally the sliding windows and the exponents are generated
   */
  void step(const matrix::Matrix& jacobi);

  /** returns the lyapunov matrix at the given horizon
   */
  const matrix::Matrix& getLyapunovMatrix(int horizon);

  /** returns the lyapunov exponents at the given horizon
   */
  const matrix::Matrix& getLyapunovExp(int horizon);

protected:
  matrix::Matrix* buffer;
  matrix::Matrix* invbuffer; // buffer for inverses
  int buffersize;
  long int t;

  Horizons horizons;
};


#endif

