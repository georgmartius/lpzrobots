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
#ifndef __SPARSEMATRIX_H_
#define __SPARSEMATRIX_H_

#include "sparsearray.h"

namespace matrix
{

  /**
   * sparse matrix which uses an HashTable
   * first (fast implemented) version
   * @author guettler
   */
  template<typename I, typename D> class SparseMatrix : public matrix::SparseArray<I, D>
  {
  public:
     SparseMatrix(I m, I n) : SparseArray<I, D>(m*n), m(m), n(n) {}

     virtual ~SparseMatrix() {}

     virtual inline D val(I row, I column) const { return (*this)[row*m+column]; }

     virtual inline D& val(I row, I column) { return (*this)[row*m+column]; }

     virtual inline I getM() { return m; }
     virtual inline I getN() { return n; }

   protected:
     I m;
     I n;
  };

}

#ifdef UNITTEST
#include "sparsematrix.tests.hpp"
#endif

#endif /* __SPARSEMATRIX_H_ */
