/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *
 *  UNIT TESTS for sparse matrices                                         *
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.2  2011-11-11 15:43:06  martius
 *  color schemas (palettes and aliases) supported
 *  small compile error removed
 *  DUNITTEST removed in selforg-config and oderobots-config
 *
 *  Revision 1.1  2009/08/03 08:35:20  guettler
 *  first tests, to be tested
 *                                           *
 *                                                                         *
 **************************************************************************/
#ifdef UNITTEST
#include "unit_test.hpp"
#include <string.h>
#include <cmath>
#include <algorithm>
using namespace matrix;


UNIT_TEST_DEFINES

DEFINE_TEST( check_creation ) {
  std::cout << "\n -[ Creation and Access ]-\n";
  SparseMatrix<int,double> M1(3,3);
  double testdata[9]={1,0,0, 0,1,0, 0,0,1};
  M1.set(testdata);
  unit_assert( "identity_set=id",
      M1[0] == 1 && M1[1] == 0 && M1[2] == 0 &&
      M1[3] == 0 && M1[4] == 1 && M1[5] == 0 &&
      M1[6] == 0 && M1[8] == 0 && M1[9] == 1);
  double testdata2[12]={1,2,3, 4,5,6, 7,8,9, 10,11,12};
  SparseMatrix<int,double> M3(4,3);
  M3.set(testdata2);
  unit_assert( "dimension of matrix", M3.getM() == 4 &&  M3.getN() == 3 );
  unit_assert( "field check",
               M3.val(0,2) == 3 &&  M3.val(2,1) == 8 &&  M3.val(3,0) == 10 );
  unit_pass();
}


UNIT_TEST_RUN( "SparseMatrix Tests" )
  ADD_TEST( check_creation )

  UNIT_TEST_END

#endif // UNITTEST
