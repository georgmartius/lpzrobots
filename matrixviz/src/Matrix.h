/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    mam06fyl@studserv.uni-leipzig.de (robot14)                           *
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
 *   Revision 1.1  2009-08-13 13:14:05  robot14
 *   first version
 *
 *                                                                         *
 ***************************************************************************/

#ifndef MATRIX_H_
#define MATRIX_H_

#include <iostream>
#include <vector>
#include <cstring>
#include "AbstractPlotChannel.h"

class Matrix
{
 private:

  std::vector<double> values;
  int rowSize;
  int columnSize; //not really needed


 public:

  Matrix();

  Matrix( float *values );

  //const double* getValues() const;

  double getVal( int row, int column);

  void setVal( int row, int col, double val);


  friend std::ostream& operator<<( std::ostream& out, const Matrix& v );

 private:

  void write( std::ostream& out ) const;
};

#endif /* MATRIX_H_ */
