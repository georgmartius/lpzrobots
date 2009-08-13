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

#include "Matrix.h"

#include <iostream>
#include <cassert>

Matrix::Matrix()
{
	//empty
}

//----------------------------------------------------------------------------

Matrix::Matrix( float *values ) //TODO füllen mittels InputReader
{
  //memcpy(this->values, values, 16*sizeof(float));
}

double Matrix::getVal( int row, int col)
{
	assert( row <= rowSize && col <= columnSize);
	return values[row * rowSize + col];
}

void Matrix::setVal(int row, int column, double val)
{
	values[row * rowSize + column] = val;
}


//----------------------------------------------------------------------------

std::ostream& operator<<( std::ostream& out, const Matrix& m )
{
    m.write( out );
    return out;
}

void Matrix::write( std::ostream& out ) const
{
	for (int i = 0; i < rowSize; i++){
		for (int j = 0; j < columnSize; j++){
			std::cout << values[ rowSize*i + columnSize ];
			if(!(j == columnSize - 1)) out << " ";
		}
		out << std::endl;
	}
//  out << values[0] << " " << values[4] << " "
//      << values[8] << " " << values[12] << endl
//      << values[1] << " " << values[5] << " "
//      << values[9] << " " << values[13] << endl
//      << values[2] << " " << values[6] << " "
//      << values[10] << " " << values[14] << endl
//      << values[3] << " " << values[7] << " "
//      << values[11] << " " << values[15] << endl;
}

//----------------------------------------------------------------------------

//const double* Matrix::getValues() const
//{
//  return values;
//}

