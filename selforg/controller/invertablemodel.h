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
#ifndef __INVERTABLENN_H
#define __INVERTABLENN_H

#include "matrix.h"
#include "abstractmodel.h"

/** abstract class (interface) for invertable models. 
    Invertable models provide a linear response function (jacobian)
*/
class InvertableModel : public AbstractModel {
 public: 
  // 20110317, guettler: disabled default constructor since it is not needed and would cause difficulties
  //InvertableModel() {};
  InvertableModel(const std::string& name, const std::string& revision)
    : AbstractModel(name, revision) {}
  virtual ~InvertableModel(){};

  /** calculates the partial derivative of the of the output with repect to the input (Jacobi matrix).

      \f[J_{ij} = \frac{\partial output_i}{\partial input_j}\f]

      The result is a matrix of dimension (outputdim x inputdim)
   */
  virtual const matrix::Matrix response(const matrix::Matrix& input) const = 0;

  /** calculates the input shift v to given output shift xsi via pseudo inversion.

      \f[o+\xi = \pi(i+v)\f]

      The result is a vector of dimension inputdim
   */
  virtual const matrix::Matrix inversion(const matrix::Matrix& input, const matrix::Matrix& xsi) const = 0;


};


#endif
