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

#ifndef __PARAMETRIZABLE_H
#define __PARAMETRIZABLE_H

#include <selforg/matrix.h>
#include <list>
#include <functional>

typedef std::list<matrix::Matrix> ParameterList;
/// using ParameterList = std::list<matrix::Matrix>;

/**
   Interface for parametrizable controller.
   Which expose a set of parameters that be set from outside.
*/
class Parametrizable {
public:

  virtual ~Parametrizable() {}

  /** Returns a list of matrices that parametrize the controller
   */
  virtual ParameterList getParameters() const = 0;

  /** sets the parameters.
      The list must have the same length as returned by getParameters()
      The matrix dimensions must fit those given by getParameters()
      @return 0 for failure, 1 for success and 2 if parameters had been changed during setting (to be valid)
         such that a get is required to get the actual values
   */
  virtual int setParameters(const ParameterList& params) = 0;;

};

ParameterList lift2PL(const ParameterList& pl1,const ParameterList& pl2,
                      std::function<matrix::Matrix (const matrix::Matrix&, const matrix::Matrix&)> fun);
ParameterList liftPL(const ParameterList& pl,std::function<matrix::Matrix (const matrix::Matrix&)> fun);

ParameterList mapPL(const ParameterList& pl, double (*fun)(double));
ParameterList divCompPL(const ParameterList& pl1,const ParameterList& pl2);

// assign values of pls to pld (keeps references to pld valid)
void assignPL(ParameterList& pld, const ParameterList& pls);

ParameterList addPL(const ParameterList& pl1,const ParameterList& pl2);
ParameterList subtractPL(const ParameterList& pl1,const ParameterList& pl2);
ParameterList scalePL(const ParameterList& pl,double f);

#endif
