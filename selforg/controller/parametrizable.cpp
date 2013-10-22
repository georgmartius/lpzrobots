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

#include "parametrizable.h"
#include <selforg/stl_adds.h>
#include <functional>

ParameterList lift2PL(const ParameterList& pl1,const ParameterList& pl2,
                      std::function<matrix::Matrix (const matrix::Matrix&, const matrix::Matrix&)> fun){
  ParameterList res;
  assert(pl1.size()==pl2.size());
  FOREACH2(pl1, pl2, p1, p2){
    res.push_back(fun(*p1,*p2));
  }
  return res;
}

ParameterList liftPL(const ParameterList& pl,std::function<matrix::Matrix (const matrix::Matrix&)> fun){
  ParameterList res;
  for (auto& p : pl)
    res.push_back(fun(p));
  return res;
}

ParameterList mapPL(const ParameterList& pl, double (*fun)(double)){
  return liftPL(pl, [&fun](const matrix::Matrix& m) { return m.map(fun);});
}

// compoment-wise division
ParameterList divCompPL(const ParameterList& pl1,const ParameterList& pl2){
  auto div = [](double d1, double d2) {return d1/d2;};
  return lift2PL(pl1,pl2, [&div](const matrix::Matrix& m1, const matrix::Matrix& m2)
                 {return matrix::Matrix::map2(div, m1,m2);} );
}

void assignPL(ParameterList& pld, const ParameterList& pls){
  ParameterList::const_iterator ps = pls.begin();
  ParameterList::const_iterator __end2=pls.end();
  for( ParameterList::iterator pd = pld.begin(), __end1=pld.end(); pd!= __end1 && ps!= __end2; pd++ , ps++)
    *pd = *ps;
}

ParameterList addPL(const ParameterList& pl1,const ParameterList& pl2){
  return lift2PL(pl1,pl2, [](const matrix::Matrix& m1, const matrix::Matrix& m2) { return m1 + m2; });
}

ParameterList subtractPL(const ParameterList& pl1,const ParameterList& pl2){
  return lift2PL(pl1,pl2, [](const matrix::Matrix& m1, const matrix::Matrix& m2) { return m1 - m2; });
}

ParameterList scalePL(const ParameterList& pl,double f){
  return liftPL(pl, [f](const matrix::Matrix& m) { return m * f; });
}
