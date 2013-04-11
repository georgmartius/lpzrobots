/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Joerg Weider   <joergweide84 at aol dot com> (robot12)               *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *    Joern Hoffmann <jhoffmann at informatik dot uni-leipzig dot de       *
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

#include "DoubleRandomStrategy.h"

#include "TemplateValue.h"

DoubleRandomStrategy::DoubleRandomStrategy() {
        // nothing
}

DoubleRandomStrategy::DoubleRandomStrategy(RandGen* random, double base, double factor, double epsilon) {
        m_random = random;
        m_base = base;
        m_factor = factor;
        m_epsilon = epsilon;
}

DoubleRandomStrategy::~DoubleRandomStrategy() {
        m_random = NULL;
}

IValue* DoubleRandomStrategy::getRandomValue(void) {
        double value = (m_random->rand()*m_factor)+m_base;                                                                //create a random double in the
                                                                                                                                                                        //interval [m_base,m_factor+m_base]
        return new TemplateValue<double>(value + (value>0?m_epsilon:-1.0*m_epsilon));        //if the value negative move it
                                                                                                                                                                        //-m_epsilon else move it +m_epsilon
                                                                                                                                                                        //and return it as IValue
                                                                                                                                                                        //(TemplateValue<double>)
}
