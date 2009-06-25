/***************************************************************************
 *   Copyright (C) 2005-2009 by Robot Group Leipzig                        *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
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
 *   This class is a implementation for the IRandomStrategy interface and  *
 *   for the random strategy of the GenPrototype. It create a IValue from  *
 *   type double with a random double value.                               *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-06-25 10:02:38  robot12
 *   finish the Random strategy and add some comments
 *
 *   Revision 1.1  2009/05/04 15:27:57  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/04/30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.1  2009/04/30 11:51:26  robot12
 *   some implements... new classes
 *
 *
 *
 ***************************************************************************/

#include "DoubleRandomStrategy.h"

#include "TemplateValue.h"

DoubleRandomStrategy::DoubleRandomStrategy() {
	// nothing
}

DoubleRandomStrategy::DoubleRandomStrategy(RandGen* random, double base, double factor, double epsilon) {
	m_random =random;
	m_base = base;
	m_factor = factor;
	m_epsilon = epsilon;
}

DoubleRandomStrategy::~DoubleRandomStrategy() {
	// nothing
}

IValue* DoubleRandomStrategy::getRandomValue(void) {
	double value = (m_random->rand()*m_factor)+m_base;								//create a random double in the
																					//interval [m_base,m_factor+m_base]
	return new TemplateValue<double>(value + (value>0?m_epsilon:-1.0*m_epsilon));	//if the value negative move it
																					//-m_epsilon else move it +m_epsilon
																					//and return it as IValue
																					//(TemplateValue<double>)
}
