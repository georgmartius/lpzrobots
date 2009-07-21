/***************************************************************************
 *   Copyright (C) 2005-2009 by Robot Group Leipzig                        *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    joergweide84@aol.com (robot12)                                       *
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
 *   Revision 1.3  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.2  2009/06/25 10:02:38  robot12
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

#ifndef DOUBLERANDOMSTRATEGY_H_
#define DOUBLERANDOMSTRATEGY_H_

//includes
#include <selforg/randomgenerator.h>

//forward declarations
class IValue;

//ga_tools includes
#include "IRandomStrategy.h"

/**
 * This class create a IValue (TemplateValue with type double) and initialize it with an random double value.
 * The value have basically a math. range from zero to one. With the parameter factor you can change it to zero to factor.
 * With the parameter base you can move the interval. With the last parameter you can defied the interval by zero and move
 * away from zero.
 */
class DoubleRandomStrategy : public IRandomStrategy {
public:
	/**
	 * The constructor.
	 * It becomes the 3 parameters which the class need to generate double value in a special range.
	 * @param random (RandGen*) random generator which create double values in the range zero to one.
	 * @param base (double) A parameter for the alg. it move a random value.
	 * @param factor (double) A parameter which resize the interval of the random values.
	 * @param epsilon (double) A parameter which dified the interval of the random values.
	 */
	DoubleRandomStrategy(RandGen* random, double base=0.0, double factor=1.0, double epsilon=0.0);

	/**
	 * default destructor
	 */
	virtual ~DoubleRandomStrategy();

	/**
	 * implementation for the interface. It create the random double value and give it as a IValue back (TemplateValue with type double)
	 * @return (IValue*) the random value.
	 */
	virtual IValue* getRandomValue(void);

protected:
	/**
	 * the random generator
	 */
	RandGen* m_random;

	/**
	 * base parameter. moves the random value interval.
	 */
	double m_base;

	/**
	 * factor parameter. resize the random value interval.
	 */
	double m_factor;

	/**
	 * epsilon parameter. dified the random value interval by zero and move it away.
	 */
	double m_epsilon;

private:
	/**
	 * disable default constructor
	 */
	DoubleRandomStrategy();
};

#endif /* DOUBLERANDOMSTRATEGY_H_ */
