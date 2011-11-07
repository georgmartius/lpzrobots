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

#ifndef FIXMUTATIONFACTORSTRATEGY_H_
#define FIXMUTATIONFACTORSTRATEGY_H_

//includes
#include <vector>

//forward declaration
class IValue;
class Gen;

//ga_tools includes
#include "IMutationFactorStrategy.h"

/**
 * This strategy implementation return a fix value for the mutation factor.
 */
class FixMutationFactorStrategy : public IMutationFactorStrategy {
public:
	/**
	 * constructor
	 * @param value (IValue*) the fix Value
	 */
	FixMutationFactorStrategy(IValue* value);

	/**
	 * default destructor
	 */
	virtual ~FixMutationFactorStrategy();

	/**
	 * returns the fix value as mutation value
	 * @param gene (vector<Gen*>) the set of gens -> here not used.
	 * @return (IValue*) the mutation factor
	 */
	virtual IValue* calcMutationFactor(const std::vector<Gen*>& gene);

protected:
	/**
	 * the saved fix value which is giving back.
	 */
	IValue* m_value;

private:
	/**
	 * disable default constructor
	 */
	FixMutationFactorStrategy();
};

#endif /* FIXMUTATIONFACTORSTRATEGY_H_ */
