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
 *   This class is a implementation of the interface IMutationStrategy.    *
 *                                                                         *
 *   $Log$
 *   Revision 1.6  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.5  2009/06/17 11:25:52  robot12
 *   finishing the mutation strategy and add some comments.
 *
 *   Revision 1.4  2009/05/14 15:29:56  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.3  2009/05/11 14:08:53  robot12
 *   patch some bugfix....
 *
 *   Revision 1.2  2009/05/06 13:28:23  robot12
 *   some implements... Finish
 *
 *   Revision 1.1  2009/05/04 15:27:57  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/05/04 09:06:00  robot12
 *   some implements... Part7
 *
 *   Revision 1.1  2009/04/30 11:51:26  robot12
 *   some implements... new classes
 *
 *
 *
 ***************************************************************************/

#include "ValueMutationStrategy.h"

//ga_tools includes
#include "Gen.h"
#include "Individual.h"
#include "SingletonGenFactory.h"
#include "IMutationFactorStrategy.h"
#include "GenContext.h"
#include "IValue.h"

ValueMutationStrategy::ValueMutationStrategy() {
	// nothing
}

ValueMutationStrategy::ValueMutationStrategy(IMutationFactorStrategy* strategy, int mutationProbability) {
	m_strategy = strategy;
	m_mutationProbability = mutationProbability;
}

ValueMutationStrategy::~ValueMutationStrategy() {
	// nothing
}

Gen* ValueMutationStrategy::mutate(GenContext* context, Individual* individual, Gen* oldGen, GenContext* oldContext, SingletonGenFactory* factory) {
	IValue* factor;						//the mutation factor
	IValue* value;						//the value of the gen

	factor = m_strategy->calcMutationFactor(oldContext->getGene());			//become the mutation factor
	value = oldGen->getValue()->operator+(*factor);							//become the value of the gen and add the factor
	//delete factor;

	return factory->createGen(context,individual,context->getPrototype(),value);	//create the new gen
}

int ValueMutationStrategy::getMutationProbability(void) {
	return m_mutationProbability;
}
