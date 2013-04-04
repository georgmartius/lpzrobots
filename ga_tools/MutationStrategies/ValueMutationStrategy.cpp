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
        m_strategy = NULL;
}

Gen* ValueMutationStrategy::mutate(GenContext* context, Individual* individual, Gen* oldGen, GenContext* oldContext, SingletonGenFactory* factory) {
        IValue* factor;                                                //the mutation factor
        IValue* value;                                                //the value of the gen

        factor = m_strategy->calcMutationFactor(oldContext->getGene());                        //become the mutation factor
        value = oldGen->getValue()->operator+(*factor);                                                        //become the value of the gen and add the factor
        //delete factor;

        return factory->createGen(context,individual,context->getPrototype(),value);        //create the new gen
}

int ValueMutationStrategy::getMutationProbability(void) {
        return m_mutationProbability;
}
