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
 *   Informative Beschreibung der Klasse                                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2009-05-04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/04/29 14:32:28  robot12
 *   some implements... Part4
 *
 *   Revision 1.1  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "SingletonGenAlgAPI.h"

//#include "SingletonGenEngine.h"
#include "GenPrototype.h"
#include "GenContext.h"
#include "Gen.h"
#include "Individual.h"

#include "IGenerationSizeStrategy.h"
#include "FixGenerationSizeStrategy.h"
#include "StandartGenerationSizeStrategy.h"

#include "IFitnessStrategy.h"
#include "SumFitnessStrategy.h"

#include "IRandomStrategy.h"
#include "DoubleRandomStrategy.h"

#include "IMutationStrategy.h"
#include "ValueMutationStrategy.h"

#include "IMutationFactorStrategy.h"
#include "FixMutationFactorStrategy.h"
#include "StandartMutationFactorStrategy.h"

#include "ISelectStrategy.h"
#include "EliteSelectStrategy.h"
#include "RandomSelectStrategy.h"
#include "TournamentSelectStrategy.h"

#include "IValue.h"
#include "TemplateValue.h"

static SingletonGenAlgAPI::SingletonGenAlgAPI* m_api = 0;

SingletonGenAlgAPI::SingletonGenAlgAPI() {
	// nothing
}

SingletonGenAlgAPI::~SingletonGenAlgAPI() {
	// nothing
}

IFitnessStrategy* SingletonGenAlgAPI::createSumFitnessStrategy()const {
	return new SumFitnessStrategy();
}

IRandomStrategy* SingletonGenAlgAPI::createDoubleRandomStrategy(RandGen* random)const {
	return new DoubleRandomStrategy(random);
}

IMutationStrategy* SingletonGenAlgAPI::createValueMutationStrategy(IMutationFactorStrategy* strategy, int mutationProbability)const {
	return new ValueMutationStrategy(strategy,mutationProbability);
}

IMutationFactorStrategy* SingletonGenAlgAPI::createFixMutationFactorStrategy(IValue* value)const {
	return new FixMutationFactorStrategy(value);
}

IMutationFactorStrategy* SingletonGenAlgAPI::createStandartMutationFactorStrategy(void)const {
	return new StandartMutationFactorStrategy();
}

IGenerationSizeStrategy* SingletonGenAlgAPI::createFixGenerationSizeStrategy(int value)const {
	return new FixGenerationSizeStrategy(value);
}

IGenerationSizeStrategy* SingletonGenAlgAPI::createStandartGenerationSizeStrategy(int startSize, int numGeneration)const {
	return new StandartGenerationSizeStrategy(startSize,numGeneration);
}

ISelectStrategy* SingletonGenAlgAPI::createEliteSelectStrategy(void)const {
	return new EliteSelectStrategy();
}

ISelectStrategy* SingletonGenAlgAPI::createTournamentSelectStrategy(RandGen* random)const {
	return new TournamentSelectStrategy(random);
}

ISelectStrategy* SingletonGenAlgAPI::createRandomSelectStrategy(RandGen* random)const {
	return new RandomSelectStrategy(random);
}

IValue* SingletonGenAlgAPI::createDoubleValue(double value)const {
	return new TemplateValue<double>(value);
}

SingletonGenEngine* SingletonGenAlgAPI::getEngine(void)const {
	return SingletonGenEngine::getInstance();
}

void SingletonGenAlgAPI::select(bool createNextGeneration) {
	SingletonGenEngine::getInstance()->select(createNextGeneration);
}

void SingletonGenAlgAPI::crosover(RandGen* random) {
	if(random!=NULL)
		SingletonGenEngine::getInstance()->crosover(random);
}

void SingletonGenAlgAPI::runGenAlg(int startSize, int startKillRate, int numGeneration, RandGen* random) {
	SingeltonGenEngine::getInstance()->runGenAlg(startSize,startKillRate,numGeneration,random);
}

GenPrototype* SingletonGenAlgAPI::createPrototype(std::string name, IRandomStrategy* randomStrategy, IMutationStrategy* mutationStrategy)const {
	return new GenPrototype(name,randomStrategy,mutationStrategy);
}
