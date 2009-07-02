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
 *   This class is a facade for the gen. alg. It covert all function to    *
 *   work with the alg.                                                    *
 *                                                                         *
 *   $Log$
 *   Revision 1.5  2009-07-02 15:24:53  robot12
 *   update and add new class InvertedFitnessStrategy
 *
 *   Revision 1.4  2009/06/30 14:20:49  robot12
 *   finishing the gen API and add some comments
 *
 *   Revision 1.3  2009/05/11 14:08:51  robot12
 *   patch some bugfix....
 *
 *   Revision 1.2  2009/05/06 13:28:22  robot12
 *   some implements... Finish
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
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

#include "GenPrototype.h"
#include "GenContext.h"
#include "Gen.h"
#include "Individual.h"

#include "IGenerationSizeStrategy.h"
#include "FixGenerationSizeStrategy.h"
#include "StandartGenerationSizeStrategy.h"

#include "IFitnessStrategy.h"
#include "SumFitnessStrategy.h"
#include "EuclidicDistanceFitnessStrategy.h"
#include "ExtreamTestFitnessStrategy.h"
#include "TestFitnessStrategy.h"

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

#include <selforg/plotoptionengine.h>

SingletonGenAlgAPI* SingletonGenAlgAPI::m_api = 0;

SingletonGenAlgAPI::SingletonGenAlgAPI() {
	m_plotEngine = 0;
	m_plotEngineGenContext = 0;
}

SingletonGenAlgAPI::~SingletonGenAlgAPI() {
	if(m_plotEngine!=0)
		delete m_plotEngine;

	if(m_plotEngineGenContext!=0)
		delete m_plotEngineGenContext;
}

IFitnessStrategy* SingletonGenAlgAPI::createSumFitnessStrategy()const {
	return new SumFitnessStrategy();
}

IFitnessStrategy* SingletonGenAlgAPI::createEuclidicDistanceFitnessStrategy()const {
	return new EuclidicDistanceFitnessStrategy();
}

IFitnessStrategy* SingletonGenAlgAPI::createExtreamTestFitnessStrategy(IFitnessStrategy* fitness)const {
	return new ExtreamTestFitnessStrategy(fitness);
}

IFitnessStrategy* SingletonGenAlgAPI::createTestFitnessStrategy()const {
	return new TestFitnessStrategy();
}

IRandomStrategy* SingletonGenAlgAPI::createDoubleRandomStrategy(RandGen* random, double base, double factor, double epsilon)const {
	return new DoubleRandomStrategy(random,base,factor,epsilon);
}

IMutationStrategy* SingletonGenAlgAPI::createValueMutationStrategy(IMutationFactorStrategy* strategy, int mutationProbability)const {
	return new ValueMutationStrategy(strategy,mutationProbability);
}

IMutationFactorStrategy* SingletonGenAlgAPI::createFixMutationFactorStrategy(IValue* value)const {
	return new FixMutationFactorStrategy(value);
}
// nothing
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

void SingletonGenAlgAPI::update(double factor) {
	SingletonGenEngine::getInstance()->update(factor);
}

void SingletonGenAlgAPI::prepare(int startSize, int startKillRate) {
	SingletonGenEngine::getInstance()->prepare(startSize, startKillRate, m_plotEngine, m_plotEngineGenContext);
}

void SingletonGenAlgAPI::runGenAlg(int startSize, int startKillRate, int numGeneration, RandGen* random) {
	SingletonGenEngine::getInstance()->runGenAlg(startSize,startKillRate,numGeneration,random,m_plotEngine,m_plotEngineGenContext);
}

GenPrototype* SingletonGenAlgAPI::createPrototype(std::string name, IRandomStrategy* randomStrategy, IMutationStrategy* mutationStrategy)const {
	return new GenPrototype(name,randomStrategy,mutationStrategy);
}

void SingletonGenAlgAPI::insertGenPrototype(GenPrototype* prototype) {
	SingletonGenEngine::getInstance()->addGenPrototype(prototype);
}

void SingletonGenAlgAPI::enableMeasure(const PlotOption& plotOption) {
	m_plotEngine = new PlotOptionEngine(plotOption);
}

void SingletonGenAlgAPI::enableMeasure(const std::list<PlotOption>& plotOptions) {
	m_plotEngine = new PlotOptionEngine(plotOptions);
}

void SingletonGenAlgAPI::enableGenContextMeasure(const PlotOption& plotOption) {
	m_plotEngineGenContext = new PlotOptionEngine(plotOption);
}

void SingletonGenAlgAPI::enableGenContextMeasure(const std::list<PlotOption>& plotOptions) {
	m_plotEngineGenContext = new PlotOptionEngine(plotOptions);
}
