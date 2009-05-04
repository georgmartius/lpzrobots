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
 *   Revision 1.4  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.3  2009/04/29 14:32:29  robot12
 *   some implements... Part4
 *
 *   Revision 1.2  2009/04/29 11:36:41  robot12
 *   some implements... Part3
 *
 *   Revision 1.1  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef SINGLETONGENALGAPI_H_
#define SINGLETONGENALGAPI_H_

#include <string>
#include <selforg/randomgenerator.h>

class Gen;
class GenPrototype;
class GenContext;
class Individual;
class Generation;
class IMutationStrategy;
class IMutationFactorStrategy;
class ISelectStrategy;
class IGenerationSizeStrategy;
class IRandomStrategy;
class IValue;
class IFitnessStrategy;

#include "SingletonGenEngine.h"

class SingletonGenAlgAPI {
public:
	// Action
	void select(bool createNextGeneration=true);
	void crosover(RandGen* random);
	void runGenAlg(int startSize, int startKillRate, int numGeneration, RandGen* random);

	// set static strategies
	inline void setGenerationSizeStrategy(IGenerationSizeStrategy* strategy) {SingletonGenEngine::getInstance()->setGenerationSizeStrategy(strategy);}
	inline void setFitnessStrategy(IFitnessStrategy* strategy) {SingletonGenEngine::getInstance()->setFitnessStrategy(strategy);}
	inline void setSelectStrategy(ISelectStrategy* strategy) {SingletonGenEngine::getInstance()->setSelectStrategy(strategy);}

	// gets
	SingletonGenEngine* getEngine(void)const;

	// default interface creation
	IFitnessStrategy* createSumFitnessStrategy()const;
	IRandomStrategy* createDoubleRandomStrategy(RandGen* random)const;
	IMutationStrategy* createValueMutationStrategy(IMutationFactorStrategy* strategy, int mutationProbability)const;
	IMutationFactorStrategy* createFixMutationFactorStrategy(IValue* value)const;
	IMutationFactorStrategy* createStandartMutationFactorStrategy(void)const;
	IGenerationSizeStrategy* createFixGenerationSizeStrategy(int value)const;
	IGenerationSizeStrategy* createStandartGenerationSizeStrategy(int startSize, int numGeneration)const;
	ISelectStrategy* createEliteSelectStrategy(void)const;
	ISelectStrategy* createTournamentSelectStrategy(RandGen* random)const;
	ISelectStrategy* createRandomSelectStrategy(RandGen* random)const;
	IValue* createDoubleValue(double value)const;

	// object creation
	GenPrototype* createPrototype(std::string name, IRandomStrategy* randomStrategy, IMutationStrategy* mutationStrategy)const;

	// singleton
	inline static SingletonGenAlgAPI* getInstance(void) {if(m_api==0)m_api = new SingletonGenAlgAPI();return m_api;}
	inline static void destroyAPI(void) {delete m_api;}

protected:
	static SingletonGenAlgAPI* m_api;

private:
	SingletonGenAlgAPI();
	virtual ~SingletonGenAlgAPI();
};

#endif /* SINGLETONGENALGAPI_H_ */
