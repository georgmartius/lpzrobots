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
 *   Revision 1.5  2009-04-30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.4  2009/04/29 14:32:28  robot12
 *   some implements... Part4
 *
 *   Revision 1.3  2009/04/29 11:36:41  robot12
 *   some implements... Part3
 *
 *   Revision 1.2  2009/04/28 13:23:55  robot12
 *   some implements... Part2
 *
 *   Revision 1.1  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef SINGLETONGENENGINE_H_
#define SINGLETONGENENGINE_H_

#include "types.h"

#include <vector>

#include "GenPrototyp.h"
#include "Generation.h"
#include "Individual.h"
#include "ISelectStrategie.h"
#include "IGenerationSizeStrategie.h"
#include "IFitnessStrategie.h"
#include "Gen.h"
#include <selforg/randomgenerator.h>

class SingletonGenEngine {
public:
	inline const std::vector<GenPrototyp*>& getSetOfGenPrototyps(void)const {}
	inline int getNumGeneration(void)const {return m_generation.size();}
	inline Generation* getGeneration(int x) {if(x<getNumGeneration())return m_generation[x];return NULL;}
	inline int getActualGenerationNumber(void)const {return m_actualGeneration;}
	inline Generation* getActualGeneration(void) {return m_generation[m_actualGeneration];}

	inline int getNumIndividual(void)const {return m_individual.size();}
	inline Individual* getIndividual(int x)const {if(x<getNumIndividual())return m_individual[x];return NULL;}

	inline void addGenPrototyp(GenPrototyp* prototyp) {m_prototyp.push_back(prototyp);}
	inline void addGen(Gen* gen) {m_gen.push_back(gen);}
	inline void addIndividual(Individual* individual) {m_individual.push_back(individual);}

	void generateFirstGeneration(int startSize, int startKillRate);
	void prepareNextGeneration(void);

	void runGenAlg(int startSize, int startKillRate, int numGeneration, RandGen* random);

	inline void select(bool createNextGeneration=true) {if(createNextGeneration)prepareNextGeneration(); m_selectStrategie->select(m_generation[m_actualGeneration-1],m_generation[m_actualGeneration]);}
	inline void crosover(RandGen* random) {m_generation[m_actualGeneration]->crosover(random);}

	inline void setSelectStrategie(ISelectStrategie* strategie) {m_selectStrategie = strategie;}
	inline void setGenerationSizeStrategie(IGenerationSizeStrategie* strategie) {m_generationSizeStrategie = strategie;}
	inline void setFitnessStrategie(IFitnessStrategie* strategie) {m_fitnessStrategie = strategie;}

	inline static SingletonGenEngine* getInstance(void) {if(m_engine==0)m_engine = new SingletonGenEngine();return m_engine;}
	inline static void destroyGenEngine(void) {delete m_engine;m_engine=0;}

protected:
	std::vector<GenPrototyp*> m_prototyp;
	std::vector<Generation*> m_generation;
	std::vector<Individual*> m_individual;
	std::vector<Gen*> m_gen;

	int m_actualGeneration;

	ISelectStrategie* m_selectStrategie;
	IFitnessStrategie* m_fitnessStrategie;
	IGenerationSizeStrategie* m_generationSizeStrategie;

	static SingletonGenEngine* m_engine = 0;

private:
	SingletonGenEngine();
	virtual ~SingletonGenEngine();
};

#endif /* SINGLETONGENENGINE_H_ */
