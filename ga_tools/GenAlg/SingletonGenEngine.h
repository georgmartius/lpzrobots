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
 *   Revision 1.2  2009-04-28 13:23:55  robot12
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
//API
#include "IGenerationSizeStrategie.h"
#include "IFitnessStrategie.h"
//END API

class SingletonGenEngine {
public:
	SingletonGenEngine();
	virtual ~SingletonGenEngine();

	inline const std::vector<GenPrototyp*>& getSetOfGenPrototyps(void)const {}
	inline int getNumGeneration(void)const {return m_generation.size();}
	inline Generation* getGeneration(int x) {if(x<getNumGeneration())return m_generation[x];return NULL;}
	inline int getActualGenerationNumber(void)const {return m_actualGeneration;}

	inline void addGenPrototyp(GenPrototyp* prototyp) {m_prototyp.push_back(prototyp);}

	void generateFirstGeneration(void);
	void generateNextGeneration(void);

	// API
	void select(void);
	void crosover(void);
	// END API

	void runGenAlg(void);

	// API
	inline void setGenerationSizeStrategie(IGenerationSizeStrategie* strategie) {Generation::setGenerationSizeStrategie(strategie);}
	inline void setFitnessStrategie(IFitnessStrategie* strategie) {Individual::setFitnessStrategie(strategie);}

	inline IMutationFactorStrategie* createFixMutationFactorStrategie(void)const {return new FixMutationFactorStrategie();}
	inline IMutationFactorStrategie* createStandartMutationFactorStrategie(void)const {return new StandartMutationFactorStrategie();}
	inline IGenerationSizeStrategie* createFixGenerationSizeStrategie(void)const {return new FixGenerationSizeStrategie();}
	inline IGenerationSizeStrategie* createStandartGenerationSizeStrategie(void)const {return new StandartGenerationSizeStrategie();}
	// END API

protected:
	std::vector<GenPrototyp*> m_prototyp;
	std::vector<Generation*> m_generation;

	int m_actualGeneration;
};

#endif /* SINGLETONGENENGINE_H_ */
