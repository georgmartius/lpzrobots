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
 *   Revision 1.2  2009-05-06 14:35:15  robot12
 *   implements findBest in StandartGenerationSizeStrategy
 *
 *   Revision 1.1  2009/05/04 15:27:57  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.3  2009/05/04 09:06:00  robot12
 *   some implements... Part7
 *
 *   Revision 1.2  2009/05/04 07:06:14  robot12
 *   some implements... Part6
 *
 *   Revision 1.1  2009/04/30 11:51:25  robot12
 *   some implements... new classes
 *
 *
 *
 ***************************************************************************/

#include "StandartGenerationSizeStrategy.h"

#include "Generation.h"
#include "Individual.h"

#define GREATER_HIGH_FACTOR 1.1
#define GREATER_LOW_FACTOR 1.05
#define LOWER_HIGH_FACTOR 0.9
#define LOWER_LOW_FACTOR 0.95

StandartGenerationSizeStrategy::StandartGenerationSizeStrategy() {
	// nothing
}

StandartGenerationSizeStrategy::StandartGenerationSizeStrategy(int startSize, int numGeneration) {
	m_startSize = startSize;
	m_numGeneration = numGeneration;
	m_firstIsSet = false;
}

StandartGenerationSizeStrategy::~StandartGenerationSizeStrategy() {
	// nothing
}

int StandartGenerationSizeStrategy::calcGenerationSize(Generation* oldGeneration) {
	double best = findBest(oldGeneration);

	if(!m_firstIsSet) {
		m_firstIsSet = true;
		m_best_first = best;
		m_best_new = best;

		return m_startSize;
	}

	m_best_old = m_best_new;
	m_best_new = best;

	// targetDevelop means the speed with this the alg. should become better with every round
	double targetDevelop = m_best_first / (double)m_numGeneration;

	// develop is the value how much the alg. is better from the last to this round
	double develop = m_best_old - m_best_new;

	// standDevelop is the differense between the targetDevelop in this round should be and the real stand
	double standDevelop = m_best_first - (((double)oldGeneration->getGenerationNumber()) * targetDevelop) - m_best_new;

	if(standDevelop>0.0) { // we are to fast --> GenerationSize must be greater!?!
		if(develop>=targetDevelop) { // yes the GenerationSize must be greater
			return (int)(GREATER_HIGH_FACTOR*oldGeneration->getSize());
		}
		else if((targetDevelop-develop)*(double)(m_numGeneration-oldGeneration->getGenerationNumber())+standDevelop>0.0) { // yes the GenerationSize must be greater but not so much
			return (int)(GREATER_LOW_FACTOR*oldGeneration->getSize());
		}
		else { //develop<targetDevelop  // no the speed is to low and it correct him self
			return oldGeneration->getSize();
		}
	}
	else if(standDevelop==0.0) { // we are of correct course
		return oldGeneration->getSize();
	}
	else { // standDevelop<0.0 // we are to slow --> GenerationSize must be lower!?!
		if(develop<=targetDevelop) { // yes the GenerationSize must be lower
			return (int)(LOWER_HIGH_FACTOR*oldGeneration->getSize());
		}
		else if((targetDevelop-develop)*(double)(m_numGeneration-oldGeneration->getGenerationNumber())+standDevelop<0.0) { // yes the GenerationSize must be lower
			return (int)(LOWER_LOW_FACTOR*oldGeneration->getSize());
		}
		else { //develop>targetDevelop  // no the speed is to high and it correct him self
			return oldGeneration->getSize();
		}
	}
}

double StandartGenerationSizeStrategy::findBest(Generation* oldGeneration) {
	double best = oldGeneration->getIndividual(0)->getFitness();
	int num = oldGeneration->getCurrentSize();

	for(int x=1;x<num;x++) {
		if(oldGeneration->getIndividual(x)->getFitness()<best)
			best=oldGeneration->getIndividual(x)->getFitness();
	}

	return best;
}
