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
 *   Revision 1.1  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "Generation.h"

Generation::Generation() {
	// nothing
}

Generation::Generation(int generationNumber, IGenerationSizeStrategie* strategie, int size, int kill) {
	m_strategie = strategie;
	m_generationNumber = generationNumber;
	m_size=size;
	m_kill = kill;
}

Generation::~Generation() {
	delete m_strategie;
	m_individual.clear();
}

void Generation::update(void) {
	m_nextGenSize = m_strategie->calcGenerationSize(/*parameters unknown*/);
}

void Generation::select(Generation* newGeneration) {
	m_individual.sort();

	int num = getCurrentSize();
	for(int x=0;x<m_size-m_kill && x<newGeneration->getSize();x++){
		newGeneration->addIndividual(m_individual[x]);
	}
}

void Generation::crosover(RandGen* random) {
	int r1,r2;
	Individual* ind;

	while(getCurrentSize()<m_size) {
		r1 = ((int)random->rand())%getCurrentSize();
		r2 = r1;
		while(r1==r2)
			r2 = ((int)random->rand())%getCurrentSize();

		ind = SingletonIndividualFactory::getInstance()->createIndividual(m_individual[r1],m_individual[r2],random);
		addIndividual(ind);
	}
}
