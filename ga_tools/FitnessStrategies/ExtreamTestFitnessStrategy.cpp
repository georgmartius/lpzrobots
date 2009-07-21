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
 *   This is a example implementation for a fitness strategy. It use a     *
 *   other fitness strategy to calculate a value. If the value is lower    *
 *   than 10 it will give value*value back. Else it will be 100.           *
 *   So the most values are by 100 and of one plane. The gen. Alg lose by  *
 *   this strategy his information how to optimize the individual. Because *
 *   all individual are equal good. Only small part of it are better.      *
 *   This is the worst case for a gen. Alg.                                *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.1  2009/06/15 13:58:36  robot12
 *   3 new fitness strategys and IFitnessStrategy and SumFitnessStragegy with comments.
 *
 *
 *
 *
 ***************************************************************************/

#include "ExtreamTestFitnessStrategy.h"

ExtreamTestFitnessStrategy::ExtreamTestFitnessStrategy() {
	// nothing
}

ExtreamTestFitnessStrategy::ExtreamTestFitnessStrategy(IFitnessStrategy* fitness) : m_strategy(fitness){
	//m_strategy = fitness;
	// nothing
}

ExtreamTestFitnessStrategy::~ExtreamTestFitnessStrategy() {
	// clean
	// do not forget to delete the other fitness strategy
	delete m_strategy;
}

double ExtreamTestFitnessStrategy::getFitness(const Individual* individual) {
	// first use the other fitness strategy to calculate the base value!
	double value = m_strategy->getFitness(individual);

	//if it lower than 10 -> return value² else 100
	if(value<10.0 && value>-10.0) {
		return value*value;
	}
	else {
		return 100.0;
	}
}
