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
 *   This fitness strategy is for calculating the invert of a other        *
 *   fitness strategy.                                                     *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2009-07-02 15:24:53  robot12
 *   update and add new class InvertedFitnessStrategy
 *
 *
 *
 ***************************************************************************/

#include "InvertedFitnessStrategy.h"

InvertedFitnessStrategy::InvertedFitnessStrategy() {
	// nothing
}

InvertedFitnessStrategy::InvertedFitnessStrategy(IFitnessStrategy* strategy) : m_strategy(strategy) {
	// nothing
}

InvertedFitnessStrategy::~InvertedFitnessStrategy() {
	m_strategy = 0;
}

double InvertedFitnessStrategy::getFitness(const Individual* individual) {
	return 1.0 / m_strategy->getFitness(individual);
}
