/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
 *    mai00bvz@studserv.uni-leipzig.de                                     *
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
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-07-16 13:07:27  robot12
 *   some comments added
 *
 *   Revision 1.1  2009/07/15 12:56:25  robot12
 *   the simulation
 *
 *
 ***************************************************************************/

#include "TemplateCycledGaSimulationFitnessStrategy.h"

#include <ga_tools/Individual.h>

TemplateCycledGaSimulationFitnessStrategy::TemplateCycledGaSimulationFitnessStrategy() {
	// nothing
}

TemplateCycledGaSimulationFitnessStrategy::~TemplateCycledGaSimulationFitnessStrategy() {
	// nothing
}

double TemplateCycledGaSimulationFitnessStrategy::getFitness(const Individual* individual) {
	double* temp = m_storage[individual->getID()];
	// add 0.1 because the most individual have a entropy from zero. And so the inverted fitness strategy return 1/0 = inf!!1
	*temp += 0.1;
	return *(m_storage[individual->getID()]);
}
