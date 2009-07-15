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
 *   Revision 1.1  2009-07-15 12:56:25  robot12
 *   the simulation
 *
 *
 ***************************************************************************/

#ifndef TEMPLATECYCLEDGASIMULATIONFITNESSSTRATEGY_H_
#define TEMPLATECYCLEDGASIMULATIONFITNESSSTRATEGY_H_

#include <ga_tools/IFitnessStrategy.h>
#include <vector>

class TemplateCycledGaSimulationFitnessStrategy: public IFitnessStrategy {
public:
	TemplateCycledGaSimulationFitnessStrategy();
	virtual ~TemplateCycledGaSimulationFitnessStrategy();

	virtual double getFitness(const Individual* individual);

	std::vector<double*> m_storage;
};

#endif /* TEMPLATECYCLEDGASIMULATIONFITNESSSTRATEGY_H_ */
