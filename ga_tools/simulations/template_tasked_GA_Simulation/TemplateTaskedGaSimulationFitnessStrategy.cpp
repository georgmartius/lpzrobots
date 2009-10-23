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
 *   Revision 1.1  2009-10-23 16:33:51  martius
 *   moved simulations here
 *
 *   Revision 1.1  2009/10/01 13:33:38  robot12
 *   rename of the TemplateCycledGaSimulationFitnessStrategy into TemplateTaskedGaSimulationFitnessStrategy
 *
 *   Revision 1.1  2009/08/24 11:09:56  robot12
 *   new template for tasked simulations use the ga_tools added
 *
 *   Revision 1.2  2009/08/11 13:00:51  robot12
 *   change the genetic algorithm
 *
 *   Revision 1.1  2009/07/21 09:09:05  robot12
 *   rename of the project from "temlate_..." to template_..."
 *
 *   Revision 1.2  2009/07/16 13:07:27  robot12
 *   some comments added
 *
 *   Revision 1.1  2009/07/15 12:56:25  robot12
 *   the simulation
 *
 *
 ***************************************************************************/

#include "TemplateTaskedGaSimulationFitnessStrategy.h"

#include <ga_tools/Individual.h>

TemplateTaskedGaSimulationFitnessStrategy::TemplateTaskedGaSimulationFitnessStrategy() {
	// nothing
}

TemplateTaskedGaSimulationFitnessStrategy::~TemplateTaskedGaSimulationFitnessStrategy() {
	// nothing
}

double TemplateTaskedGaSimulationFitnessStrategy::getFitness(const Individual* individual) {
	// return 0.0000000001 because the most individual have a entropy from zero. And so the inverted fitness strategy return 1/0 = inf!!!
  if(m_storage[individual->getID()]==0.0)
    return 0.0000000001;

	return m_storage[individual->getID()];
}
