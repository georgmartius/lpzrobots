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
 *   Revision 1.2  2009-05-04 07:06:14  robot12
 *   some implements... Part6
 *
 *   Revision 1.1  2009/04/30 11:51:25  robot12
 *   some implements... new classes
 *
 *
 *
 ***************************************************************************/

#include "StandartGenerationSizeStrategie.h"

StandartGenerationSizeStrategie::StandartGenerationSizeStrategie() {
	// nothing
}

StandartGenerationSizeStrategie::StandartGenerationSizeStrategie(int startSize) {
	m_startSize = startSize;
	m_firstIsSet = false;
}

StandartGenerationSizeStrategie::~StandartGenerationSizeStrategie() {
	// nothing
}

int StandartGenerationSizeStrategie::calcGenerationSize(Generation* oldGeneration) {
	double best = findBest(oldGeneration);

	if(!m_firstIsSet) {
		m_firstIsSet = true;
		m_best_first = best;
		m_best_new = best;

		return m_startSize;
	}

	m_best_old = m_best_new;
	m_best_new = best;
}
