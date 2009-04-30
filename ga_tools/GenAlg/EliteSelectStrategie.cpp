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
 *   Revision 1.2  2009-04-30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.1  2009/04/30 11:51:26  robot12
 *   some implements... new classes
 *
 *
 *
 ***************************************************************************/

#include "EliteSelectStrategie.h"

#include <list>
struct SfitnessEliteStrategieStruct {
	double fitness;
	Individual* ind;

	inline bool operator<(SfitnessEliteStrategieStruct other) {return fitness<other.fitness;}
};

EliteSelectStrategie::EliteSelectStrategie() {
	// nothing
}

EliteSelectStrategie::~EliteSelectStrategie() {
	// nothing
}

void EliteSelectStrategie::select(Generation* oldGeneration, Generation* newGeneration) {
	std::list<SfitnessEliteStrategieStruct*> list;
	SfitnessEliteStrategieStruct* storage;
	int num,x,kill;

	// prepare the list
	num = oldGeneration.getCurrentSize();
	for(x=0;x<num;x++) {
		storage = new SfitnessEliteStrategieStruct();
		storage->ind = oldGeneration->getIndividual(x);
		storage->fitness = storage->ind->getFitness();
		list.push_back(storage);
		storage = 0;
	}

	// sort the list
	list.sort();

	// kill the badest
	kill = oldGeneration->getKillRate();
	for(x=num-1;x>=num-kill;x--) {
		delete list[x];
		list.erase(list.begin()+x);
	}

	// take the best in the new generation
	for(x=0;x<num-kill && x<newGeneration->getSize();x++) {
		newGeneration->addIndividual(list[x]->ind);
	}

	// clean
	while(list.size()>0) {
		delete list[0];
		list.erase(list.begin());
	}
	list.clear();
}
