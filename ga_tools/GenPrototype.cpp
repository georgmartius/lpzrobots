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
 *   This class is used for group some gens and is needed from the         *
 *   genFactory. It saves all GenContexte which are use this prototype.    *
 *   The group of gens becomes whit it an name.                            *
 *                                                                         *
 *   The prototypes inside the gen. alg. are saved in the GenContext, in   *
 *   the Gen and in the GenEngine (only here can be deleted!!!).           *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-05-14 15:29:54  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.2  2009/05/07 14:47:46  robot12
 *   some comments
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.1  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "GenPrototype.h"

#include "GenContext.h"
#include "SingletonGenFactory.h"

GenPrototype::GenPrototype() {
	// nothing
}

GenPrototype::GenPrototype(std::string name, IRandomStrategy* randomStrategy, IMutationStrategy* mutationStrategy) {
	m_name = name;
	m_randomStrategy = randomStrategy;
	m_mutationStrategy = mutationStrategy;
}

GenPrototype::~GenPrototype() {
	std::map<Generation*,GenContext*>::iterator itr = m_context.begin();

	// delete all contexte
	while(itr!=m_context.end()) {
		delete itr->second;
		m_context.erase(itr);
		itr=m_context.begin();
	}

	delete m_randomStrategy;
	delete m_mutationStrategy;
}

void GenPrototype::insertContext(Generation* generation, GenContext* context) {
	m_context[generation]=context;
}

GenContext* GenPrototype::getContext(Generation* generation) {
	return m_context[generation];
}

Gen* GenPrototype::mutate(GenContext* context, Individual* individual, Gen* oldGen, GenContext* oldContext)const {
	return m_mutationStrategy->mutate(context, individual, oldGen, oldContext, SingletonGenFactory::getInstance());
}

int GenPrototype::getMutationProbability(void)const {
	return m_mutationStrategy->getMutationProbability();
}
