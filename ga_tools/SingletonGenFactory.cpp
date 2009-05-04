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
 *   Revision 1.1  2009-05-04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.3  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.2  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "SingletonGenFactory.h"

#include "Gen.h"
#include "GenContext.h"
#include "GenPrototype.h"
#include "Individual.h"
#include "IValue.h"
#include "SingletonGenEngine.h"

int SingletonGenFactory::m_number = 0;
SingletonGenFactory* SingletonGenFactory::m_factory = 0;

SingletonGenFactory::SingletonGenFactory() {
	// nothing
}

SingletonGenFactory::~SingletonGenFactory() {
	// nothing
}

Gen* SingletonGenFactory::createGen(GenContext* context, Individual* individual, GenPrototype* prototype)const {
	Gen* gen = new Gen(prototype, m_number);
	m_number++;
	IValue* value = prototype->getRandomValue();

	gen->setValue(value);

	context->addGen(gen);
	individual->addGen(gen);
	SingletonGenEngine::getInstance()->addGen(gen);

	return gen;
}

Gen* SingletonGenFactory::createGen(GenContext* context, Individual* individual, GenPrototype* prototype, GenContext* oldContext, Individual* oldIndividual, Gen* oldGen, bool mutate)const {
	if(mutate) {
		return prototype->mutate(oldGen,oldContext,oldIndividual);
	}


	Gen* gen = new Gen(prototype,m_number);

	m_number++;

	IValue* value = oldGen->getValue();

	gen->setValue(value);

	context->addGen(gen);
	individual->addGen(gen);
	SingletonGenEngine::getInstance()->addGen(gen);

	return gen;
}

Gen* SingletonGenFactory::createGen(GenContext* context, Individual* individual, GenPrototype* prototype, IValue* value) {
	Gen* gen = new Gen(prototype,m_number);

	m_number++;

	gen->setValue(value);

	context->addGen(gen);
	individual->addGen(gen);
	SingletonGenEngine::getInstance()->addGen(gen);

	return gen;
}
