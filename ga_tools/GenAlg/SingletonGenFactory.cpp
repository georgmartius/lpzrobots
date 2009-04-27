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
 *   Revision 1.2  2009-04-27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "SingletonGenFactory.h"

SingletonGenFactory::SingletonGenFactory() {
	// nothing
}

SingletonGenFactory::~SingletonGenFactory() {
	// nothing
}

Gen* SingletonGenFactory::createGen(GenPrototyp* prototyp)const {
	Gen* gen = new Gen(prototyp->getName(), m_number);
	m_number++;
	IValue* value = prototyp->getRandomValue();

	gen->setValue(value);

	return gen;
}

Gen* SingletonGenFactory::createGen(GenPrototyp* prototyp, GenContext* oldContext, Gen* oldGen, bool mutate)const {
	Gen* gen = new Gen(prototyp->getName(),m_number);

	m_number++;

	if(mutate) {
		IValue* oldValue = oldGen->getValue();
		IValue* mut = oldContext->getMutationFactor();
		IValue* value = (*oldValue)*(*mut);

		gen->setValue(value);
	}
	else {
		IValue* value = oldGen->getValue();

		gen->setValue(value);
	}

	return gen;
}
