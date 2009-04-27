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

#ifndef SINGLETONGENFACTORY_H_
#define SINGLETONGENFACTORY_H_

#include "types.h"

#include "Gen.h"
#include "GenContext.h"
#include "GenPrototyp.h"

class SingletonGenFactory {
public:
	inline static SingletonGenFactory* getInstance(void) {if(m_factory==0)m_factory = new SingletonGenFactory();return m_factory;}
	inline static void destroyGenFactory(void) {delete m_factory;m_factory=0;}

	// 3 Methodes to create an Gen
	Gen* createGen(GenPrototyp* prototyp)const;												// random
	Gen* createGen(GenPrototyp* prototyp, GenContext* oldContext, Gen* oldGen, bool mutate=false)const;				// copy + mutation

private:
	static SingletonGenFactory* m_factory = 0;
	static int m_number = 0;

	/**
	 * disable the default constructor
	 */
	SingletonGenFactory();

	/**
	 * disable destructor
	 */
	virtual ~SingletonGenFactory();
};

#endif /* SINGLETONGENFACTORY_H_ */
