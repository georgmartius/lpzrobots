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

#ifndef SINGLETONGENFACTORY_H_
#define SINGLETONGENFACTORY_H_

class GenContext;
class Gen;
class Individual;
class GenPrototype;
class IValue;

class SingletonGenFactory {
public:
	inline static SingletonGenFactory* getInstance(void) {if(m_factory==0)m_factory = new SingletonGenFactory();return m_factory;}
	inline static void destroyGenFactory(void) {delete m_factory;m_factory=0;}

	// 3 methodes to create an Gen
	Gen* createGen(GenContext* context, Individual* individual, GenPrototype* prototype)const;																			// random
	Gen* createGen(GenContext* context, Individual* individual, GenPrototype* prototype, GenContext* oldContext, Individual* oldIndividual, Gen* oldGen, bool mutate=false)const;				// copy + mutation
	Gen* createGen(GenContext* context, Individual* individual, GenPrototype* prototype, IValue* value);																	// value

private:
	static SingletonGenFactory* m_factory;
	static int m_number;

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
