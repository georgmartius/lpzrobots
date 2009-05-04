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
 *   Revision 1.1  2009-05-04 15:27:56  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.1  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef GENPROTOTYPE_H_
#define GENPROTOTYPE_H_

#include <string>
#include <map>

// forward declarations
class Generation;
class GenContext;
class IValue;
class Gen;

#include "IRandomStrategy.h"
#include "IMutationStrategy.h"

class GenPrototype {
public:
	GenPrototype(std::string name, IRandomStrategy* randomStrategy, IMutationStrategy* mutationStrategy);
	virtual ~GenPrototype();

	inline std::string getName(void)const {return m_name;}
	inline IValue* getRandomValue(void)const {return m_randomStrategy->getRandomValue();}

	void insertContext(Generation* generation, GenContext* context);
	GenContext* getContext(Generation* generation);
	Gen* mutate(Gen* gen, GenContext* context, Individual* individual)const;
	int getMutationProbability(void)const;

protected:
	std::string m_name;
	std::map<Generation*,GenContext*> m_context;
	IRandomStrategy* m_randomStrategy;
	IMutationStrategy* m_mutationStrategy;

private:
	/**
	 * disable the default constructor
	 */
	GenPrototype();
};

#endif /* GENPROTOTYPE_H_ */
