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
 *   Revision 1.1  2009-04-27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef GENPROTOTYP_H_
#define GENPROTOTYP_H_

#include "types.h"

#include <string>
#include <map>

#include "Generation.h"
#include "GenContext.h"
#include "IRandomStrategie.h"
#include "IValue.h"

class GenPrototyp {
public:
	GenPrototyp(std::string name, IRandomStrategie* randomStrategie);
	virtual ~GenPrototyp();

	inline std::string getName(void)const {return m_name;}
	inline void insertContext(Generation* generation,GenContext* context) {m_context[generation]=context;}
	inline GenContext* getContext(Generation* generation)const {return m_context[generation];}
	inline IValue* getRandomValue(void)const {return m_strategie->getRandomValue();}

protected:
	std::string m_name;
	std::map<Generation*,GenContext*> m_context;
	IRandomStrategie* m_strategie;

private:
	/**
	 * disable the default constructor
	 */
	GenPrototyp();
};

#endif /* GENPROTOTYP_H_ */
