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
 *   Revision 1.1  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef GENCONTEXT_H_
#define GENCONTEXT_H_

#include <vector>
#include <algorithm>

class Gen;
class GenPrototype;

class GenContext {
public:
	GenContext(GenPrototype* prototype);
	virtual ~GenContext();

	inline GenPrototype* getPrototype(void)const {return m_prototype;}

	inline void addGen(Gen* gen) {m_storage.push_back(gen);}
	inline void removeGen(Gen* gen) {std::vector<Gen*>::iterator itr = std::find(m_storage.begin(),m_storage.end(),gen); m_storage.erase(itr);}
	inline const std::vector<Gen*>& getGene(void)const {return m_storage;}

protected:
	std::vector<Gen*> m_storage;
	GenPrototype* m_prototype;

private:
	/**
	 * disable default constructor
	 */
	GenContext();
};

#endif /* GENCONTEXT_H_ */
