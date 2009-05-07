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
 *   This class is used for representing one gen in the gen. alg.          *
 *   It has one ID which make it individual and a name (string)            *
 *   which group it with other gens to a gen pool.                         *
 *   Also it has a IValue which is used to save the real value.            *
 *   An IValue can be a number, a matrix, a 3D Modell or something else.   *
 *                                                                         *
 *   Places for saving the gen inside the gen. alg. are the GenContext,    *
 *   the Individual and the GenEngine. Deleting only in the GenEngine!     *
 *                                                                         *
 *   $Log$
 *   Revision 1.5  2009-05-07 14:47:47  robot12
 *   some comments
 *
 *   Revision 1.4  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.5  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "Gen.h"

Gen::Gen(void) {
	// nothing
}

Gen::Gen(GenPrototype* prototype, int id) {
	m_prototype = prototype;
	m_value = NULL;
	m_ID = id;
}

Gen::~Gen(void) {
	delete m_value;
	m_value = NULL;
}

std::string Gen::getName(void)const {
	return m_prototype->getName();
}

GenPrototype* Gen::getPrototype(void)const {
	return m_prototype;
}
