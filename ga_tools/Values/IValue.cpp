/***************************************************************************
 *   Copyright (C) 2005-2009 by Robot Group Leipzig                        *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    joergweide84@aol.com (robot12)                                       *
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
 *   This interface is only to describe a value for a Gen. It gives some   *
 *   conditions for the values. The operators add and mul must be declared.*
 *   They must have a name and it must be possible to change the value in a*
 *   string for log or debug outputs.                                      *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.3  2009/06/26 13:08:25  robot12
 *   finishing Values and add some comments
 *
 *   Revision 1.2  2009/05/12 13:29:24  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.1  2009/05/04 15:27:58  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.3  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *   Revision 1.2  2009/04/23 15:17:42  jhoffmann
 *   inserted copyright-template, corrected includes, doxygen style templated
 *
 ***************************************************************************/
#include "IValue.h"

IValue::IValue(std::string name) : m_name(name) {
	// nothing
}

IValue::~IValue() {
	// nothing
}

IValue::operator std::string(void)const {
	return "";
}
