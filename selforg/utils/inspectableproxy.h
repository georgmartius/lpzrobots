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
 *   This class adds functionality to the inspectable interface. So it is  *
 *   possible to change the inspectable object to become the next value.   *
 *                                                                         *
 *   The values of the inspectable will not longer be generate by only one *
 *   class.                                                                *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-08-05 13:22:21  robot12
 *   add one clean up, the function replaceList now modified the old list and don't delete all and make it new
 *
 *   Revision 1.2  2009/07/21 08:47:33  robot12
 *   add some comments
 *
 *   Revision 1.1  2009/06/29 13:37:05  robot12
 *   add the new class inspectableproxy
 *
 *
 ***************************************************************************/

#ifndef INSPECTABLEPROXY_H_
#define INSPECTABLEPROXY_H_

//includes
#include <list>
#include "inspectable.h"

/**
 * This class is a proxy for the inspectable class. It "divide" the inspectable interface  from the inspected object
 */
class InspectableProxy : public Inspectable {
public:
	/**
	 * constructor
	 * Take a list of Inspectables for which the proxy stand.
	 * @param list (list<Inspectable*>&) the list
	 */
	InspectableProxy(const std::list<Inspectable*>& list);

	/**
	 * default destructor
	 */
	virtual ~InspectableProxy();

	/**
	 * this function is for replacing the old list
	 * @param list (list<Inspectable*>& the new list
	 * @return (bool) true if it successful
	 */
	bool replaceList(const std::list<Inspectable*>& list);

private:
	/**
	 * the saved list of the Inspectable where the proxy stand for.
	 */
	//std::list<Inspectable*> m_list;

	/**
	 * disable the default constructor
	 * @return
	 */
	InspectableProxy();
};

#endif /* INSPECTABLEPROXY_H_ */
