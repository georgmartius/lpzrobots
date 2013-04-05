/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 *                                                                         *
 ***************************************************************************/

#ifndef INSPECTABLEPROXY_H_
#define INSPECTABLEPROXY_H_

//includes
#include <list>
#include "inspectable.h"

/**
 * This class is a proxy for the inspectable class. It "divides" the inspectable interface  from the inspected object
 */
class InspectableProxy : public Inspectable {
public:
        /**
         * constructor
         * Takes a list of Inspectables for which the proxy stands.
         * Attention: The inspectables must have registered
         * all their variables already!
         * @param list (list<Inspectable*>&) the list
         */
        InspectableProxy(const std::list<Inspectable*>& list, const iparamkey& name = "InspectableProxy");

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
        InspectableProxy(const iparamkey& name = "InspectableProxy");
};

#endif /* INSPECTABLEPROXY_H_ */
