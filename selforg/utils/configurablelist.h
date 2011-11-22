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

#ifndef __CONFIGURABLELIST_H_
#define __CONFIGURABLELIST_H_

#include <vector>
#include "configurable.h"

/**
 * Establishes for some methods the notifications for registered Callbackable instances
 * (use addCallbackable(...)).
 * @warning Only the following methods are currently supported: 
    push_back(...), pop_back(), erase() and clear()!
 * You can use iterators with the limitation to not delete or insert.
 */
class ConfigurableList : public std::vector<Configurable*>, public BackCaller {
  public:
    ConfigurableList();
    virtual ~ConfigurableList();

    /**
     * Indicates that the list has been modified, a Configurable instance was either added or removed.
     */
    static const CallbackableType CALLBACK_CONFIGURABLE_LIST_MODIFIED = 3;

    /**
     * Indicates that the list is being deleted.
     */
    static const CallbackableType CALLBACK_CONFIGURABLE_LIST_BEING_DELETED = 4;

    void push_back(Configurable* const & configurable);
    iterator erase(iterator pos);
    void pop_back();
    void clear();
};

#endif /* __CONFIGURABLELIST_H_ */
