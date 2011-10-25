/***************************************************************************
 *   Copyright (C) 2011 by                                                 *
 *   Research Network for Self-Organization of Robot Behavior              *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    wrabe@informatik.uni-leipzig.de                                      *
 *    Georg.Martius@mis.mpg.de                                             *
 *    ralfder@mis.mpg.de                                                   *
 *    frank@nld.ds.mpg.de                                                  *
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
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2011-10-25 12:30:08  guettler
 *   list which handles some callback stuff (so far: list modified and list being deleted).
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __CONFIGURABLELIST_H_
#define __CONFIGURABLELIST_H_

#include <vector>
#include "configurable.h"

/**
 * Establishes for some methods the notifications for registered Callbackable instances
 * (use addCallbackable(...)).
 * @warning Only three methods are currently supported: push_back(...), pop_back() and clear()!
 * You can use iterators with the limitation that no deletion or insertion process is done with them.
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
    void pop_back();
    void clear();
};

#endif /* __CONFIGURABLELIST_H_ */
