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
#ifndef __BACKCALLER_H_
#define __BACKCALLER_H_

#include <vector>
#include <cstddef>
#include "stl_map.h"

class Callbackable;

/**
 * Class prototype which provides functions to handle callbackable classes.
 * If a class implements this class, just use the function callBack which
 * calls all registered callbackable classes.
 * If you use different callbackable pools, just use the overloaded functions
 * callBack(CallbackableType type),
 * addCallbackable(CallbackableType, Callbackable* cb) and
 * removeCallbackable(CallbackableType, Callbackable* cb).
 */
class BackCaller
{
  public:
    typedef unsigned long CallbackableType;

    /**
     * This is the default Callbackable type.
     * If you derive from BackCaller, just define your own CallbackableTypes.
     */
    static const CallbackableType DEFAULT_CALLBACKABLE_TYPE = 0;

    BackCaller();
    virtual ~BackCaller();

    /**
     * Adds a Callbackable instance to this caller instance.
     * @param type the desired CallbackableType of the Callbackable class.
     * @param callbackableInstance the instance to add
     */
    virtual void addCallbackable(Callbackable* callbackableInstance, CallbackableType type = BackCaller::DEFAULT_CALLBACKABLE_TYPE);

    /**
     * Removes a Callbackable instance from this caller instance.
     * @param type the CallbackableType of the Callbackable class.
     * @param callbackableInstance
     */
    virtual void removeCallbackable(Callbackable* callbackableInstance, CallbackableType type = BackCaller::DEFAULT_CALLBACKABLE_TYPE);

    /**
     * Removes all Callbackable instances from this caller instance
     * @param type the CallbackableType of the Callbackable class to be removed.
     */
    virtual void removeAllCallbackables(CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE */);


    /**
     * Calls all registered callbackable classes of the determined type.
     * This is done by Callbackable::doOnCallback(CallbackableType type).
     * You can make this function private/protected if you like.
     * @param type the CallbackableType of the Callbackable classes.
     */
    virtual void callBack(CallbackableType type = BackCaller::DEFAULT_CALLBACKABLE_TYPE);

    /**
     * Calls all registered callbackable classes of the determined type.
     * This is done by Callbackable::doOnCallback(CallbackableType type).
     * This function uses QUICKMP in order to parallelise the callbacks.
     * Remember that there is only shared the used CallbackableList. So if you
     * have other variables/objects to share, implement your own version.
     * You can make this function private/protected if you like.
     * @param type the CallbackableType of the Callbackable classes.
     */
    virtual void callBackQMP(CallbackableType type = BackCaller::DEFAULT_CALLBACKABLE_TYPE);

  private:
    struct CallbackableTypeHash
    {
      size_t operator() (const CallbackableType& type) const { return type; }
    };

    typedef std::vector<Callbackable*> callbackableListType;
    typedef HashMap<CallbackableType, callbackableListType*, CallbackableTypeHash> callbackableMapType;
    /**
     * This hashmap holds every list of Callbackables for each CallbackableType.
     */
    callbackableMapType callbackableMap;
};

#endif /* __BACKCALLER_H_ */
