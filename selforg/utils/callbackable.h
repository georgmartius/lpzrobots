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
#ifndef _CALLBACKABLE_H
#define _CALLBACKABLE_H

#include "backcaller.h"

/**
 * Interface class for a class which wants to be callback on a certain action.
 * In lpzRobots this should be the most case when the time loop is going to the
 * next step.
 *
 * NEW since 20090731:
 * Use the class BackCaller to get already implemented functions like addCallbackable(...).
 *
 * @see BackCaller
 */
class Callbackable
{
  public:

    Callbackable() {};

    virtual ~Callbackable() {}

    /**
     * This method is invoked when a callback is done from the class where this
     * class is for callback registered
     * @param source the caller instance which did the callback.
     * @param type this type can be used to differ from varying types of callback.
     * @see BackCaller
     */
    virtual void doOnCallBack(BackCaller* source, BackCaller::CallbackableType type = BackCaller::DEFAULT_CALLBACKABLE_TYPE) = 0;


};

#endif
