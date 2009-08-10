/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *  Uses design pattern mediator:                                          *
 *  BackCaller - subject and changemanager (combined)              *
 *  Callbackable       - observer                                          *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-08-10 07:31:04  guettler
 *   -new BackCaller class to provide common
 *     functions used for callback
 *   -Callbackable interface modified
 *   -callBack now supports different types
 *     of CallBackable types
 *
 *   Revision 1.1  2007/05/07 21:04:44  robot3
 *   added interface class, which provides the possibility, that an implementing class
 *   can have a callback from another class where the implementing class is registered
 *   for the callback.
 *
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
