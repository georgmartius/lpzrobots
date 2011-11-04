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

#include "backcaller.h"
#include "callbackable.h"
#include <algorithm>
#include <selforg/stl_adds.h>
#include "quickmp.h"

BackCaller::BackCaller() {}

BackCaller::~BackCaller()
{
  // remove all lists in callbackableMap
  FOREACH(callbackableMapType, callbackableMap, mapItr)
  {
    delete (*mapItr).second;
  }
  callbackableMap.clear();
}

void BackCaller::addCallbackable(Callbackable *callbackableInstance, CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE*/)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else
  { // not found, create new one
    list = new callbackableListType();
    callbackableMap[type] = list;
  }
  // check if callbackableInstance is already registered
  callbackableListType::const_iterator listItr = find(list->begin(), list->end(), callbackableInstance);
  if (listItr==list->end())
    list->push_back(callbackableInstance);
}


void BackCaller::removeCallbackable(Callbackable *callbackableInstance, CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE*/)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else // not found, ignore
    return;
  callbackableListType::iterator listItr = find(list->begin(), list->end(), callbackableInstance);
  if (listItr!=list->end())
    list->erase(listItr);
}

void BackCaller::removeAllCallbackables(CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE */)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else // not found, ignore
    return;
  list->clear();
}


void BackCaller::callBack(CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE*/)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else // not found, ignore
    return;
  FOREACHC(callbackableListType, *list, listItr)
  {
    (*listItr)->doOnCallBack(this, type);
  }
}


void BackCaller::callBackQMP(CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE*/)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else // not found, ignore
    return;
  unsigned int listSize = list->size();
  if(listSize==1){
    list->front()->doOnCallBack(this, type);
  }else if (listSize>1){
    callbackableListType derefList = *list; // use dereferenced version to avoid parallel accesses to the pointer (but i guess that the compiler optimises it already)
    BackCaller thisCaller = *this;
    QMP_SHARE(derefList);
    QMP_SHARE(type);
    QMP_SHARE(thisCaller);
    QMP_PARALLEL_FOR(i, 0, listSize){
      QMP_USE_SHARED(derefList, callbackableListType);
      QMP_USE_SHARED(type, CallbackableType);
      QMP_USE_SHARED(thisCaller, BackCaller);
      derefList[i]->doOnCallBack(&thisCaller, type);
    }
    QMP_END_PARALLEL_FOR;
  }
}


