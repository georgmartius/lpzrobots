/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *  
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.1  2009-08-10 07:34:49  guettler
 *  -Base classes which support use of design pattern
 *   mediator - similar to callbackable, but with more functionality:
 *   The mediator takes a more central role in mediation, the collegues
 *   are able to inform the mediator that something happend with
 *   easy use of mediated events (MediatorEvent).
 *										   *
 *                                                                         *
 **************************************************************************/
#include "mediator.h"
#include <assert.h>
#include "stl_adds.h"
//#include "quickmp.h"

Mediator::Mediator() {}

Mediator::~Mediator() {}

void Mediator::removeAllMediatorCollegues()
{
  collegueList.clear();
}



void Mediator::addMediatorCollegue(MediatorCollegue *collegue)
{
  collegueList.push_back(collegue);
}

MediatorCollegue* Mediator::getMediatorCollegue(int index)
{
  assert(index < getNumberOfMediatorCollegues());
  return collegueList[index];
}

unsigned int Mediator::getMediatorCollegueIndex(MediatorCollegue* collegue)
{
  for (unsigned int index=0; index<collegueList.size(); index++)
    if (collegueList[index]==collegue)
      return index;
  return -1;
}

unsigned int Mediator::getNumberOfMediatorCollegues()
{
  return (unsigned int) collegueList.size();
}

void Mediator::mediate(int indexOfMediatorCollegue, MediatorEvent* event)
{
  if (indexOfMediatorCollegue<getNumberOfMediatorCollegues())
    getMediatorCollegue(indexOfMediatorCollegue)->doOnMediatorCallBack(event);
  delete event;
}

void Mediator::mediate(MediatorCollegue* collegue, MediatorEvent* event)
{
  collegue->doOnMediatorCallBack(event);
  delete event;
}

void Mediator::mediateToAll(MediatorEvent* event)
{
  FOREACHC(MediatorCollegueListType, collegueList, collegue)
  {
    (*collegue)->doOnMediatorCallBack(event);
  }
  delete event;
}

void Mediator::mediateToAllQMP(MediatorEvent* event)
{
  mediateToAll(event);
  /*
  unsigned int listSize = collegueList.size();
  if(listSize==1){
    list.front()->doOnMediatorCallBack(event);
  }else if (listSize>1){
    QMP_SHARE(collegueList);
    QMP_SHARE(event);
    QMP_PARALLEL_FOR(i, 0, listSize){
      QMP_USE_SHARED(collegueList, MediatorCollegueListType);
      QMP_USE_SHARED(event, MediatorEvent);
      collegueList[i]->doOnMediatorCallBack(event);
    }
    QMP_END_PARALLEL_FOR;
  }
  delete event;*/
}
