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
#include "mediatorcollegue.h"
#include "mediator.h"
#include <assert.h>

MediatorCollegue::MediatorCollegue(Mediator* myMediator) : myMediator(myMediator)
{
  assert(myMediator!=0);
  myMediator->addMediatorCollegue(this);
}

MediatorCollegue::~MediatorCollegue() {}

void MediatorCollegue::informMediator(MediatorEvent* event)
{
  myMediator->mediatorInformed(this,event);
  delete event;
}




