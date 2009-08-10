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
#ifndef __MEDIATORCOLLEGUE_H_
#define __MEDIATORCOLLEGUE_H_

//#include "mediator.h"

class Mediator;
class MediatorEvent;

class MediatorCollegue
{
  public:
    typedef unsigned long InformMediatorType;

    static const InformMediatorType DEFAULT_INFORM_MEDIATOR_TYPE = 0;

    MediatorCollegue(Mediator* myMediator);
    virtual ~MediatorCollegue();

    /**
     * Calls the mediator that this collegue has performed something.
     * The event and the instance of this class is handed over.
     */
    void informMediator(MediatorEvent* event);

    /**
     * Is called when the mediator informs this collegue that an event
     * has to be performed by this collegue instance.
     */
    virtual void doOnMediatorCallBack(MediatorEvent* event) = 0;

  private:

    Mediator* myMediator;
};

#endif /* _MEDIATORCOLLEGUE_H_ */
