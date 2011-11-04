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

    MediatorCollegue(Mediator* myMediator = 0);
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

  protected:

    /**
     * Sets the own mediator. Useful if mediator is unknown in initialization.
     * @param myMediator
     */
    void setMediator(Mediator* myMediator);


  private:

    Mediator* myMediator;
};

#endif /* _MEDIATORCOLLEGUE_H_ */
