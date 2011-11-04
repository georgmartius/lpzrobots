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
#ifndef __MEDIATOR_H_
#define __MEDIATOR_H_


#include "mediatorcollegue.h"
#include <vector>

/**
 * The default MediatorEvent holds no information,
 * the implementation of the mediator should implement a
 * derived version of the MediatorEvent.
 */
class MediatorEvent
{
  public:
};

class Mediator
{
  public:
    typedef std::vector<MediatorCollegue*> MediatorCollegueListType;

    Mediator();
    virtual ~Mediator();

    virtual void mediatorInformed(MediatorCollegue* source, MediatorEvent* event) = 0;

    void addMediatorCollegue(MediatorCollegue* collegue);

    void removeMediatorCollegue(MediatorCollegue *collegue);

    void removeAllMediatorCollegues();

    MediatorCollegue* getMediatorCollegue(unsigned int index);

    unsigned int getMediatorCollegueIndex(MediatorCollegue* collegue);

    unsigned int getNumberOfMediatorCollegues();

    void mediate(unsigned int indexOfMediatorCollegue, MediatorEvent* event);

    void mediate(MediatorCollegue* collegue, MediatorEvent* event);

    void mediateToAll(MediatorEvent* event);

    void mediateToAllQMP(MediatorEvent* event);



  protected:


  private:
    MediatorCollegueListType collegueList;
};

#endif /* _MEDIATOR_H_ */
