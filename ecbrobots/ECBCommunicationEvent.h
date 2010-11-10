/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    wolfgang.rabe@01019freenet.de                                        *
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
 *   Revision 1.1  2010-11-10 09:32:00  guettler
 *   - port to Qt part 1
 *                                                *
 *                                                                         *
 ***************************************************************************/


#ifndef __ECBCOMMUNICATIONEVENT_H_
#define __ECBCOMMUNICATIONEVENT_H_

#include <selforg/mediator.h>
#include <selforg/inspectable.h>
#include "ECBCommunicationData.h"

namespace lpzrobots {


  class ECBCommunicationEvent : public MediatorEvent, public Inspectable
  {
    public:
      enum CommunicationEventType
      {
          EVENT_PACKAGE_SENSORS_RECEIVED, //!< indicates that a package with sensor information was received: ECBCommunicator --> ECB
          EVENT_PACKAGE_DIMENSION_RECEIVED, //!< indicates that a package with dimension (and description info) was received: ECBCommunicator --> ECB
          EVENT_REQUEST_SEND_MOTOR_PACKAGE, //!< indicates that a package with new motor values (or reset command) is requested from ECBCommunicator from a ECB: ECBCommunicator --> ECB
          EVENT_COMMUNICATION_ANSWER_TIMEOUT, //!< indicates that an awaited package was not received: ECBCommunicator --> ECB
          EVENT_REQUEST_SEND_COMMAND_PACKAGE //!< indicates that the ECB has a package to send out (motor, reset, motorstop, beep,...): ECB --> ECBCommunicator
      };

      CommunicationEventType type;
      ECBCommunicationData commPackage;

      ECBCommunicationEvent() {}
      ECBCommunicationEvent(CommunicationEventType type) : type(type) {}
      ECBCommunicationEvent(ECBCommunicationData commPackage) : commPackage(commPackage) {}
      ECBCommunicationEvent(CommunicationEventType type, ECBCommunicationData commPackage) : type(type), commPackage(commPackage) {}
  };

} // namespace lpzrobots

#endif /* __ECBCOMMUNICATIONEVENT_H_ */
