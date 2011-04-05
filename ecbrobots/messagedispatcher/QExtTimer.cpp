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
 *   Revision 1.2  2011-04-05 12:16:04  guettler
 *   - new tabWidget
 *   - all found DNS devices are shown in tabWidget with a QDNSDeviceWidget each
 *   - QDNSDeviceWidget shows DNS device name, USB adapter name and type,
 *     response time and incoming/outgoing status (if messages are currently sent
 *     or received)
 *
 *   Revision 1.1  2010/11/18 16:58:18  wrabe
 *   - current state of work
 *
 *                                                                         *
 ***************************************************************************/

#include "QExtTimer.h"

namespace lpzrobots {
  
  QExtTimer::QExtTimer() :
    QTimer(), eventId(0), timeRan(0) {
    connect(this, SIGNAL(timeout()), this, SLOT(sl_timeout()));
    connect(&tickTimer, SIGNAL(timeout()), this, SLOT(sl_tickTimeout()));
    tickTimer.setInterval(1);
    tickTimer.setSingleShot(false);
  }

  QExtTimer::~QExtTimer() {}


  void QExtTimer::start(int msec, uint eventId) {
    this->eventId = eventId;
    timeRan = 0;
    QTimer::start(msec);
    tickTimer.start();
  }
  
  void QExtTimer::start(uint eventId) {
    this->eventId = eventId;
    timeRan = 0;
    QTimer::start();
    tickTimer.start();
  }

  unsigned int QExtTimer::stop() {
    QTimer::stop();
    tickTimer.stop();
    return timeRan;
  }


  void QExtTimer::sl_timeout() {
    emit timeout(eventId);
  }

  void QExtTimer::sl_tickTimeout() {
    timeRan++;
  }

  unsigned int QExtTimer::getTimeRan() {
    return timeRan;
  }

}
