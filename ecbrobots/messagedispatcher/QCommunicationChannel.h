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
 *   Revision 1.1  2010-11-11 15:35:59  wrabe
 *   -current development state of QMessageDispatchServer
 *   -introduction of QCommunicationChannels and QCCHelper
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __QCOMMUNICATIONCHANNEL_H_
#define __QCOMMUNICATIONCHANNEL_H_
#include "QECBMessageDispatchServer.h"
#include "QCCHelper.h"

namespace lpzrobots {

  class QExtTimer : public QTimer {

    Q_OBJECT

    public:
      QExtTimer() :
        QTimer() {
        connect(this, SIGNAL(timeout()), this, SLOT(sl_timeout()));
      }

      virtual ~QExtTimer() {
      }

      virtual void start(int msec, uint eventId) {
        this->eventId = eventId;
        QTimer::start(msec);
      }
      virtual void start(uint eventId) {
        this->eventId = eventId;
        QTimer::start();
      }

    public slots:
      virtual void sl_timeout() {
        emit timeout(eventId);
      }

    signals:
      void timeout(uint eventId);

    private:
      uint eventId;

  };
  
  class QCommunicationChannel : public QObject {

    Q_OBJECT

    public:
      QCommunicationChannel();
      virtual ~QCommunicationChannel();

      virtual QStringList getUsbDeviceList();

    signals:

    private slots:
      void sl_ResponseTimerExpired(uint eventId);

    private:

      void send_XBeeATND();
      void send_XBeeCommand(QByteArray command);
      void send_XBeeRemoteCommand(QByteArray command);
      void send_ECB_Reset();
      void dispatch_XbeeCommandResponse(QByteArray receiveBuffer);


      QFT232DeviceManager usbDeviceManager;
      QExtTimer responseTimer;

  };

}

#endif /* __QCOMMUNICATIONCHANNEL_H_ */
