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
 *   Revision 1.7  2011-04-05 12:16:04  guettler
 *   - new tabWidget
 *   - all found DNS devices are shown in tabWidget with a QDNSDeviceWidget each
 *   - QDNSDeviceWidget shows DNS device name, USB adapter name and type,
 *     response time and incoming/outgoing status (if messages are currently sent
 *     or received)
 *
 *   Revision 1.6  2010/11/26 12:22:36  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *   Revision 1.5  2010/11/23 11:08:06  guettler
 *   - some helper functions
 *   - bugfixes
 *   - better event handling
 *
 *   Revision 1.4  2010/11/19 15:15:00  guettler
 *   - new QLog feature
 *   - bugfixes
 *   - FT232Manager is now in lpzrobots namespace
 *   - some cleanups
 *
 *   Revision 1.3  2010/11/18 16:58:18  wrabe
 *   - current state of work
 *
 *   Revision 1.2  2010/11/14 20:39:37  wrabe
 *   - save current developent state
 *
 *   Revision 1.1  2010/11/11 15:35:59  wrabe
 *   -current development state of QMessageDispatchServer
 *   -introduction of QCommunicationChannels and QCCHelper
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __QCOMMUNICATIONCHANNEL_H_
#define __QCOMMUNICATIONCHANNEL_H_
#include <QObject>
#include "types.h"
#include "constants.h"
#include "QFT232DeviceManager.h"
#include "QCCHelper.h"
#include "QExtTimer.h"
#include "QECBMessageDispatchServer.h"

template<class Key, class T> class QHash;

namespace lpzrobots {

  class QCommunicationChannel : public QObject {

    Q_OBJECT

    public:
      QCommunicationChannel(QString usbDeviceName);
      virtual ~QCommunicationChannel();

      void close();
      QString getCCTypeString();
      void scanDNSDevices();
      QStringList getDNSDeviceStringList();
      QList<QCCHelper::DNSDevice_t*> getDNSDeviceList() { return dnsDeviceList; }
      bool isDeviceInitialised();
      QCCHelper::usbDeviceType_t getUSBDeviceType();
      QString getUSBDeviceName();
      uint getResponseTime();
      void sendMessage(struct _communicationMessage& msg);

    protected:

    signals:
      void sig_cc_initalised(QCommunicationChannel* cc);
      void sig_cc_dns_name_resolved(QCommunicationChannel* cc);
      void sig_messageReceived(struct _communicationMessage message);

    private slots:
      void sl_ResponseTimerExpired(uint eventId);
      void sl_messageReceived(QByteArray received_msg);
      void sl_XBee_ReadDnsNames_Delayed();
      void sl_switchResetOff();
      void sl_DNSScanTimeout();


    private:

      void openUsbDevice(QString usbDeviceName);

      void sleep(ulong msecs);
      void clearXbeeRemoteList();

      void send_XBeeCommand(QByteArray command);
      void send_XBeeRemoteCommand(QByteArray command, struct QCCHelper::XBeeRemoteNode_t* node);
      void send_ECB_Reset(struct QCCHelper::XBeeRemoteNode_t* node);

      void printMessage(QString s, QByteArray data);
      void clear_usbDeviceManagerList();

      void dispatch_isp(QByteArray received_msg);
      void dispatch_usart(QByteArray received_msg);
      void dispatch_xbee(QByteArray received_msg);
      void dispatch_xbee_command(QByteArray received_command);

      QFT232DeviceManager usbDeviceManager;
      QExtTimer responseTimer;

      QByteArray transmitBuffer;
      QByteArray temporaryFlashBuffer;
      QByte transmitBufferCheckSum;
      QWord ECB_OperationRetriesMax;
      QWord ECB_OperationRetries;
      QCCHelper::typeInitialisedState initialisedState;
      QList<QCCHelper::DNSDevice_t*> dnsDeviceList;
      QCCHelper::usbDeviceType_t usbDeviceType;


      struct QCCHelper::XBeeLocalNode_t xbee;
      QList<QCCHelper::XBeeRemoteNode_t*> xbeeRemoteNodeList;

      struct QCCHelper::XBeeRemoteNode_t* resetted_xbee;

  };

}

#endif /* __QCOMMUNICATIONCHANNEL_H_ */
