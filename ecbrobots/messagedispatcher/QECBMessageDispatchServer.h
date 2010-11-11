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
 *                                            *
 *                                                                         *
 ***************************************************************************/

#ifndef QECBMESSAGEDISPATCHSERVER_H_
#define QECBMESSAGEDISPATCHSERVER_H_
#include <QObject>
#include <QList>
#include <QHash>
#include "types.h"
#include "constants.h"
#include "QFT232DeviceManager.h"
#include "QAbstractMessageClient.h"
#include "QAbstractMessageDispatchServer.h"

namespace lpzrobots {

  class QECBMessageDispatchServer : public QAbstractMessageDispatchServer {

    Q_OBJECT

    public:

      QECBMessageDispatchServer();
      virtual ~QECBMessageDispatchServer();

      void scanUsbDevices();

    signals:
      virtual void sig_messageReceived(struct _communicationMessage msg);
      virtual void sig_TextLog(QString sText);
      virtual void sig_quitServer();


    public slots:
      virtual void sl_sendMessage(struct _communicationMessage msg);
      virtual void sl_receiveMessageFromUsbDeviceManager(QString usbDeviceName, QByteArray received_msg);

    private:
      bool transmit(QFT232DeviceManager *ft232manager);
      void send_Message(QFT232DeviceManager *ft232manager, QByteArray command);
      void send_ECB_Reset();
      void send_XBeeCommand(QByteArray command);
      void send_XBeeRemoteCommand(QByte command[], int length);
      void send_XBeeATND();
      void printBuffer(QByteArray buffer);
      void dispatch_XbeeCommandResponse(QByteArray receiveBuffer);
      QList<QFT232DeviceManager*> managerList;

      QTimer *timer;

      QByteArray transmitBuffer;
      QByteArray temporaryFlashBuffer;
      QByte transmitBufferCheckSum;
      QWord USBDeviceXBeeType;
      QWord ECB_OperationRetriesMax;
      QWord ECB_OperationRetries;
      quint16 ECB_XBeeAddress16;
      quint64 ECB_XBeeAddress64;
      int timerParams;
      int applicationMode;

      enum packageFormatType_t {
        TYPE_CABLE,
        TYPE_XBEE_SERIE1,
        TYPE_XBEE_SERIE2
      };

      struct usbDeviceTableItem_t
      {
          QString dnsName;
          QString usbDeviceName;
          QString xbeeNodeIdentifier;
          QFT232DeviceManager *deviceManager;
          packageFormatType_t packageFormatType;
          uint8 xbeeAddress16[2];
          uint8 xbeeAddress64[8];
      };

      /**
       * key = dnsName
       * value = table containing dnsName, usbDeviceName, pointer to devManager, formatType and xbeeAddresses
       */
      QHash<QString, usbDeviceTableItem_t> usbDeviceTable;
  };

} // namespace lpzrobots

#endif /* QECBMESSAGEDISPATCHSERVER_H_ */
