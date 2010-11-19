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
 *   Revision 1.4  2010-11-19 15:15:00  guettler
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

namespace lpzrobots {

  struct XBeeLocalNode_t {
      XBeeLocalNode_t(){
        type = 0;
        hardwareVersion = 0;
        rf_channel = 0x10000;  // Parameter Range: 0x0B - 0x1A (XBee) , 0x0C - 0x17 (XBee-PRO)
        pan_identifier = 0x10000;         // Parameter Range: 0 - 0xFFFF
      };
      uint16  hardwareVersion;
      uint    type;
      uint    rf_channel;
      uint    pan_identifier;
  };

  struct DNSDevice_t {
    public:
      QString dns_name;
  };

  struct XBeeRemoteNode_t : public DNSDevice_t {
    public:
      XBeeRemoteNode_t(){
        address16 = 0xFFFE;
        address64 = 0x000000000000FFFF;
      };
      QString Identifier;
      uint16  address16;
      uint64  address64;
  };
  
  class QCommunicationChannel : public QObject {

    Q_OBJECT

    public:
      QCommunicationChannel(QString usbDeviceName);
      virtual ~QCommunicationChannel();

      void close();
      QString getCCTypeString();
      void scanDNSDevices();
      QStringList getDNSDeviceList();
      bool isDeviceInitialised();
      QCCHelper::usbDeviceType_t getUSBDeviceType();
      QString getUSBDeviceName();

    protected:
      enum timerEvent_t {
        EVENT_TIMEOUT_GENERAL,
        EVENT_TIMEOUT_INITIALISE,
        EVENT_TIMEOUT_NODEDISCOVER
      };

    signals:
      void sig_cc_initalised();
      void sig_cc_dns_name_resolved(QCommunicationChannel* cc);

    private slots:
      void sl_ResponseTimerExpired(uint eventId);
      void sl_messageReceived(QByteArray received_msg);
      void sl_XBee_ReadDnsNames_Delayed();
      void sl_switchResetOff();


    private:

      void openUsbDevice(QString usbDeviceName);

      void sleep(ulong msecs);
      void clearXbeeRemoteList();

      void send_XBeeCommand(QByteArray command);
      void send_XBeeRemoteCommand(QByteArray command, struct XBeeRemoteNode_t* node);
      void send_ECB_Reset(struct XBeeRemoteNode_t* node);

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
      QList<DNSDevice_t*> dnsDeviceList;
      QCCHelper::usbDeviceType_t usbDeviceType;


      struct XBeeLocalNode_t xbee;
      QList<XBeeRemoteNode_t*> xbeeRemoteNodeList;

      struct XBeeRemoteNode_t* resetted_xbee;

  };

}

#endif /* __QCOMMUNICATIONCHANNEL_H_ */
