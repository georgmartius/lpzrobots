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
 *   Revision 1.6  2011/01/24 16:58:25  guettler
 *   - QMessageDispatchServer is now informed when client app closes itself
 *   - QMessageDispatchWindow actually closes if client app closes itself
 *   - hint: this should late be
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
 *                                            *
 *                                                                         *
 ***************************************************************************/

#ifndef QECBMESSAGEDISPATCHSERVER_H_
#define QECBMESSAGEDISPATCHSERVER_H_
#include <QObject>
#include <QList>
#include "types.h"
#include "constants.h"
#include "QFT232DeviceManager.h"
#include "QAbstractMessageClient.h"
#include "QAbstractMessageDispatchServer.h"
#include "QCommunicationChannel.h"


template<class Key, class T> class QHash;

namespace lpzrobots {

  class QECBMessageDispatchServer : public QAbstractMessageDispatchServer {

    Q_OBJECT

    public:

      QECBMessageDispatchServer();
      virtual ~QECBMessageDispatchServer();

      void scanUsbDevices();
      QList<QCCHelper::DNSDevice_t*> getDNSDeviceList() { return dnsDeviceList; }


    signals:
      void sig_messageReceived(struct _communicationMessage msg);
      void sig_quitServer();
      void sig_quitClient();
      void sig_DNSDeviceListChanged();
      void sig_dataIncoming(QCCHelper::DNSDevice_t* fromDevice);
      void sig_dataOutgoing(QCCHelper::DNSDevice_t* toDevice);


    public slots:
      virtual void sl_sendMessage(struct _communicationMessage msg);
      virtual void sl_quitClient() { emit sig_quitClient(); }

      virtual void sl_Initialize();
      virtual void sl_CCIsInitialised(QCommunicationChannel* cc);
      virtual void sl_scanDNSDevicesComplete(QCommunicationChannel* cc);
      virtual void sl_printDNSDeviceMap();
      virtual void sl_messageFromQCCReceived(struct _communicationMessage msg);

    private:
      void clear_usbDeviceManagerList();

      QList<QCommunicationChannel*> commChannelList;
      QFT232DeviceManager static_usbDeviceManager;

      QStringList dnsDeviceStringList;
      QList<QCCHelper::DNSDevice_t*> dnsDeviceList;
      int notYetInitialisedCCs;
      int notYetDNSScannedCCs;

      /**
       * key = dnsDeviceName
       * value = pointer to corresponding dnsDevice_t&
       */
      QHash<QString, QCCHelper::DNSDevice_t*> dnsDeviceMap;

  };

} // namespace lpzrobots

#endif /* QECBMESSAGEDISPATCHSERVER_H_ */
