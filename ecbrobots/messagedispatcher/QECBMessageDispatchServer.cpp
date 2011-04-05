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
 *   Revision 1.8  2011-04-05 12:16:04  guettler
 *   - new tabWidget
 *   - all found DNS devices are shown in tabWidget with a QDNSDeviceWidget each
 *   - QDNSDeviceWidget shows DNS device name, USB adapter name and type,
 *     response time and incoming/outgoing status (if messages are currently sent
 *     or received)
 *
 *   Revision 1.7  2011/01/24 16:25:16  guettler
 *   -use new QLog feature
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
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "QECBMessageDispatchServer.h"
#include "constants.h"
#include "QMDSHelper.h"
#include "QLog.h"

#include <QHash>

namespace lpzrobots {

  QECBMessageDispatchServer::QECBMessageDispatchServer() :
    QAbstractMessageDispatchServer(), notYetInitialisedCCs(0), notYetDNSScannedCCs(0) {

    QTimer::singleShot(1, this, SLOT(sl_Initialize()));
  }

  QECBMessageDispatchServer::~QECBMessageDispatchServer() {
  }
  void QECBMessageDispatchServer::sl_Initialize() {
    QLogDebug("QMessageDispatchServer initialised.");

    scanUsbDevices();
  }

  void QECBMessageDispatchServer::sl_sendMessage(struct _communicationMessage msg) {
    QLogVerbose("Client demands a message to be sent to " + msg.ecb_dns_name);
    if (this->dnsDeviceMap.contains(msg.ecb_dns_name)) {
      dnsDeviceMap[msg.ecb_dns_name]->channel->sendMessage(msg);
        //dnsDeviceToQCCMap[msg.ecb_dns_name]->get
      emit sig_dataOutgoing(dnsDeviceMap[msg.ecb_dns_name]);
    }
    else
      QLogWarning("Unknown DNS device " + msg.ecb_dns_name + ".");
  }

  void QECBMessageDispatchServer::clear_usbDeviceManagerList() {
    foreach(QCommunicationChannel* commChannel, commChannelList)
      {
        commChannelList.removeOne(commChannel);
        commChannel->close();
        disconnect(commChannel, SIGNAL(sig_cc_initalised(QCommunicationChannel*)));
        disconnect(commChannel, SIGNAL(sig_cc_dns_name_resolved(QCommunicationChannel*)));
        disconnect(commChannel, SIGNAL(sig_messageReceived(struct _communicationMessage)));
        delete (commChannel);
      }
    dnsDeviceStringList.clear();
    dnsDeviceList.clear();
    dnsDeviceMap.clear();
  }

  void QECBMessageDispatchServer::scanUsbDevices() {
    clear_usbDeviceManagerList();
    QStringList usbDeviceNameList = static_usbDeviceManager.getDeviceList();
    QLogVerbose("number USB-Devices found " + QString::number(usbDeviceNameList.length()));
    notYetInitialisedCCs = usbDeviceNameList.length();
    foreach (QString usbDeviceName, usbDeviceNameList)
      {
        QLogVerbose(usbDeviceName);

        QCommunicationChannel* commChannel = new QCommunicationChannel(usbDeviceName);
        commChannelList.append(commChannel);
        connect(commChannel, SIGNAL(sig_cc_initalised(QCommunicationChannel*)), this,
            SLOT(sl_CCIsInitialised(QCommunicationChannel*)));
        connect(commChannel, SIGNAL(sig_cc_dns_name_resolved(QCommunicationChannel*)), this,
            SLOT(sl_scanDNSDevicesComplete(QCommunicationChannel*)));
        connect(commChannel, SIGNAL(sig_messageReceived(struct _communicationMessage)), this,
            SLOT(sl_messageFromQCCReceived(struct _communicationMessage)));
      }
  }

  void QECBMessageDispatchServer::sl_CCIsInitialised(QCommunicationChannel* cc) {
    QLogDebug("QCC[" + cc->getUSBDeviceName() + "] has been initialised, still pending: " + QString::number(
        notYetInitialisedCCs - 1));
    if (--notYetInitialisedCCs == 0) {
      notYetDNSScannedCCs = commChannelList.size();
      QLogVerbose("All QCC initialised.");
      // all QCC are initialised, now they have to get their available DNSDevices.
      foreach (QCommunicationChannel* cc, commChannelList)
        {
          if (!cc->isDeviceInitialised()) {
            notYetDNSScannedCCs--;
            continue; // skip QCC
          }
          QStringList deviceTypeStringList;
          QString deviceTypeString = cc->getCCTypeString();
          if (!deviceTypeStringList.contains(deviceTypeString)) {
            deviceTypeStringList.append(deviceTypeString);
            cc->scanDNSDevices();
            // signaling when ready with sl_scanDNSDevicesComplete(QCommunicationChannel* cc);
          } else
            // wenn doppelt, kein scanDNSDevices aufrufen
            notYetDNSScannedCCs--;
        }
    }
  }

  void QECBMessageDispatchServer::sl_scanDNSDevicesComplete(QCommunicationChannel* cc) {
    QLogDebug("[" + cc->getUSBDeviceName() + "] has completed the device scan, still pending: "
        + QString::number(notYetDNSScannedCCs - 1));


    QList<QCCHelper::DNSDevice_t*> DNSdeviceList = cc->getDNSDeviceList();
    foreach(QCCHelper::DNSDevice_t* dnsDevice, DNSdeviceList)
      {
        dnsDeviceStringList.append(dnsDevice->dns_name);  // for log purposes
        QLogDebug("Found: " + dnsDevice->dns_name);
        // take the fastest one, USART or XBee?
        if (dnsDeviceMap.contains(dnsDevice->dns_name)) {
          if (cc->getResponseTime() < dnsDeviceMap[dnsDevice->dns_name]->channel->getResponseTime()) { // replace!
            dnsDeviceMap.remove(dnsDevice->dns_name);
            dnsDeviceMap[dnsDevice->dns_name] = dnsDevice;
          }
          // else do not replace, skip!
        } else { // not in list yet, insert
          dnsDeviceMap[dnsDevice->dns_name] = dnsDevice;
        }
      }
    if (--notYetDNSScannedCCs == 0) {
      QLogVerbose("<font color=#008800>Scan of DNS devices complete.</font>");
      QMDSHelper::printDNSDeviceMap(&dnsDeviceMap);
    }
    dnsDeviceList = cc->getDNSDeviceList();
    emit sig_DNSDeviceListChanged();
    // in Liste eintragen cc->getDNSDevices():
  }

  void QECBMessageDispatchServer::sl_printDNSDeviceMap() {
    QMDSHelper::printDNSDeviceMap(&dnsDeviceMap);
  }

  void QECBMessageDispatchServer::sl_messageFromQCCReceived(struct _communicationMessage msg) {
    if (dnsDeviceMap.contains(msg.ecb_dns_name)) {
      emit sig_dataIncoming(dnsDeviceMap[msg.ecb_dns_name]);
    }
    emit sig_messageReceived(msg);
  }


} // namespace lpzrobots

