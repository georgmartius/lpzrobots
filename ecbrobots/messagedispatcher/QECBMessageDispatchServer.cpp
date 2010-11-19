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
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "QECBMessageDispatchServer.h"
#include "constants.h"
#include "QCCHelper.h"
#include "QLog.h"

namespace lpzrobots {

  QECBMessageDispatchServer::QECBMessageDispatchServer() :
    QAbstractMessageDispatchServer(), notYetInitialisedCCs(0) {

    QTimer::singleShot(1, this, SLOT(sl_Initialize()));
  }

  QECBMessageDispatchServer::~QECBMessageDispatchServer() {
  }
  void QECBMessageDispatchServer::sl_Initialize() {
    QLog::logDebug("QMessageDispatchServer initialised.");

    scanUsbDevices();
  }

  void QECBMessageDispatchServer::sl_sendMessage(struct _communicationMessage msg) {
    QLog::logVerbose("Client demands a message to be sent to " + msg.ecb_dns_name);
  }

  void QECBMessageDispatchServer::clear_usbDeviceManagerList() {
    foreach(QCommunicationChannel* commChannel, commChannelList)
      {
        commChannelList.removeOne(commChannel);
        commChannel->close();
        disconnect(commChannel, SIGNAL(sig_cc_initalised()));
        disconnect(commChannel, SIGNAL(sig_cc_dns_name_resolved(QCommunicationChannel*)));
        delete (commChannel);
      }
    dnsDeviceList.clear();
  }

  void QECBMessageDispatchServer::scanUsbDevices() {
    clear_usbDeviceManagerList();
    QStringList usbDeviceNameList = static_usbDeviceManager.getDeviceList();
    QLog::logVerbose("number USB-Devices found " + QString::number(usbDeviceNameList.length()));
    notYetInitialisedCCs = usbDeviceNameList.length();
    foreach (QString usbDeviceName, usbDeviceNameList)
      {
        QLog::logVerbose(usbDeviceName);

        QCommunicationChannel* commChannel = new QCommunicationChannel(usbDeviceName);
        commChannelList.append(commChannel);
        connect(commChannel, SIGNAL(sig_cc_initalised()), this, SLOT(sl_CCIsInitialised()));
        connect(commChannel, SIGNAL(sig_cc_dns_name_resolved(QCommunicationChannel*)), this,
            SLOT(sl_scanDNSDevicesComplete(QCommunicationChannel*)));
      }
  }

  void QECBMessageDispatchServer::sl_CCIsInitialised() {
    QLog::logDebug("Another one QCC has been initialised, still pending: " + QString::number(notYetInitialisedCCs - 1));
    if (--notYetInitialisedCCs == 0) {
      QLog::logDebug("All QCC initialised.");
      // all QCC are initialised, now they have to get their available DNSDevices.
      foreach (QCommunicationChannel* cc, commChannelList)
        {
          if (!cc->isDeviceInitialised())
            continue; // skip QCC
          QStringList deviceTypeStringList;
          QString deviceTypeString = cc->getCCTypeString();
          if (!deviceTypeStringList.contains(deviceTypeString)) {
            deviceTypeStringList.append(deviceTypeString);
            cc->scanDNSDevices();
            // signaling when ready with sl_scanDNSDevicesComplete(QCommunicationChannel* cc);
          }
          // wenn doppelt, kein scanDNSDevices aufrufen
        }
    }
  }

  void QECBMessageDispatchServer::sl_scanDNSDevicesComplete(QCommunicationChannel* cc) {
    QLog::logDebug("Scan of DNS devices complete.");
    QStringList DNSdeviceList = cc->getDNSDeviceList();
    dnsDeviceList.append(DNSdeviceList); // for log purposes
    foreach(QString dnsDevice, DNSdeviceList)
      {
        // take the fastest one, USART or XBee?
        if (dnsDeviceToQCCMap.contains(dnsDevice)) {
          if (cc->getUSBDeviceType() == QCCHelper::USBDevice_USART_ADAPTER) { // replace!
            dnsDeviceToQCCMap.remove(dnsDevice);
            dnsDeviceToQCCMap.insert(dnsDevice, cc);
          }
          // else do not replace!
          // if (dnsDeviceToQCCMap[dnsDevice]->getUSBDeviceType()== QCCHelper::USBDevice_USART_ADAPTER) not needed to check
        } else { // not in list yet
          dnsDeviceToQCCMap.insert(dnsDevice, cc);
        }
      }
    // in Liste eintragen cc->getDNSDevices():
  }

  void QECBMessageDispatchServer::sl_printDNSDeviceToQCCMap() {
    if (dnsDeviceToQCCMap.isEmpty())
      QLog::logWarning("No DNS devices found!");
    foreach(QString dnsDeviceName, dnsDeviceToQCCMap.keys())
      {
        // emit sig_TextLog("DNSDevice = " + dnsName + ", USBDevice = " + dnsDeviceToQCCMap[dnsDevice]->getUSBDeviceName());
        QLog::logVerbose("[" + dnsDeviceName + "]->[" + dnsDeviceToQCCMap[dnsDeviceName]->getUSBDeviceName() + "]");
      }
  }

} // namespace lpzrobots

