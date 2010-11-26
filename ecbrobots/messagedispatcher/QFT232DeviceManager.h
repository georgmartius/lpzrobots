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
 *  Diese Klasse stellt den Zugriff zu einer seriellen Schnittstelle       *
 *  bereit. Alle verfügbaren (nicht verwendeten) seriellen Verbindungen    *
 *  können abgefragt werden. Eingehende Daten werden mit Hilfe eines       *
 *  eigenständigen Threads verarbeitet.                                    *
 *  Die Klasse sendet drei Signale: ein Signal über das Öffnen des Ports,  *
 *  ein signal über Status/Fehler-Mitteilungen und ein Signal über den     *
 *  Empfang neuer Daten vom Port                                           *
 *                                                                         *
 *   $Log$
 *   Revision 1.6  2010-11-26 12:22:36  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
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
 *                                       *
 *                                                                         *
 ***************************************************************************/

#ifndef QFT232DEVICEMANAGER_H_
#define QFT232DEVICEMANAGER_H_

#include <QtGui>
#include <QThread>
#include <QString>
#include <ftdi.h>
#include "constants.h"
#include "types.h"

namespace lpzrobots {

  class QFT232DeviceManager : public QThread {
    Q_OBJECT

    public:
      QFT232DeviceManager();
      virtual ~QFT232DeviceManager();
      virtual void run();

      void createDeviceList();
      QStringList getDeviceList();
      int openDevice(struct usb_device* usb_dev, int baudrate);
      int openDeviceByName(QString usb_deviceName_to_open, int baudrate_to_use);
      int setBaudrate(int baudrate_to_set);
      int setLatencyTimer(int latency_to_set);
      int setDTR(int dtr_val);
      int closeDevice();

      bool isDeviceAvailable(QString deviceName);
      bool isDeviceOpened() {
        return opened;
      }
      int getBaudrate() {
        return baudrate;
      }
      ;
      QString getDeviceName() {
        return deviceName;
      }
      ;
      int writeData(QByteArray msg);

      /*
       void setDeviceName(QString name){ deviceName = name.toLatin1(); };
       QString getDeviceName() {return deviceName;};
       */

    signals:
      void sig_newData(QByteArray msg);
      void sig_DeviceOpened();

    private:

      QString deviceName;
      unsigned int baudrate;
      QByteArray receiveBuffer;
      volatile bool opened;
      volatile bool runListener;

      struct ftdi_context ftdic;
      struct ftdi_device_list* devlist;
      struct usb_device* usb_device_opened;

  };

} // namespace lpzrobots

#endif /* SERIALCOMMUNICATION_H_ */
