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
 *   Revision 1.2  2010-11-09 17:56:55  wrabe
 *   - change of the communication protocoll between lupae and usb-isp-adapter
 *   - therefore recoding the dedicated methods
 *   - reduction of the overloded send_Message methods to one method only
 *   - insertion of QExtActions to join all events of menu-buttons as well of quickstart-buttons
 *   - adding two new functions to read out and write into the eeprom-space of the atmega128 at an ecb
 *   - change of the fontSize in the hexViewer, change of the total-width of the window
 *                                              *
 *                                                                         *
 ***************************************************************************/

#ifndef QPANELSETTING_H_
#define QPANELSETTING_H_

#include <QtGui>
#include <QWidget>
#include "avrDeviceList.h"
#include "constants.h"
#include "types.h"

namespace lpzrobots {

  class QPanelSetting : public QWidget {
  Q_OBJECT

  public:
    QPanelSetting();
    virtual ~QPanelSetting();

    void setMode(int mode);

    void setUSBDeviceNames(QStringList list);
    void setUSBDeviceBaudrates(QStringList list);
    void setAVRDeviceAccessSpeeds(QStringList list);
    void setUSBDeviceXBeeTypes(QStringList list);
    void setXBeeRemoteNodeIdentifiers(QStringList list);

    QString getUSBDeviceName();
    QString getUSBDeviceBaudrate();
    QString getUSBDeviceXBeeType();
    QString getAVRDeviceAccessSpeed();
    QString getXBeeRemoteNodeIdentifier();

    void setUSBDeviceName(QString name);
    void setUSBDeviceBaudrate(QString name);
    void setAVRDeviceAccessSpeed(QString name);
    void setUSBDeviceXBeeType(QString name);
    void setXBeeRemoteNodeIdentifier(QString name);

    void setDefaultBaudrate(QString deviceName, int iBaudrate);
    void setNodeIdentifierSettingEnabled(bool b);
    void setAVRDeviceName(QString text);
    void appendLogViewText(QString text);

    void stopSignaling();
    void startSignaling();

  signals:
    void signal_USBDeviceName_changed(QString name);
    void signal_USBDeviceBaudrate_changed(QString name);
    void signal_AVRDeviceAccessSpeed_changed(QString name);
    void signal_USBDeviceXBeeType_changed(QString name);
    void signal_XBeeRemoteNodeIdentifier_changed(QString name);

  private slots:
    void USBDevice_Name_changed(QString name);
    void USBDevice_Baudrate_changed(QString name);
    void AVRDevice_AccessSpeed_changed(QString name);
    void XBEE_AdapterType_changed(QString elementName);
    void XBEE_RemoteNodeIdentifier_changed(QString elementName);

  private:
    QComboBox *coBox_USBDeviceName;
    QComboBox *coBox_USBDeviceBaudrate;
    QComboBox *coBox_AVRDeviceAccSpeed;
    QComboBox *coBox_USBDeviceXBeeType;
    QComboBox *coBox_XBeeRemoteNodeIdentifier;

    QLabel *label_coBox_USBDeviceName;
    QLabel *label_coBox_USBDeviceBaudrate;
    QLabel *label_coBox_AVRDeviceAccSpeed;
    QLabel *label_coBox_AVRDeviceName;
    QLabel *label_coBox_USBDeviceXBeeType;
    QLabel *label_coBox_XBeeRemoteNodeIdentifier;

    QLabel *label_AVRDeviceNameView;

    QLabel *labelPixmap_USB_ISP_Adapter;
    QLabel *labelPixmap_USB_UART_Adapter;
    QLabel *labelPixmap_USB_XBEE_Adapter;

    QGroupBox *groupBox_ISP;
    QGroupBox *groupBox_BL;
    QGroupBox *groupBox_Settings;
    QGridLayout *gridLayout;

    bool signaling; ///< Stopps all signaling if false.

    int iDefaultBaudrate_isp;
    int iDefaultBaudrate_usart;
    int iDefaultBaudrate_xbee;

  };

}//namespace lpzrobots
#endif /* QPANELSETTING_H_ */
