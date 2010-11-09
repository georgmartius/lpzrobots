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
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "panelSetting.h"

namespace lpzrobots {

  QPanelSetting::QPanelSetting() {
    gridLayout = NULL;

    //setPalette(QPalette(QColor(220, 230, 220)));
    setAutoFillBackground(true);

    QFont fHugeBold("Curier", 12, QFont::Bold);
    QFont fHugeItalic("Curier", 12, QFont::StyleItalic);
    QFont fLargeBold("Curier", 10, QFont::Bold);
    QFont fBold("Curier", 8, QFont::Bold);
    QFont fNormal("Curier", 8, QFont::Normal);

    groupBox_Settings = new QGroupBox("Settings ...");
    groupBox_Settings->setFont(fLargeBold);

    QGridLayout *grid = new QGridLayout();
    this->setLayout(grid);
    grid->addWidget(groupBox_Settings, 0, 0);

    label_coBox_USBDeviceName = new QLabel();
    label_coBox_USBDeviceName->setText("Device-Name");
    label_coBox_USBDeviceName->setFont(fLargeBold);

    label_coBox_USBDeviceBaudrate = new QLabel();
    label_coBox_USBDeviceBaudrate->setText("Baudrate");
    label_coBox_USBDeviceBaudrate->setFont(fLargeBold);

    label_coBox_AVRDeviceAccSpeed = new QLabel();
    label_coBox_AVRDeviceAccSpeed->setText("Access-Speed");
    label_coBox_AVRDeviceAccSpeed->setFont(fLargeBold);

    label_coBox_AVRDeviceName = new QLabel();
    label_coBox_AVRDeviceName->setText("AVR-Target-Device:");
    label_coBox_AVRDeviceName->setFont(fLargeBold);

    label_AVRDeviceNameView = new QLabel();
    label_AVRDeviceNameView->setText("not determined");
    label_AVRDeviceNameView->setFont(fLargeBold);

    label_coBox_USBDeviceXBeeType = new QLabel();
    label_coBox_USBDeviceXBeeType->setText("XBeeDeviceType");
    label_coBox_USBDeviceXBeeType->setFont(fLargeBold);

    label_coBox_XBeeRemoteNodeIdentifier = new QLabel();
    label_coBox_XBeeRemoteNodeIdentifier->setText("RemoteNodeIdentifier");
    label_coBox_XBeeRemoteNodeIdentifier->setFont(fLargeBold);
    label_coBox_XBeeRemoteNodeIdentifier->setEnabled(false);

    coBox_USBDeviceName = new QComboBox();
    coBox_USBDeviceName->setMinimumWidth(170);
    coBox_USBDeviceName->setFont(fNormal);

    coBox_USBDeviceBaudrate = new QComboBox();
    coBox_USBDeviceBaudrate->setMinimumWidth(170);
    coBox_USBDeviceBaudrate->setFont(fNormal);
    coBox_USBDeviceBaudrate->setCurrentIndex(0);

    coBox_AVRDeviceAccSpeed = new QComboBox();
    coBox_AVRDeviceAccSpeed->setMinimumWidth(170);
    coBox_AVRDeviceAccSpeed->setFont(fNormal);

    coBox_USBDeviceXBeeType = new QComboBox();
    coBox_USBDeviceXBeeType->setMinimumWidth(170);
    coBox_USBDeviceXBeeType->setFont(fNormal);

    coBox_XBeeRemoteNodeIdentifier = new QComboBox();
    coBox_XBeeRemoteNodeIdentifier->setMinimumWidth(200);
    coBox_XBeeRemoteNodeIdentifier->setFont(fNormal);
    coBox_XBeeRemoteNodeIdentifier->setEnabled(false);

    labelPixmap_USB_ISP_Adapter = new QLabel;
    labelPixmap_USB_ISP_Adapter->setBackgroundRole(QPalette::Base);
    labelPixmap_USB_ISP_Adapter->setScaledContents(false);
    labelPixmap_USB_ISP_Adapter->setPixmap(QPixmap::fromImage(QImage(":/images/USB_ISP_Adapter.png").scaledToWidth(200, Qt::SmoothTransformation)));

    labelPixmap_USB_UART_Adapter = new QLabel;
    labelPixmap_USB_UART_Adapter->setBackgroundRole(QPalette::Base);
    labelPixmap_USB_UART_Adapter->setScaledContents(false);
    labelPixmap_USB_UART_Adapter->setPixmap(QPixmap::fromImage(QImage(":/images/USB_UART_Adapter.png").scaledToWidth(200, Qt::SmoothTransformation)));

    labelPixmap_USB_XBEE_Adapter = new QLabel;
    labelPixmap_USB_XBEE_Adapter->setBackgroundRole(QPalette::Base);
    labelPixmap_USB_XBEE_Adapter->setScaledContents(false);
    labelPixmap_USB_XBEE_Adapter->setPixmap(QPixmap::fromImage(QImage(":/images/USB_XBEE_Adapter.png").scaledToWidth(200, Qt::SmoothTransformation)));

    connect(coBox_USBDeviceName, SIGNAL(currentIndexChanged (QString)), this, SLOT(USBDevice_Name_changed(QString)));
    connect(coBox_USBDeviceBaudrate, SIGNAL(currentIndexChanged (QString)), this, SLOT(USBDevice_Baudrate_changed(QString)));
    connect(coBox_AVRDeviceAccSpeed, SIGNAL(currentIndexChanged (QString)), this, SLOT(AVRDevice_AccessSpeed_changed(QString)));
    connect(coBox_USBDeviceXBeeType, SIGNAL(currentIndexChanged (QString)), this, SLOT(XBEE_AdapterType_changed(QString)));
    connect(coBox_XBeeRemoteNodeIdentifier, SIGNAL(currentIndexChanged (QString)), this, SLOT(XBEE_RemoteNodeIdentifier_changed(QString)));

    setMode(APPLICATION_MODE_None);

    signaling = true;

  }

  QPanelSetting::~QPanelSetting() {
  }
  void QPanelSetting::setMode(int mode) {
    delete gridLayout;

    gridLayout = new QGridLayout();
    gridLayout->setColumnMinimumWidth(0, 150);
    gridLayout->setColumnMinimumWidth(2, 20);
    gridLayout->setColumnMinimumWidth(3, 250);
    groupBox_Settings->setMaximumHeight(500);

    coBox_USBDeviceName->setVisible(false);
    coBox_USBDeviceBaudrate->setVisible(false);
    coBox_AVRDeviceAccSpeed->setVisible(false);
    coBox_USBDeviceXBeeType->setVisible(false);
    coBox_XBeeRemoteNodeIdentifier->setVisible(false);
    label_coBox_USBDeviceName->setVisible(false);
    label_coBox_USBDeviceBaudrate->setVisible(false);
    label_coBox_AVRDeviceAccSpeed->setVisible(false);
    label_coBox_AVRDeviceName->setVisible(false);
    label_coBox_USBDeviceXBeeType->setVisible(false);
    label_coBox_XBeeRemoteNodeIdentifier->setVisible(false);
    label_AVRDeviceNameView->setVisible(false);
    labelPixmap_USB_ISP_Adapter->setVisible(false);
    labelPixmap_USB_UART_Adapter->setVisible(false);
    labelPixmap_USB_XBEE_Adapter->setVisible(false);

    switch (mode)
    {
      case APPLICATION_MODE_ISP_Adapter:
      {
        label_coBox_USBDeviceName->setVisible(true);
        label_coBox_USBDeviceBaudrate->setVisible(true);
        label_coBox_AVRDeviceAccSpeed->setVisible(true);
        coBox_USBDeviceName->setVisible(true);
        coBox_USBDeviceBaudrate->setVisible(true);
        coBox_AVRDeviceAccSpeed->setVisible(true);
        label_AVRDeviceNameView->setVisible(true);
        labelPixmap_USB_ISP_Adapter->setVisible(true);

        gridLayout->addWidget(label_coBox_USBDeviceName, 0, 0, Qt::AlignRight);
        gridLayout->addWidget(label_coBox_USBDeviceBaudrate, 1, 0, Qt::AlignRight);
        gridLayout->addWidget(label_coBox_AVRDeviceAccSpeed, 2, 0, Qt::AlignRight);
        gridLayout->addWidget(label_coBox_AVRDeviceName, 3, 0, Qt::AlignRight);
        gridLayout->addWidget(label_AVRDeviceNameView, 4, 0, 1, 2, Qt::AlignCenter);
        gridLayout->addWidget(coBox_USBDeviceName, 0, 1);
        gridLayout->addWidget(coBox_USBDeviceBaudrate, 1, 1);
        gridLayout->addWidget(coBox_AVRDeviceAccSpeed, 2, 1);
        gridLayout->addWidget(labelPixmap_USB_ISP_Adapter, 0, 3, 5, 1, Qt::AlignCenter);

        groupBox_Settings->setTitle("USB-ISP-Adapter");
        groupBox_Settings->setLayout(gridLayout);
        break;
      }
      case APPLICATION_MODE_USART_Adapter:
      {
        label_coBox_USBDeviceName->setVisible(true);
        label_coBox_USBDeviceBaudrate->setVisible(true);
        coBox_USBDeviceName->setVisible(true);
        coBox_USBDeviceBaudrate->setVisible(true);
        labelPixmap_USB_UART_Adapter->setVisible(true);

        gridLayout->addWidget(label_coBox_USBDeviceName, 0, 0, Qt::AlignRight);
        gridLayout->addWidget(label_coBox_USBDeviceBaudrate, 1, 0, Qt::AlignRight);
        gridLayout->addWidget(coBox_USBDeviceName, 0, 1);
        gridLayout->addWidget(coBox_USBDeviceBaudrate, 1, 1);
        gridLayout->addWidget(labelPixmap_USB_UART_Adapter, 0, 3, 3, 1, Qt::AlignCenter);

        groupBox_Settings->setTitle("USB-UART-Adapter");
        groupBox_Settings->setLayout(gridLayout);
        break;
      }
      case APPLICATION_MODE_XBEE_Adapter:
      {
        label_coBox_USBDeviceName->setVisible(true);
        label_coBox_USBDeviceBaudrate->setVisible(true);
        label_coBox_USBDeviceXBeeType->setVisible(true);
        label_coBox_XBeeRemoteNodeIdentifier->setVisible(true);
        coBox_USBDeviceName->setVisible(true);
        coBox_USBDeviceBaudrate->setVisible(true);
        coBox_USBDeviceXBeeType->setVisible(true);
        coBox_XBeeRemoteNodeIdentifier->setVisible(true);
        labelPixmap_USB_XBEE_Adapter->setVisible(true);

        gridLayout->addWidget(label_coBox_USBDeviceName, 0, 0, Qt::AlignRight);
        gridLayout->addWidget(label_coBox_USBDeviceBaudrate, 1, 0, Qt::AlignRight);
        gridLayout->addWidget(label_coBox_USBDeviceXBeeType, 2, 0, Qt::AlignRight);
        gridLayout->addWidget(label_coBox_XBeeRemoteNodeIdentifier, 3, 0, Qt::AlignRight | Qt::AlignBottom);
        gridLayout->addWidget(coBox_USBDeviceName, 0, 1);
        gridLayout->addWidget(coBox_USBDeviceBaudrate, 1, 1);
        gridLayout->addWidget(coBox_USBDeviceXBeeType, 2, 1);
        gridLayout->addWidget(coBox_XBeeRemoteNodeIdentifier, 4, 0, 1, 2);
        gridLayout->addWidget(labelPixmap_USB_XBEE_Adapter, 0, 3, 5, 1, Qt::AlignRight | Qt::AlignTop);

        groupBox_Settings->setTitle("USB-XBEE-Adapter");
        groupBox_Settings->setLayout(gridLayout);
        break;
      }
      default:
      {
        label_coBox_USBDeviceName->setVisible(true);
        coBox_USBDeviceName->setVisible(true);

        gridLayout->addWidget(label_coBox_USBDeviceName, 0, 0, Qt::AlignRight);
        gridLayout->addWidget(coBox_USBDeviceName, 0, 1);

        groupBox_Settings->setTitle("Select-Adapter");
        groupBox_Settings->setLayout(gridLayout);
        break;
      }
    }
  }

  void QPanelSetting::stopSignaling() {
    signaling = false;
  }
  void QPanelSetting::startSignaling() {
    signaling = true;
  }

  //------------------------------------------------------------------------------------------------------
  void QPanelSetting::USBDevice_Name_changed(QString elementName) {
    QStringList baudrates;
    int iDefaultBaudrate = 9600;

    if (elementName.startsWith("USB-ISP-Adapter"))
    {
      setMode(APPLICATION_MODE_ISP_Adapter);
      iDefaultBaudrate = iDefaultBaudrate_isp;
    } else if (elementName.startsWith("USB-USART-Adapter"))
    {
      setMode(APPLICATION_MODE_USART_Adapter);
      iDefaultBaudrate = iDefaultBaudrate_usart;
    } else if (elementName.startsWith("USB-XBEE-Adapter"))
    {
      setMode(APPLICATION_MODE_XBEE_Adapter);
      iDefaultBaudrate = iDefaultBaudrate_xbee;
    }

    // trage die default-baudrate in die ComboBox ein
    for (int i = 0; i < coBox_USBDeviceBaudrate->count(); i++)
    {
      QString sLine = coBox_USBDeviceBaudrate->itemText(i).split(" ").at(0);
      if (sLine.compare(QString::number(iDefaultBaudrate)) == 0)
      {
        coBox_USBDeviceBaudrate->setItemText(i, sLine + " (default)");
        coBox_USBDeviceBaudrate->setCurrentIndex(i);
      } else
        coBox_USBDeviceBaudrate->setItemText(i, sLine);
    }

    if (signaling)
      emit signal_USBDeviceName_changed(elementName);
  }
  void QPanelSetting::setUSBDeviceNames(QStringList USBDeviceNameList) {
    coBox_USBDeviceName->clear();
    coBox_USBDeviceName->insertItems(0, USBDeviceNameList);
  }
  QString QPanelSetting::getUSBDeviceName() {
    return coBox_USBDeviceName->currentText();
  }
  void QPanelSetting::setUSBDeviceName(QString serialUSBDeviceName) {
    coBox_USBDeviceName->setCurrentIndex(coBox_USBDeviceName->findText(serialUSBDeviceName, Qt::MatchExactly));
  }
  //------------------------------------------------------------------------------------------------------
  void QPanelSetting::USBDevice_Baudrate_changed(QString elementName) {
    QStringList sList = elementName.split(" ");
    if (signaling)
      emit signal_USBDeviceBaudrate_changed(sList.at(0));
  }
  void QPanelSetting::setUSBDeviceBaudrates(QStringList baudrateList) {
    coBox_USBDeviceBaudrate->clear();
    coBox_USBDeviceBaudrate->insertItems(0, baudrateList);
  }
  QString QPanelSetting::getUSBDeviceBaudrate() {
    return coBox_USBDeviceBaudrate->currentText().split(" ").at(0);
  }
  void QPanelSetting::setUSBDeviceBaudrate(QString baudrate) {
    //coBox_USBDeviceBaudrate->setCurrentIndex(coBox_USBDeviceBaudrate->findText(baudrate, Qt::MatchExactly));

    for (int i = 0; i < coBox_USBDeviceBaudrate->count(); i++)
    {
      QString sLine = coBox_USBDeviceBaudrate->itemText(i).split(" ").at(0);
      if (sLine.compare(baudrate) == 0)
        coBox_USBDeviceBaudrate->setCurrentIndex(i);
    }

  }

  //------------------------------------------------------------------------------------------------------
  void QPanelSetting::AVRDevice_AccessSpeed_changed(QString elementName) {
    if (signaling)
      emit signal_AVRDeviceAccessSpeed_changed(elementName);
  }
  void QPanelSetting::setAVRDeviceAccessSpeeds(QStringList targetSpeedList) {
    coBox_AVRDeviceAccSpeed->clear();
    coBox_AVRDeviceAccSpeed->insertItems(0, targetSpeedList);
  }
  QString QPanelSetting::getAVRDeviceAccessSpeed() {
    return coBox_AVRDeviceAccSpeed->currentText();
  }
  void QPanelSetting::setAVRDeviceAccessSpeed(QString targetSpeed) {
    coBox_AVRDeviceAccSpeed->setCurrentIndex(coBox_AVRDeviceAccSpeed->findText(targetSpeed, Qt::MatchExactly));
  }
  //------------------------------------------------------------------------------------------------------
  void QPanelSetting::XBEE_AdapterType_changed(QString elementName) {
    if (signaling)
      emit signal_USBDeviceXBeeType_changed(elementName);
  }
  void QPanelSetting::setUSBDeviceXBeeTypes(QStringList connectionList) {
    coBox_USBDeviceXBeeType->clear();
    coBox_USBDeviceXBeeType->insertItems(0, connectionList);
  }
  QString QPanelSetting::getUSBDeviceXBeeType() {
    return coBox_USBDeviceXBeeType->currentText();
  }
  void QPanelSetting::setUSBDeviceXBeeType(QString connection) {
    coBox_USBDeviceXBeeType->setCurrentIndex(coBox_USBDeviceXBeeType->findText(connection, Qt::MatchExactly));
  }
  //------------------------------------------------------------------------------------------------------
  void QPanelSetting::XBEE_RemoteNodeIdentifier_changed(QString elementName) {
    if (signaling)
      emit signal_XBeeRemoteNodeIdentifier_changed(elementName);
  }
  void QPanelSetting::setXBeeRemoteNodeIdentifiers(QStringList nodeIdentifierList) {
    coBox_XBeeRemoteNodeIdentifier->clear();
    coBox_XBeeRemoteNodeIdentifier->insertItems(0, nodeIdentifierList);
  }
  QString QPanelSetting::getXBeeRemoteNodeIdentifier() {
    return coBox_XBeeRemoteNodeIdentifier->currentText();
  }
  void QPanelSetting::setXBeeRemoteNodeIdentifier(QString nodeIdentifier) {
    int itemPosition = coBox_XBeeRemoteNodeIdentifier->findText(nodeIdentifier, Qt::MatchExactly);
    int NumberItems = coBox_XBeeRemoteNodeIdentifier->count();
    if (itemPosition < 0)
    {
      coBox_XBeeRemoteNodeIdentifier->insertItem(NumberItems, nodeIdentifier);
      coBox_XBeeRemoteNodeIdentifier->setCurrentIndex(NumberItems);
    } else
    {
      coBox_XBeeRemoteNodeIdentifier->setCurrentIndex(itemPosition);
    }
  }
  void QPanelSetting::setNodeIdentifierSettingEnabled(bool b) {
    label_coBox_XBeeRemoteNodeIdentifier->setEnabled(b);
    coBox_XBeeRemoteNodeIdentifier->setEnabled(b);
  }
  //------------------------------------------------------------------------------------------------------
  void QPanelSetting::setAVRDeviceName(QString text) {
    label_AVRDeviceNameView->setText(text);
  }
  //------------------------------------------------------------------------------------------------------
  void QPanelSetting::setDefaultBaudrate(QString deviceName, int iBaudrate) {
    if (deviceName.startsWith("USB-ISP-Adapter"))
      iDefaultBaudrate_isp = iBaudrate;
    else if (deviceName.startsWith("USB-USART-Adapter"))
      iDefaultBaudrate_usart = iBaudrate;
    else if (deviceName.startsWith("USB-XBEE-Adapter"))
      iDefaultBaudrate_xbee = iBaudrate;

  }

}//namespace lpzrobots
