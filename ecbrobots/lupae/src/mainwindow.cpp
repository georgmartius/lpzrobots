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
 *   Revision 1.5  2010-11-18 17:00:11  wrabe
 *   - current state of work
 *
 *   Revision 1.4  2010/11/14 12:14:06  wrabe
 *   - set debug to false
 *   - fix: show avrDeviceName after reading signature-bytes
 *
 *   Revision 1.3  2010/11/09 18:07:40  wrabe
 *   - change location of the application saves (settings)
 *
 *   Revision 1.2  2010/11/09 17:56:55  wrabe
 *   - change of the communication protocoll between lupae and usb-isp-adapter
 *   - therefore recoding the dedicated methods
 *   - reduction of the overloded send_Message methods to one method only
 *   - insertion of QExtActions to join all events of menu-buttons as well of quickstart-buttons
 *   - adding two new functions to read out and write into the eeprom-space of the atmega128 at an ecb
 *   - change of the fontSize in the hexViewer, change of the total-width of the window
 *                                              *
 *                                                                         *
 ***************************************************************************/

#include "mainwindow.h"

namespace lpzrobots {

  MainWindow::MainWindow(QString applicationPathArg) {
    applicationPath = applicationPathArg;

    panelHexViewer = new QPanelHexViewer();
    avrFuseDialog = new QAvrFuseDialog();
    ft232manager = new QFT232DeviceManager();
    panelSetting = new QPanelSetting();
    panelLogView = new QPanelLogView();
    timer = new QTimer();
    progress = 0;
    hasBinary = false;
    hasAVRDeviceIdedentified = false;
    nextOperationState = NEXT_OP_NONE;

    // Der Transmit-Puffer
    timerParams = transmitTimerLastAction_none;
    transmitBuffer.clear();
    transmitBufferCheckSum = 0;
    ECB_OperationRetries = 0;
    ECB_OperationRetriesMax = 3;
    applicationMode = APPLICATION_MODE_None;

    programmerWaitCycles = 100;

    // FileDialog:
    //------------------------------------------------------
    fileDialog = new QFileDialog();
    QStringList filters;
    filters << "binary files (*.bin)" << "any files (*)";
    fileDialog->setFilters(filters);

    // Layout:
    //------------------------------------------------------
    QGridLayout *grid = new QGridLayout();
    QWidget *mainpanel = new QWidget();
    mainpanel->setLayout(grid);
    setCentralWidget(mainpanel);
    grid->addWidget(panelSetting, 0, 0);

    tabWidget = new QTabWidget;
    tabWidget->addTab(panelLogView, tr("Report"));
    tabWidget->addTab(panelHexViewer, tr("Binary"));
    grid->addWidget(tabWidget, 1, 0);

    setMinimumWidth(650);
    setMaximumWidth(650);

    initPanelSetting();
    createActions();
    createMenus();
    createToolBars();
    createStatusBar();
    setCurrentFile("");

    connect(avrFuseDialog, SIGNAL(readFuseBits(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));
    connect(avrFuseDialog, SIGNAL(writeFuseBits(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));
    connect(avrFuseDialog, SIGNAL(textLog(QString)), this, SLOT(sl_TextLog(QString)));
    connect(ft232manager, SIGNAL(textLog(QString)), this, SLOT(sl_TextLog(QString)));
    connect(ft232manager, SIGNAL(newData(QByteArray)), this, SLOT(sl_DispatchMessage(QByteArray)));
    connect(ft232manager, SIGNAL(deviceOpened()), this, SLOT(sl_USBDevice_opened()));
    connect(panelSetting, SIGNAL(signal_USBDeviceName_changed(QString)), this, SLOT(sl_USBDevice_Name_changed(QString)));
    connect(panelSetting, SIGNAL(signal_USBDeviceBaudrate_changed(QString)), this, SLOT(sl_USBDevice_Baudrate_changed(QString)));
    connect(panelSetting, SIGNAL(signal_AVRDeviceAccessSpeed_changed(QString)), this, SLOT(sl_AVRDevice_AccessSpeed_changed(QString)));
    connect(panelSetting, SIGNAL(signal_USBDeviceXBeeType_changed(QString)), this, SLOT(sl_USBDeviceXBeeType_Changed(QString)));
    connect(panelSetting, SIGNAL(signal_XBeeRemoteNodeIdentifier_changed(QString)), this, SLOT(sl_XBeeRemoteNodeIdentifier_changed(QString)));
    connect(timer, SIGNAL(timeout()), this, SLOT(sl_TimerExpired()));

    panelSetting->stopSignaling();
    readSettings();
    panelSetting->startSignaling();
    loadTargetDeviceParameter();

    sl_eventHandler_application(EVENT_APPLICATION_SCAN_USBDEVICE);
    //sl_ScanUSBDevices();

  }
  void MainWindow::initPanelSetting() {
    QStringList baudrates;
    baudrates << "9600" << "19200" << "57600" << "115200" << "230400" << "460800" << "921600";
    panelSetting->setUSBDeviceBaudrates(baudrates);
    panelSetting->setUSBDeviceBaudrate(baudrates.at(0));

    QStringList targetSpeeds;
    targetSpeeds << "slow" << "medium" << "fast";
    panelSetting->setAVRDeviceAccessSpeeds(targetSpeeds);
    panelSetting->setAVRDeviceAccessSpeed(targetSpeeds.at(0));
    setProgrammerWaitCycles(targetSpeeds.at(0));

    QStringList connections;
    connections << "unknown" << "XBeeSerie1" << "XBeeSerie2";
    panelSetting->setUSBDeviceXBeeTypes(connections);
    panelSetting->setUSBDeviceXBeeType(connections.at(0));

    USBDeviceXBeeType = XBeeType_unknown;

  }
  void MainWindow::createActions() {
    action_Binary_open = new QExtAction(EVENT_APPLICATION_BINARY_OPEN, QIcon(":/images/open.png"), tr("&Open..."), this);
    //action_Binary_open = new QAction(tr("&Open..."), this);
    action_Binary_open->setShortcut(tr("Ctrl+O"));
    action_Binary_open->setStatusTip(tr("Open binary"));
    connect(action_Binary_open, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_application(int)));

    action_Binary_save = new QExtAction(EVENT_APPLICATION_BINARY_SAVE, QIcon(":/images/save.png"), tr("&Save"), this);
    //action_Binary_save = new QAction(tr("&Save"), this);
    action_Binary_save->setShortcut(tr("Ctrl+S"));
    action_Binary_save->setStatusTip(tr("Save the binary"));
    action_Binary_save->setEnabled(false);
    connect(action_Binary_save, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_application(int)));

    action_Exit = new QExtAction(EVENT_APPLICATION_CLOSE, tr("&Quit"), this);
    action_Exit->setShortcut(tr("Ctrl+Q"));
    action_Exit->setStatusTip(tr("Exit the application"));
    connect(action_Exit, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_application(int)));

    // Actions for Target Device
    action_Target_SignatureBytes_read = new QExtAction(EVENT_ISP_AVRDEVICE_SIGNATURE_READ, tr("&Identify Target Device"), this);
    action_Target_SignatureBytes_read->setShortcut(tr("Ctrl+I"));
    action_Target_SignatureBytes_read->setStatusTip(tr("Read Target Device Signature Bytes"));
    action_Target_SignatureBytes_read->setEnabled(false);
    connect(action_Target_SignatureBytes_read, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    action_Target_CalibrationBytes_read = new QExtAction(EVENT_ISP_AVRDEVICE_CALIBRATION_READ, tr("Read Calibration-Bytes"), this);
    action_Target_CalibrationBytes_read->setStatusTip(tr("Read Target Device Calibration Bytes"));
    action_Target_CalibrationBytes_read->setEnabled(false);
    connect(action_Target_CalibrationBytes_read, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    action_Target_ShowFuseDialog = new QExtAction(EVENT_ISP_AVRDEVICE_FUSES_EDITOR_SHOW, tr("Show FuseBit-Editor"), this);
    action_Target_ShowFuseDialog->setStatusTip(tr("Open the Fuse-Bit-Editor."));
    action_Target_ShowFuseDialog->setEnabled(false);
    connect(action_Target_ShowFuseDialog, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    action_Target_Flash_read = new QExtAction(EVENT_ISP_AVRDEVICE_FLASH_READ, tr("&Read Flash"), this);
    action_Target_Flash_read->setStatusTip(tr("Read Target Device Programm-Space"));
    action_Target_Flash_read->setShortcut(tr("Ctrl+R"));
    action_Target_Flash_read->setEnabled(false);
    connect(action_Target_Flash_read, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    action_Target_Flash_write = new QExtAction(EVENT_ISP_AVRDEVICE_FLASH_WRITE, tr("&WriteFlash"), this);
    action_Target_Flash_write->setShortcut(tr("Ctrl+W"));
    action_Target_Flash_write->setStatusTip(tr("Write Target Device Programm-Space"));
    action_Target_Flash_write->setEnabled(false);
    connect(action_Target_Flash_write, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    action_Target_Flash_update_write = new QExtAction(EVENT_ISP_AVRDEVICE_FLASH_UPDATEWRITE, tr("&Update/Flash"), this);
    action_Target_Flash_update_write->setShortcut(tr("Ctrl+U"));
    action_Target_Flash_update_write->setStatusTip(tr("Reload the binary and write Target Device Programm-Space"));
    action_Target_Flash_update_write->setEnabled(false);
    connect(action_Target_Flash_update_write, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    // Actions for Programmer
    action_Programmer_SoftwareVersion = new QExtAction(EVENT_ISP_PROGRAMMER_SOFTWARE_VERSION, tr("Version"), this);
    action_Programmer_SoftwareVersion->setShortcut(tr("ALT+SHIFT+V"));
    action_Programmer_SoftwareVersion->setStatusTip(tr("Reads out the SoftwareVersion of the USB-ISP-Adapter"));
    connect(action_Programmer_SoftwareVersion, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    action_Programmer_reset = new QExtAction(EVENT_ISP_PROGRAMMER_RESET, tr("Reset"), this);
    action_Programmer_reset->setStatusTip(tr("Resets the USB-Programmer"));
    connect(action_Programmer_reset, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    action_Programmer_Flash_read = new QExtAction(EVENT_ISP_PROGRAMMER_FLASH_READ, tr("ReadFlash"), this);
    action_Programmer_Flash_read->setStatusTip(tr("Read Programm from Programmer"));
    action_Programmer_Flash_read->setShortcut(tr("ALT+SHIFT+R"));
    connect(action_Programmer_Flash_read, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    action_Programmer_Flash_write = new QExtAction(EVENT_ISP_PROGRAMMER_FLASH_WRITE, tr("WriteFlash"), this);
    action_Programmer_Flash_write->setStatusTip(tr("Write new Programm to Programmer"));
    action_Programmer_Flash_write->setShortcut(tr("ALT+SHIFT+W"));
    action_Programmer_Flash_write->setEnabled(false);
    connect(action_Programmer_Flash_write, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ispProgrammer(int)));

    // Actions for the Bootloader
    action_ECB_reset = new QExtAction(EVENT_ECB_RESET, tr("Reset ECB"), this);
    action_ECB_reset->setShortcut(tr("Ctrl+Shift+R"));
    action_ECB_reset->setStatusTip(tr("Send Command to RESET the Controller"));
    action_ECB_reset->setEnabled(false);
    connect(action_ECB_reset, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ecbBootloader(int)));

    action_ECB_Bootloader_Flash_read = new QExtAction(EVENT_ECB_BOOTLOADER_FLASH_READ, tr("&Read ProgramSpace"), this);
    action_ECB_Bootloader_Flash_read->setShortcut(tr("Ctrl+R"));
    action_ECB_Bootloader_Flash_read->setStatusTip(tr("Read the Programm-Space of the specified Controller"));
    action_ECB_Bootloader_Flash_read->setEnabled(false);
    connect(action_ECB_Bootloader_Flash_read, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ecbBootloader(int)));

    action_ECB_Bootloader_Flash_write = new QExtAction(EVENT_ECB_BOOTLOADER_FLASH_WRITE, tr("&Write ProgramSpace"), this);
    action_ECB_Bootloader_Flash_write->setShortcut(tr("Ctrl+W"));
    action_ECB_Bootloader_Flash_write->setStatusTip(tr("Write the loaded Binary to the Programm-Space of the connected ECB"));
    action_ECB_Bootloader_Flash_write->setEnabled(false);
    connect(action_ECB_Bootloader_Flash_write, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ecbBootloader(int)));

    action_ECB_Bootloader_Flash_update_write = new QExtAction(EVENT_ECB_BOOTLOADER_FLASH_UPDATEWRITE, tr("&Update/Write ProgramSpace"), this);
    action_ECB_Bootloader_Flash_update_write->setShortcut(tr("Ctrl+U"));
    action_ECB_Bootloader_Flash_update_write->setStatusTip(tr("Reload the binary and write in into the Programm-Space of the connected ECB"));
    action_ECB_Bootloader_Flash_update_write->setEnabled(false);
    connect(action_ECB_Bootloader_Flash_update_write, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ecbBootloader(int)));

    action_ECB_Bootloader_EEPROM_read = new QExtAction(EVENT_ECB_BOOTLOADER_EEPROM_READ, tr("Read EEPROM"), this);
    action_ECB_Bootloader_EEPROM_read->setStatusTip(tr("Read the Data-Space of the specified Controller"));
    action_ECB_Bootloader_EEPROM_read->setEnabled(true);
    connect(action_ECB_Bootloader_EEPROM_read, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ecbBootloader(int)));

    action_ECB_Bootloader_EEPROM_write = new QExtAction(EVENT_ECB_BOOTLOADER_EEPROM_WRITE, tr("Write EEPROM"), this);
    action_ECB_Bootloader_EEPROM_write->setStatusTip(tr("Write the loaded Binary to the Data-Space of the connected ECB"));
    action_ECB_Bootloader_EEPROM_write->setEnabled(false);
    connect(action_ECB_Bootloader_EEPROM_write, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_ecbBootloader(int)));

    // Extras...
    action_ClearLogView = new QExtAction(EVENT_APPLICATION_LOGVIEW_CLEAR, tr("&Clear LogView"), this);
    action_ClearLogView->setShortcut(tr("Ctrl+Shift+C"));
    action_ClearLogView->setStatusTip(tr("Clear the LogView"));
    connect(action_ClearLogView, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_application(int)));

    action_SerialPorts_refresh = new QExtAction(EVENT_APPLICATION_SCAN_USBDEVICE, tr("ScanUSBDevices"), this);
    action_SerialPorts_refresh->setStatusTip(tr("Scan for connected USB-Tools."));
    action_SerialPorts_refresh->setShortcut(Qt::Key_F5);
    action_SerialPorts_refresh->setEnabled(true);
    connect(action_SerialPorts_refresh, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_application(int)));

    // Actions About
    action_About = new QExtAction(EVENT_APPLICATION_ABOUT, tr("&About"), this);
    action_About->setStatusTip(tr("Show the application's About box"));
    connect(action_About, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler_application(int)));

  }
  void MainWindow::createToolBars() {
    fileToolBar = addToolBar(tr("File"));
    fileToolBar->addAction(action_Binary_open);
    fileToolBar->addAction(action_Binary_save);
  }
  void MainWindow::createMenus(int applicationMode) {
    //delete this->menuBar();
    this->menuBar()->clear();

    switch (applicationMode) {
      case APPLICATION_MODE_ISP_Adapter: {
        fileMenu = menuBar()->addMenu(tr("&File"));
        fileMenu->addAction(action_Binary_open);
        fileMenu->addAction(action_Binary_save);
        fileMenu->addSeparator();
        fileMenu->addAction(action_Exit);

        ispMenu = menuBar()->addMenu(tr("&ISP"));
        ispMenu->addAction(action_Target_SignatureBytes_read);
        ispMenu->addSeparator();
        ispMenu->addAction(action_Target_CalibrationBytes_read);
        ispMenu->addAction(action_Target_ShowFuseDialog);
        ispMenu->addSeparator();
        ispMenu->addAction(action_Target_Flash_read);
        ispMenu->addAction(action_Target_Flash_write);
        ispMenu->addAction(action_Target_Flash_update_write);

        extraMenu = menuBar()->addMenu(tr("&Extra"));
        ispProgrammerMenu = extraMenu->addMenu(tr("USB-ISP-Adapter"));
        ispProgrammerMenu->addAction(action_Programmer_SoftwareVersion);
        ispProgrammerMenu->addAction(action_Programmer_reset);
        ispProgrammerMenu->addAction(action_Programmer_Flash_read);
        ispProgrammerMenu->addAction(action_Programmer_Flash_write);
        extraMenu->addSeparator();
        extraMenu->addAction(action_SerialPorts_refresh);
        extraMenu->addAction(action_ClearLogView);

        helpMenu = menuBar()->addMenu(tr("&Help"));
        helpMenu->addAction(action_About);
        break;
      }
      case APPLICATION_MODE_USART_Adapter:
      case APPLICATION_MODE_XBEE_Adapter: {
        fileMenu = menuBar()->addMenu(tr("&File"));
        fileMenu->addAction(action_Binary_open);
        fileMenu->addAction(action_Binary_save);
        fileMenu->addSeparator();
        fileMenu->addAction(action_Exit);

        blMenu = menuBar()->addMenu(tr("Bootloader"));
        blMenu->addAction(action_ECB_reset);
        blMenu->addSeparator();
        blMenu->addAction(action_ECB_Bootloader_EEPROM_read);
        blMenu->addAction(action_ECB_Bootloader_EEPROM_write);
        blMenu->addSeparator();
        blMenu->addAction(action_ECB_Bootloader_Flash_read);
        blMenu->addAction(action_ECB_Bootloader_Flash_write);
        blMenu->addAction(action_ECB_Bootloader_Flash_update_write);

        extraMenu = menuBar()->addMenu(tr("&Extra"));
        extraMenu->addAction(action_SerialPorts_refresh);
        extraMenu->addSeparator();
        extraMenu->addAction(action_ClearLogView);

        helpMenu = menuBar()->addMenu(tr("&Help"));
        helpMenu->addAction(action_About);
        break;
      }
      default: {
        fileMenu = menuBar()->addMenu(tr("&File"));
        fileMenu->addAction(action_Binary_open);
        fileMenu->addAction(action_Binary_save);
        fileMenu->addSeparator();
        fileMenu->addAction(action_Exit);

        extraMenu = menuBar()->addMenu(tr("&Extra"));
        extraMenu->addAction(action_SerialPorts_refresh);
        extraMenu->addAction(action_ClearLogView);

        helpMenu = menuBar()->addMenu(tr("&Help"));
        helpMenu->addAction(action_About);
        break;
      }
    }
  }

  void MainWindow::createStatusBar() {
    statusBar()->showMessage(tr("Ready"));
  }
  void MainWindow::readSettings() {
    QSettings settings(applicationPath + QString("application.saves"), QSettings::IniFormat);
    QPoint pos = settings.value("pos", QPoint(200, 200)).toPoint();
    QSize size = settings.value("size", QSize(400, 400)).toSize();
    resize(size);
    move(pos);

    iDefaultBaudrate_isp = settings.value("USBDevice_ISP-Adapter_BaudrateDefaultValue", 921600).toInt();
    iDefaultBaudrate_usart = settings.value("USBDevice_USART-Adapter_BaudrateDefaultValue", 460800).toInt();
    iDefaultBaudrate_xbee = settings.value("USBDevice_XBEE-Adapter_BaudrateDefaultValue", 57600).toInt();

    panelSetting->setDefaultBaudrate("USB-ISP-Adapter", iDefaultBaudrate_isp);
    panelSetting->setDefaultBaudrate("USB-USART-Adapter", iDefaultBaudrate_usart);
    panelSetting->setDefaultBaudrate("USB-XBEE-Adapter", iDefaultBaudrate_xbee);

    QStringList deviceNames = ft232manager->getDeviceList();
    panelSetting->setUSBDeviceNames(deviceNames);

    // Das zuletzt verwendete USB-Device
    QString sDeviceName = settings.value("USBDevice_Name").toString();
    panelSetting->setUSBDeviceName(sDeviceName);

    // Die zuletzt verwendete Geschwindigkeit des Zielsystems
    QString sAVRDeviceISPAccessSpeed = settings.value("AVRDevice_ISPAccessSpeed").toString();
    if (0 < sAVRDeviceISPAccessSpeed.length()) {
      panelSetting->setAVRDeviceAccessSpeed(sAVRDeviceISPAccessSpeed);
      setProgrammerWaitCycles(sAVRDeviceISPAccessSpeed);
    }

    // Zuletzt genutzen Pfad wiederherstellen
    fileDialog->selectFile(settings.value("Location_LastFile").toString());
    // Addresse des AVR-Paramter-Files einlesen
    deviceFileName = settings.value("Location_AVRDeviceList", "AVR_Device_List.xml").toString();
  }

  void MainWindow::writeSettings() {
    QSettings settings(applicationPath + QString("application.saves"), QSettings::IniFormat);
    settings.setValue("pos", pos());
    settings.setValue("size", size());

    switch (applicationMode) {
      case APPLICATION_MODE_ISP_Adapter:
        settings.setValue("ApplicationMode", "ISP_Mode");
        break;
      case APPLICATION_MODE_USART_Adapter:
        settings.setValue("ApplicationMode", "USART_Mode");
        break;
      case APPLICATION_MODE_XBEE_Adapter:
        settings.setValue("ApplicationMode", "XBEE_Mode");
        break;
      default:
        settings.setValue("ApplicationMode", "None");
        break;
    }

    settings.setValue("USBDevice_ISP-Adapter_BaudrateDefaultValue", iDefaultBaudrate_isp);
    settings.setValue("USBDevice_USART-Adapter_BaudrateDefaultValue", iDefaultBaudrate_usart);
    settings.setValue("USBDevice_XBEE-Adapter_BaudrateDefaultValue", iDefaultBaudrate_xbee);

    settings.setValue("USBDevice_Name", panelSetting->getUSBDeviceName());
    settings.setValue("USBDevice_Baudrate", panelSetting->getUSBDeviceBaudrate());
    settings.setValue("AVRDevice_ISPAccessSpeed", panelSetting->getAVRDeviceAccessSpeed());
    settings.setValue("BLMode_Connection", panelSetting->getUSBDeviceXBeeType());
    settings.setValue("Location_LastFile", fileDialog->selectedFiles().at(0));
    settings.setValue("Location_AVRDeviceList", deviceFileName);
  }
  void MainWindow::loadTargetDeviceParameter() {
    // Die Parameter der Mikrocontroller
    avrDevice = 0;
    avrDeviceList = new QAVR_DeviceList(deviceFileName);
    if (!avrDeviceList->isLoaded()) {
      // die AVR-Parameter-Datei konnte nicht eingelesen werden, möglicherweise ist der Pfad
      // zur Datei nicht korrekt gesetzt, öffne FileBrowser zur Lokalisation durch den Nutzer
      QString fileName = QFileDialog::getOpenFileName(this, // Parent(Widget)
          tr("Please specify file-location of 'AVR_Device_List.xml'"), // Caption
          "", // Pfad-Vorgabe
          tr("Xml (*.xml)")); // Datei-Filter

      if (!fileName.isEmpty())
        avrDeviceList = new QAVR_DeviceList(fileName);

      if (!avrDeviceList->isLoaded()) {
        // Die Datei konnte auch nun noch nicht gelesen werden,
        // da ohne diese Datei keine Aktionen durchgeführt werden können wird das
        // Programm geschlossen.

        QMessageBox::information(this, tr("My Application"), tr("Can't find/read AVR-Parameter-File (i.e.'AVR_Device_List.xml'), \n"
          "programm will be closed."), QMessageBox::Ok, QMessageBox::Ok);

        QTimer::singleShot(1, this, SLOT(sl_Close()));
      } else {
        deviceFileName = fileName;
      }
    }

  }
  void MainWindow::setMode(int mode) {
    switch (mode) {
      default:
        break;
    }
  }

  void MainWindow::closeEvent(QCloseEvent *event) {
    ft232manager->closeDevice();
    writeSettings();
    event->accept();
  }
  void MainWindow::sleep(ulong msecs) {
    QTime dieTime = QTime::currentTime().addMSecs(msecs);
    while (QTime::currentTime() < dieTime)
      QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
  }

  void MainWindow::push_Frame(uchar c) {
    transmitBuffer.append(c);
  }
  void MainWindow::push_FrameEscaped(uchar c) {
    // Von der Prüfsumme ausgeschlossen sind das Startsymbol und das Längenfeld,
    // deswegen erst ab dem 3. Zeichen die Prüfsumme bilden!
    if (2 < transmitBuffer.length())
      transmitBufferCheckSum += c;

    // Ist fuer dieses Zeichen eine Ausnahmebehandlung notwendig?
    if (c == 0x7E || c == 0x7D || c == 0x13 || c == 0x11) {
      transmitBuffer.append(0x7D);
      transmitBuffer.append((QByte) (c ^ 0x20));
    } else {
      transmitBuffer.append(c);
    }
  }
  bool MainWindow::transmit(int timerExpiredCode) {
    // Schreibe die Prüfsumme
    push_FrameEscaped((QByte) (255 - transmitBufferCheckSum % 256));
    // Gebe die Nachricht ueber den Seriellen-Port aus
    bool ret = ft232manager->writeData(transmitBuffer) == 0 ? true : false;

    //panelLogView->appendLogViewText("OUT:");
    //printBuffer(transmitBuffer);

    // Loesche nun den Übertragungs-Puffer und Reinitialisiere die benötigten Variablen
    transmitBufferCheckSum = 0;
    transmitBuffer.clear();
    timer->start(3000);
    timerParams = timerExpiredCode;
    return ret;
  }
  /*
   void MainWindow::send_Message(QByteArray command)
   {
   QWord length = command.length();

   push_Frame(0x7E); // Startsymbol
   push_FrameEscaped((QByte) (length >> 8)); // Length MSB
   push_FrameEscaped((QByte) (length >> 0)); // Length LSB
   for (int i = 0; i < length; i++)
   push_FrameEscaped(command[i]);
   transmit();
   timerParams = transmitTimerLastAction_SendMessageRaw;

   }
   */
  void MainWindow::send_Message(QByteArray msg) {
    switch (applicationMode) {
      case APPLICATION_MODE_ISP_Adapter: {
        QWord length = 0 + msg.length();
        push_Frame(0x7E); // Startsymbol
        push_FrameEscaped((QByte) (length >> 8)); // Length MSB
        push_FrameEscaped((QByte) (length >> 0)); // Length LSB
        for (int i = 0; i < msg.length(); i++)
          push_FrameEscaped(msg[i]);
        transmit(transmitTimerLastAction_SendMessageISP);
        break;
      }
      case APPLICATION_MODE_USART_Adapter: {
        QWord length = 2 + msg.length();
        push_Frame(0x7E); //  1: Startsymbol
        push_FrameEscaped((QByte) (length >> 8)); //  2: Length MSB
        push_FrameEscaped((QByte) (length >> 0)); //  3: Length LSB
        push_FrameEscaped(0x20); //  4: API_ID - Cable
        // AnwenderDaten
        push_FrameEscaped(0x00); // MessageGroup_ID - immer 0 -> Bootloader
        for (int i = 0; i < msg.length(); i++)
          push_FrameEscaped(msg[i]);
        transmit(transmitTimerLastAction_SendMessageBL);
        break;
      }
      case APPLICATION_MODE_XBEE_Adapter: {
        switch (USBDeviceXBeeType) {
          case XBeeType_Serie1: {
            QWord length = 2 + msg.length();
            push_Frame(0x7E); // 0x01 - Startsymbol
            push_FrameEscaped((QByte) (length >> 8)); // 0x02 - Length MSB
            push_FrameEscaped((QByte) (length >> 0)); // 0x03 - Length LSB
            push_FrameEscaped(0x01); // 0x04 - API-ID
            push_FrameEscaped(0x00); // 0x05 - Frame-ID, immer 0 -> kein ResponsePaket vom XBee
            push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 8)); // 0x06 - DestinationAddress MSB
            push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 0)); // 0x07 - DestinationAddress LSB
            push_FrameEscaped(0x01); // 0x08 - Options, immer 1  -> kein ResponsePaket vom XBee
            // AnwenderDaten
            push_FrameEscaped(0x00); // 0x09 - MessageGroup_ID, immer 0 -> Bootloader
            for (int i = 0; i < msg.length(); i++)
              push_FrameEscaped(msg[i]);
            transmit(transmitTimerLastAction_SendMessageBL);
            break;
          }
          case XBeeType_Serie2: {
            QWord length = 0 + msg.length();
            push_Frame(0x7E); // 0x01 - Startsymbol
            push_FrameEscaped((QByte) (length >> 8)); // 0x02 - Length MSB
            push_FrameEscaped((QByte) (length >> 0)); // 0x03 - Length LSB
            push_FrameEscaped(0x10); // 0x04 - API_ID - TransmitRequest XBeeSerie2
            push_FrameEscaped(0x00); // 0x05 - Frame-ID - immer 0 -> kein ResponsePaket vom XBee
            push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 7 * 8)); // 0x06 - 64_Bit_Destination_Network_Address
            push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 6 * 8)); // 0x07
            push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 5 * 8)); // 0x08
            push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 4 * 8)); // 0x09
            push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 3 * 8)); // 0x0A
            push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 2 * 8)); // 0x0B
            push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 1 * 8)); // 0x0C
            push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 0 * 8)); // 0x0D
            push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 1 * 8)); // 0x0E - 16_Bit_Destination_Network_Address
            push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 0 * 8)); // 0x0F
            push_FrameEscaped(0x00); // 0x10 - Broadcast-Range
            push_FrameEscaped(0x01); // 0x11 - OptionsByte, immer 1  -> kein ResponsePaket vom XBee
            // AnwenderDaten
            push_FrameEscaped(0x00); // 0x12 - MessageGroup_ID, immer 0 -> Bootloader
            for (int i = 0; i < msg.length(); i++)
              push_FrameEscaped(msg[i]);
            transmit(transmitTimerLastAction_SendMessageBL);
            break;
          }
        }//end switch
        break;
      }
    }
  }
  void MainWindow::send_XBeeATND() {
    push_Frame(0x7E); // Startsymbol
    push_FrameEscaped(0x00); // Length MSB
    push_FrameEscaped(0x04); // Length LSB
    push_FrameEscaped(0x08); // API AT-Command
    push_FrameEscaped(0x52); // Frame ('R')
    push_FrameEscaped(0x4E); // AT-Command 'N'
    push_FrameEscaped(0x44); // AT-Command 'D'
    transmit(transmitTimerLastAction_XBeeCommand);
  }
  void MainWindow::send_XBeeCommand(QByteArray command) {
    QWord length = 2 + command.length();

    push_Frame(0x7E); // Startsymbol
    push_FrameEscaped((QByte) (length >> 8)); // Length MSB
    push_FrameEscaped((QByte) (length >> 0)); // Length LSB
    push_FrameEscaped(0x08); // API_ID - AT_Command
    push_FrameEscaped(0x01); // Frame_ID - immer 0 -> kein ResponsePaket vom XBee
    for (int i = 0; i < command.length(); i++)
      push_FrameEscaped(command[i]);
    transmit(transmitTimerLastAction_XBeeCommand);
  }
  void MainWindow::send_XBeeRemoteCommand(QByte command[], int commandLength) {
    QWord length = 13 + commandLength;

    push_Frame(0x7E); //  1: Startsymbol
    push_FrameEscaped((QByte) (length >> 8)); //  2: Length MSB
    push_FrameEscaped((QByte) (length >> 0)); //  3: Length LSB
    push_FrameEscaped(0x17); //  4: API-ID 'RemoteCommand'
    push_FrameEscaped(0x00); //  5: Frame-ID '0x00' - kein ResponsePaket vom XBee
    push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 7 * 8)); //  6: 64_Bit_Destination_Network_Address
    push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 6 * 8)); //  7:
    push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 5 * 8)); //  8:
    push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 4 * 8)); //  9:
    push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 3 * 8)); // 10:
    push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 2 * 8)); // 11:
    push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 1 * 8)); // 12:
    push_FrameEscaped((QByte) (ECB_XBeeAddress64 >> 0 * 8)); // 13:
    push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 1 * 8)); // 14: 16_Bit_Destination_Network_Address
    push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 0 * 8)); // 15:
    push_FrameEscaped(0x02); // 16: Options '0x02 - apply changes'
    // Command-Data
    for (int i = 0; i < commandLength; i++)
      push_FrameEscaped(command[i]);
    transmit(transmitTimerLastAction_XBeeRemoteCommand);
  }
  void MainWindow::send_ECB_Reset() {
    switch (applicationMode) {
      case APPLICATION_MODE_USART_Adapter: {
        // Cable-Mode, sende nur eine Nachricht an den Atmega8.
        QByteArray msg;
        msg.append(MsgCode_ECB_AtMega128_ResetCableMode);
        send_Message(msg);
        break;
      }
      case APPLICATION_MODE_XBEE_Adapter: {
        switch (USBDeviceXBeeType) {
          case XBeeType_Serie1: {
            if (ECB_XBeeAddress16 == 0xFFFE && ECB_XBeeAddress64 == 0x000000000000FFFF) {
              sl_TextLog("Bitte erst einen Knoten waehlen!");
              return;
            }

            QByte command_d0_activ_high[] = { (QByte) 'D', (QByte) '0', 0x05 };
            QByte command_d0_off[] = { (QByte) 'D', (QByte) '0', 0x00 };

            // Sende XBeeRemoteCommand und Setze D0 auf 'Digital output, default low'
            send_XBeeRemoteCommand(command_d0_activ_high, 3);
            sleep(100);
            // Sende XBeeRemoteCommand und Setze D0 auf 'Unmonitored digital input'
            send_XBeeRemoteCommand(command_d0_off, 3);
            break;
          }
          case XBeeType_Serie2: {
            if (ECB_XBeeAddress16 == 0xFFFE && ECB_XBeeAddress64 == 0x000000000000FFFF) {
              sl_TextLog("Bitte erst einen Knoten waehlen!");
              return;
            }

            QByte command_d0_activ_high[] = { (QByte) 'D', (QByte) '0', 0x05 };
            QByte command_d0_off[] = { (QByte) 'D', (QByte) '0', 0x00 };

            // Sende XBeeRemoteCommand und Setze D0 auf 'Digital output, default low'
            send_XBeeRemoteCommand(command_d0_activ_high, 3);
            sleep(100);
            // Sende XBeeRemoteCommand und Setze D0 auf 'Unmonitored digital input'
            send_XBeeRemoteCommand(command_d0_off, 3);
            break;
          }
        }//end switch
      }
    }
  }

  void MainWindow::printBuffer(QByteArray buffer) {
    QString hex;
    QString line;

    for (int i = 0; i < buffer.length(); i++) {
      line.append(QString::number((buffer[i] >> 4) & 0x0F, 16).toUpper());
      line.append(QString::number((buffer[i] >> 0) & 0x0F, 16).toUpper());
      line.append(" ");
    }

    panelLogView->appendLogViewText(line);
  }

  void MainWindow::isp_MessageHandler_Bootloader(QByteArray receiveBuffer) {
    QByte msgCode = receiveBuffer[5];

    switch (msgCode) {
      // BootloaderActions
      //==========================================================================================================
      case MsgCode_IspProgrammer_Bootloader_Start: {
        // Nachrichten-Format:
        // ----------------------
        // 0x00 - StartDelemiter
        // 0x01 - Length_HighByte
        // 0x02 - Length_LowByte
        // 0x03 - API-Identifier
        // ----------------------
        // 0x04 - MsgGroup
        // 0x05 - MsgCode

        sl_TextLog("USB-ISP-Adapter bootloader start");

        switch (nextOperationState) {
          case NEXT_OP_ISP_PROGRAMMER_FLASH_READ: {
            nextOperationState = NEXT_OP_NONE;

            progress = new QProgressDialog("0 Pages", "Cancel", 0, 112, this);
            progress->setWindowTitle("Read Flash ...");
            progress->setWindowModality(Qt::WindowModal);
            progress->setFixedWidth(300);
            progress->show();

            QByteArray msg;
            msg.append((char) Api_ISP_TransmitBootloader);
            msg.append((char) MsgGroup_IspBootloader);
            msg.append((char) MsgCode_IspProgrammer_Bootloader_FlashPageRead);
            msg.append((char) 0);
            msg.append((char) 0);
            msg.append((char) 0);
            msg.append((char) 0);
            send_Message(msg);

            sl_TextLog("Begin Programmer Read Flash");
            break;
          }
          case NEXT_OP_ISP_PROGRAMMER_FLASH_WRITE: {
            nextOperationState = NEXT_OP_NONE;

            progress = new QProgressDialog("0 Pages", "Cancel", 0, 112, this);
            progress->setWindowTitle("Write Flash ...");
            progress->setWindowModality(Qt::WindowModal);
            progress->setFixedWidth(300);
            progress->show();

            QByteArray msg;
            msg.append((char) Api_ISP_TransmitBootloader);
            msg.append((char) MsgGroup_IspBootloader);
            msg.append((char) MsgCode_IspProgrammer_Bootloader_FlashPageWrite);
            msg.append((char) 0);
            msg.append((char) 0);
            msg.append((char) 0);
            msg.append((char) 0);
            msg.append(panelHexViewer->getPage(0, ProgrammerFlashPageSize));
            send_Message(msg);

            sl_TextLog("Begin Programmer Write Flash");
            return;
          }
          default: {
            QByteArray msg;
            msg.append((char) Api_ISP_TransmitBootloader);
            msg.append((char) MsgGroup_IspBootloader);
            msg.append((char) MsgCode_IspProgrammer_Bootloader_Deactivate);
            send_Message(msg);
            break;
          }
        }//switch(nextOperationState)
      }
      case MsgCode_IspProgrammer_Bootloader_End: {
        sl_TextLog("Programmer-Bootloader End");
        if (progress != NULL)
          progress->hide();
        break;
      }
      case MsgCode_ResponsePacket: {
        QByte msgCodeResponse = receiveBuffer[6];

        switch (msgCodeResponse) {
          case MsgCode_IspProgrammer_Bootloader_FlashPageRead: {
            // Nachrichten-Format:
            // ----------------------
            // 0x00 - StartDelemiter
            // 0x01 - Length_HighByte
            // 0x02 - Length_LowByte
            // 0x03 - API-Identifier
            // ----------------------
            // 0x04 - MsgGroup
            // 0x05 - MsgCode (MsgCode_ResponsePaket)
            // 0x06 - MsgResponseCode
            // 0x07 - pageNumber_High
            // 0x08 - pageNumber_Low
            // 0x09 - responseState
            // 0x0A - data... (128Byte)

            int msgLength = ((QByte) receiveBuffer[1] << 8) + (QByte) receiveBuffer[2];
            int pageNumber = ((QByte) receiveBuffer[7] << 8) + (QByte) receiveBuffer[8];
            int pageReadState = (QByte) receiveBuffer[9];

            if (pageReadState != 0) {
              sl_TextLog("BootloaderFlashPage out of bound!");
              progress->cancel();

              QByteArray msg;
              msg.append((char) Api_ISP_TransmitBootloader);
              msg.append((char) MsgGroup_IspBootloader);
              msg.append((char) MsgCode_IspProgrammer_Bootloader_Deactivate);
              send_Message(msg);
              break;
            }

            if (!progress->wasCanceled()) {
              if (pageNumber == 0) {
                int PgmSpaceSize = ProgrammerFlashPageSize * ProgrammerNumberFlashPages;
                temporaryBuffer.resize(PgmSpaceSize);
                temporaryBuffer.fill(0xFF);
              }
              uint startIndex = pageNumber * ProgrammerFlashPageSize;
              for (int i = 0; i < msgLength - 7; i++)
                temporaryBuffer[startIndex + i] = receiveBuffer[10 + i];

              pageNumber++;

              // Die Info für den Nutzer in der Statusleiste
              try {
                progress->setValue(pageNumber);
                progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(ProgrammerNumberFlashPages) + " Pages");
              } catch (...) {
              }

              //naechste Seite
              if (pageNumber < ProgrammerNumberFlashPages) {
                QByteArray msg;
                msg.append((char) Api_ISP_TransmitBootloader);
                msg.append((char) MsgGroup_IspBootloader);
                msg.append((char) MsgCode_IspProgrammer_Bootloader_FlashPageRead);
                msg.append((char) pageNumber >> 8);
                msg.append((char) pageNumber >> 0);
                msg.append((char) 0);
                msg.append((char) 0);
                send_Message(msg);
              } else {
                sl_TextLog("ProgrammerRead complete.");

                QByteArray msg;
                msg.append((char) Api_ISP_TransmitBootloader);
                msg.append((char) MsgGroup_IspBootloader);
                msg.append((char) MsgCode_IspProgrammer_Bootloader_Deactivate);
                send_Message(msg);

                panelHexViewer->setBinary(temporaryBuffer);
                temporaryBuffer.clear();
                action_Binary_save->setEnabled(true);
              }
            } else {
              // Benutzer-Abbruch
              sl_TextLog("User-Abbort");

              QByteArray msg;
              msg.append((char) Api_ISP_TransmitBootloader);
              msg.append((char) MsgGroup_IspBootloader);
              msg.append((char) MsgCode_IspProgrammer_Bootloader_Deactivate);
              send_Message(msg);
            }
            break;
          }
          case MsgCode_IspProgrammer_Bootloader_FlashPageWrite: {
            // Nachrichten-Format:
            // ----------------------
            // 0x00 - StartDelemiter
            // 0x01 - Length_HighByte
            // 0x02 - Length_LowByte
            // 0x03 - API-Identifier
            // ----------------------
            // 0x04 - MsgGroup
            // 0x05 - MsgCode (MsgCode_ResponsePaket)
            // 0x06 - MsgResponseCode
            // 0x07 - pageNumber_High
            // 0x08 - pageNumber_Low
            // 0x09 - responseState

            int pageNumber = ((int) receiveBuffer[7]) * 256 + (int) receiveBuffer[8];
            int responseState = receiveBuffer[9];

            if (responseState == 0 && !progress->wasCanceled()) {
              // Wenn kein Benutzer-Abbruch, dann weiter ...

              // waehle naechste Seitennummer
              pageNumber++;

              // Ueberspringe alle leeren Seiten
              while (true) {
                if (!panelHexViewer->hasPage(pageNumber, ProgrammerFlashPageSize))
                  break;
                if (!panelHexViewer->isPageEmpty(pageNumber, ProgrammerFlashPageSize))
                  break;
                if (!(pageNumber < ProgrammerNumberFlashPages))
                  break;
                pageNumber++;
              }

              // Die Info für den Nutzer in der Statusleiste
              try {
                progress->setValue(pageNumber);
                progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(ProgrammerNumberFlashPages) + " Pages");
              } catch (...) {
              }

              if ((pageNumber < ProgrammerNumberFlashPages) && panelHexViewer->hasPage(pageNumber, ProgrammerFlashPageSize)) {
                if (panelHexViewer->hasPage(pageNumber, ProgrammerFlashPageSize)) {
                  QByteArray msg;
                  msg.append((char) Api_ISP_TransmitBootloader);
                  msg.append((char) MsgGroup_IspBootloader);
                  msg.append((char) MsgCode_IspProgrammer_Bootloader_FlashPageWrite);
                  msg.append((char) pageNumber >> 8);
                  msg.append((char) pageNumber >> 0);
                  msg.append((char) 0);
                  msg.append((char) 0);
                  msg.append(panelHexViewer->getPage(pageNumber, ProgrammerFlashPageSize));
                  send_Message(msg);
                }
              } else {
                QByteArray msg;
                msg.append((char) Api_ISP_TransmitBootloader);
                msg.append((char) MsgGroup_IspBootloader);
                msg.append((char) MsgCode_IspProgrammer_Bootloader_Deactivate);
                send_Message(msg);

                sl_TextLog("Write Programmer complete.");
                progress->hide();
              }
            } else {
              // Schreiben der Seite nicht erfolgreich!!!
              QByteArray msg;
              msg.append((char) Api_ISP_TransmitBootloader);
              msg.append((char) MsgGroup_IspBootloader);
              msg.append((char) MsgCode_IspProgrammer_Bootloader_Deactivate);
              send_Message(msg);

              sl_TextLog("Write Programmer abbort, error");
            }
            break;
          }
        }
        break;
      }// case
    } // switch(msgCode)
  }

  void MainWindow::isp_MessageHandler_Firmware(QByteArray receiveBuffer) {
    // Nachrichten-Format:
    // ----------------------
    // 0x00 - StartDelemiter
    // 0x01 - Length_HighByte
    // 0x02 - Length_LowByte
    // 0x03 - API-Identifier
    // ----------------------
    // 0x04 - MsgGroup
    // 0x05 - MsgCode
    // 0x05 - MsgCode_response

    uint msgLength = ((QByte) receiveBuffer[1] << 8) + ((QByte) receiveBuffer[2]);
    uint msgCode = ((QByte) receiveBuffer[5]);

    if (msgCode == MsgCode_ResponsePacket) {

      uint msgResponseCode = ((QByte) receiveBuffer[6]);

      switch (msgResponseCode) {
        case MsgCode_IspProgrammer_Firmware_SoftwareVersionRead: {
          // Nachrichten-Format:
          // ----------------------
          // 0x00 - StartDelemiter
          // 0x01 - Length_HighByte
          // 0x02 - Length_LowByte
          // 0x03 - API-Identifier
          // ----------------------
          // 0x04 - MsgGroup
          // 0x05 - MsgCode_ResponsePaket
          // 0x06 - MsgCode_IspProgrammer_Firmware_SoftwareVersionRead
          // 0x07 - some chars ....
          sl_TextLog("USB-ISP-Adapter: SoftwareVersion = " + QString(receiveBuffer.mid(7, msgLength - 4)));
          break;
        }
          // TargetDeviceActions
          //==========================================================================================================
        case MsgCode_IspProgrammer_TargetDevice_BeginTransaction: // Bestätigung erfolgreich/nicht erfolgreich EnableProgrammingMode
        {
          // Nachrichten-Format:
          // ----------------------
          // 0x00 - StartDelemiter
          // 0x01 - Length_HighByte
          // 0x02 - Length_LowByte
          // 0x03 - API-Identifier
          // ----------------------
          // 0x04 - MsgGroup
          // 0x05 - MsgCode_ResponsePaket
          // 0x06 - msgCode
          // 0x07 - returnCode
          // 0x08 - actionCommand

          uint returnCode = (QByte) receiveBuffer[7];
          uint actionCommand = (QByte) receiveBuffer[8];

          if (returnCode == 0) {
            // Fuehre Aktionen durch wie: ReadFuseBits, ReadSignatureBytes, Read/Write Pages ...
            switch (actionCommand) {
              case MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead: {
                sl_TextLog("Begin Read SignatureBytes");
                QByteArray msg;
                msg.append((char) Api_ISP_TransmitFirmware);
                msg.append((char) MsgGroup_IspFirmware);
                msg.append((char) MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead);
                send_Message(msg);
                timerParams = transmitTimerLastAction_SendMessageISP;
                break;
              }
              case MsgCode_IspProgrammer_TargetDevice_CalibrationBytesRead: {
                sl_TextLog("Begin Read CalibrationBytes");
                QByteArray msg;
                msg.append((char) Api_ISP_TransmitFirmware);
                msg.append((char) MsgGroup_IspFirmware);
                msg.append((char) MsgCode_IspProgrammer_TargetDevice_CalibrationBytesRead);
                send_Message(msg);
                timerParams = transmitTimerLastAction_SendMessageISP;
                break;
              }
              case MsgCode_IspProgrammer_TargetDevice_FuseBitsRead:
                if (avrDevice != 0) {
                  sl_TextLog("Begin Read FuseBits");
                  QByteArray msg;
                  msg.append((char) Api_ISP_TransmitFirmware);
                  msg.append((char) MsgGroup_IspFirmware);
                  msg.append((char) MsgCode_IspProgrammer_TargetDevice_FuseBitsRead);
                  msg.append((char) avrDevice->NumberFuseBytes);
                  send_Message(msg);
                  timerParams = transmitTimerLastAction_SendMessageISP;
                }
                break;

              case MsgCode_IspProgrammer_TargetDevice_FuseBitsWrite:
                if (avrDevice != 0) {
                  sl_TextLog("Begin Write FuseBits");
                  QByteArray msg;
                  msg.append((char) Api_ISP_TransmitFirmware);
                  msg.append((char) MsgGroup_IspFirmware);
                  msg.append((char) MsgCode_IspProgrammer_TargetDevice_FuseBitsWrite);
                  msg.append((char) avrDevice->NumberFuseBytes);
                  msg.append((char) avrDevice->FuseBytes[0]->Value);
                  msg.append((char) avrDevice->FuseBytes[1]->Value);
                  msg.append((char) avrDevice->FuseBytes[2]->Value);
                  send_Message(msg);
                  timerParams = transmitTimerLastAction_SendMessageISP;
                }
                break;

              case MsgCode_IspProgrammer_TargetDevice_FlashPageRead:
                if (avrDevice != 0) {
                  progress = new QProgressDialog("0 Pages", "Cancel", 0, avrDevice->NumberPages, this);
                  progress->setWindowTitle("Read Flash ...");
                  progress->setWindowModality(Qt::WindowModal);
                  progress->setFixedWidth(300);
                  progress->show();

                  sl_TextLog("Begin Read Flash");
                  QByteArray msg;
                  msg.append((char) Api_ISP_TransmitFirmware);
                  msg.append((char) MsgGroup_IspFirmware);
                  msg.append((char) MsgCode_IspProgrammer_TargetDevice_FlashPageRead);
                  msg.append((char) 0);
                  msg.append((char) 0);
                  msg.append((char) (avrDevice->PageSizeBytes >> 8));
                  msg.append((char) (avrDevice->PageSizeBytes >> 0));
                  send_Message(msg);
                  timerParams = transmitTimerLastAction_SendMessageISP;
                }
                break;

              case MsgCode_IspProgrammer_TargetDevice_FlashPageWrite:
                if (avrDevice != 0) {
                  progress = new QProgressDialog("0 Pages", "Cancel", 0, avrDevice->NumberPages, this);
                  progress->setWindowTitle("Write Flash ...");
                  progress->setWindowModality(Qt::WindowModal);
                  progress->setFixedWidth(300);
                  progress->show();

                  sl_TextLog("Begin Chip Erase");
                  QByteArray msg;
                  msg.append((char) Api_ISP_TransmitFirmware);
                  msg.append((char) MsgGroup_IspFirmware);
                  msg.append((char) MsgCode_IspProgrammer_TargetDevice_ChipErase);
                  msg.append((char) MsgCode_IspProgrammer_TargetDevice_FlashPageWrite);
                  send_Message(msg);
                  timerParams = transmitTimerLastAction_SendMessageISP;
                }
                break;

            }// switch
          }// if
          else {
            // Die Aktiviereung des Programmier-Modus war nicht erfolgreich.
            // Es koennen keine Aktionen durchgefuehrt werden!
            // Moegliche Ursachen:
            //  - kein Taget vorhanden
            //  - Programmer nicht mit Target verbunden
            //  - Taget ohne Betriebsspannungsversorgung
            //  - Target mit zu niedrigem, oder ganz ohne Takt
            sl_TextLog("Error TargetDevice-InitProgrammMode, please check Connection or TargetSpeed");
          }
          break;
        }
        case MsgCode_IspProgrammer_TargetDevice_EndTransaction: // Bestaetigung erfolgreich/nicht erfolgreich EnableProgrammingMode
        {
          if (progress != 0) {
            progress->setValue(progress->maximum());
          }
          sl_TextLog("Function completed.");
          break;
        }
        case MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead: {
          // Nachrichten-Format:
          // ----------------------
          // 0x00 - StartDelemiter
          // 0x01 - Length_HighByte
          // 0x02 - Length_LowByte
          // 0x03 - API-Identifier
          // ----------------------
          // 0x04 - MsgGroup
          // 0x05 - MsgCode_ResponsePaket
          // 0x06 - MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead
          // 0x07 - signature_0 (MSB)
          // 0x08 - signature_1
          // 0x09 - signature_2 (LSB)
          uint targetSignature = ((QByte) receiveBuffer[7] << 16) + ((QByte) receiveBuffer[8] << 8) + ((QByte) receiveBuffer[9] << 0);

          if (debug)
            sl_TextLog("Signature = 0x" + QString::number(targetSignature, 16).toUpper());

          avrDevice = avrDeviceList->getDevice(targetSignature);

          if (avrDevice != 0) {
            panelSetting->setAVRDeviceName(*avrDevice->Name);
            sl_TextLog(*avrDevice->Name);
            hasAVRDeviceIdedentified = true;
            action_Target_CalibrationBytes_read->setEnabled(true);
            action_Target_ShowFuseDialog->setEnabled(true);
            //action_Target_FuseBytes_write->setEnabled(true);
            action_Target_Flash_read->setEnabled(true);
            if (hasBinary) {
              action_Target_Flash_write->setEnabled(true);
              action_Target_Flash_update_write->setEnabled(true);
            }
          } else {
            QString s = "unknown target device";
            sl_TextLog(s);
            panelSetting->setAVRDeviceName(s);
            hasAVRDeviceIdedentified = false;

            action_Target_CalibrationBytes_read->setEnabled(false);
            action_Target_ShowFuseDialog->setEnabled(false);
            action_Target_Flash_read->setEnabled(false);
            action_Target_Flash_write->setEnabled(false);
            action_Target_Flash_update_write->setEnabled(false);
          }

          QByteArray msg;
          msg.append((char) Api_ISP_TransmitFirmware);
          msg.append((char) MsgGroup_IspFirmware);
          msg.append((char) MsgCode_IspProgrammer_TargetDevice_EndTransaction);
          send_Message(msg);
          break;
        }
        case MsgCode_IspProgrammer_TargetDevice_CalibrationBytesRead: {
          // Nachrichten-Format:
          // ----------------------
          // 0x00 - StartDelemiter
          // 0x01 - Length_HighByte
          // 0x02 - Length_LowByte
          // 0x03 - API-Identifier
          // ----------------------
          // 0x04 - MsgGroup
          // 0x05 - MsgCode_ResponsePaket
          // 0x06 - MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead
          // 0x07 - CalibrationByte_1MHz
          // 0x08 - CalibrationByte_2MHz
          // 0x09 - CalibrationByte_4MHz
          // 0x0A - CalibrationByte_8MHz

          uint targetCalibrationByte_1 = (QByte) receiveBuffer[7];
          uint targetCalibrationByte_2 = (QByte) receiveBuffer[8];
          uint targetCalibrationByte_3 = (QByte) receiveBuffer[9];
          uint targetCalibrationByte_4 = (QByte) receiveBuffer[10];

          sl_TextLog("CalibrationsBytes received:");
          sl_TextLog("  1MHz: 0x" + QString::number(targetCalibrationByte_1, 16).toUpper());
          sl_TextLog("  2MHz: 0x" + QString::number(targetCalibrationByte_2, 16).toUpper());
          sl_TextLog("  4MHz: 0x" + QString::number(targetCalibrationByte_3, 16).toUpper());
          sl_TextLog("  8MHz: 0x" + QString::number(targetCalibrationByte_4, 16).toUpper());

          QByteArray msg;
          msg.append((char) Api_ISP_TransmitFirmware);
          msg.append((char) MsgGroup_IspFirmware);
          msg.append((char) MsgCode_IspProgrammer_TargetDevice_EndTransaction);
          send_Message(msg);

          QString info;
          info.append("TargetDevice='" + *avrDevice->Name + "'\n");
          info.append("1MHz: 0x" + QString::number(targetCalibrationByte_1, 16).toUpper() + "\n");
          info.append("2MHz: 0x" + QString::number(targetCalibrationByte_2, 16).toUpper() + "\n");
          info.append("4MHz: 0x" + QString::number(targetCalibrationByte_3, 16).toUpper() + "\n");
          info.append("8MHz: 0x" + QString::number(targetCalibrationByte_4, 16).toUpper() + "\n");

          QMessageBox msgBox;
          msgBox.setText("Calibration Bytes:");
          msgBox.setInformativeText(info);
          msgBox.exec();

          break;
        }
        case MsgCode_IspProgrammer_TargetDevice_FuseBitsRead: {
          // Nachrichten-Format:
          // ----------------------
          // 0x00 - StartDelemiter
          // 0x01 - Length_HighByte
          // 0x02 - Length_LowByte
          // 0x03 - API-Identifier
          // ----------------------
          // 0x04 - MsgGroup
          // 0x05 - MsgCode_ResponsePaket
          // 0x06 - MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead
          // 0x07 - fuseBits
          // 0x08 - fuseBitsHigh
          // 0x09 - fuseBitsExt

          uint targetFuseBits_0 = (QByte) (receiveBuffer[7]);
          uint targetFuseBits_1 = (QByte) (receiveBuffer[8]);
          uint targetFuseBits_2 = (QByte) (receiveBuffer[9]);

          sl_TextLog("FuseBits received:");
          sl_TextLog("  Low: 0x" + QString::number(targetFuseBits_0, 16).toUpper());
          sl_TextLog(" High: 0x" + QString::number(targetFuseBits_1, 16).toUpper());
          sl_TextLog("  Ext: 0x" + QString::number(targetFuseBits_2, 16).toUpper());

          if (avrDevice != 0) {
            avrDevice->FuseBytes[0]->Value = targetFuseBits_0;
            avrDevice->FuseBytes[1]->Value = targetFuseBits_1;
            avrDevice->FuseBytes[2]->Value = targetFuseBits_2;
            avrFuseDialog->setAvrDevice(avrDevice);
          }

          QByteArray msg;
          msg.append((char) Api_ISP_TransmitFirmware);
          msg.append((char) MsgGroup_IspFirmware);
          msg.append((char) MsgCode_IspProgrammer_TargetDevice_EndTransaction);
          send_Message(msg);
          break;
        }
        case MsgCode_IspProgrammer_TargetDevice_FuseBitsWrite: {
          QByteArray msg;
          msg.append((char) Api_ISP_TransmitFirmware);
          msg.append((char) MsgGroup_IspFirmware);
          msg.append((char) MsgCode_IspProgrammer_TargetDevice_EndTransaction);
          send_Message(msg);
          break;
        }
        case MsgCode_IspProgrammer_TargetDevice_FlashPageRead: {
          // Nachrichten-Format:
          // ----------------------
          // 0x00 - StartDelemiter
          // 0x01 - Length_HighByte
          // 0x02 - Length_LowByte
          // 0x03 - API-Identifier
          // ----------------------
          // 0x04 - MsgGroup
          // 0x05 - MsgCode_ResponsePaket
          // 0x06 - MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead
          // 0x07 - pageNumberH
          // 0x08 - pageNumberL
          // 0x09 - responseState
          // 0x0A - pageData...

          uint pageNumber = ((QByte) receiveBuffer[7] << 8) + (QByte) receiveBuffer[8];
          uint pageReadState = (QByte) receiveBuffer[9];

          if (avrDevice != 0 && pageReadState == 0) {
            if (!progress->wasCanceled()) {
              if (pageNumber == 0) {
                int PgmSpaceSize = avrDevice->PageSizeBytes * avrDevice->NumberPages;
                temporaryBuffer.resize(PgmSpaceSize);
                temporaryBuffer.fill(0xFF);
              }
              int startIndex = pageNumber * avrDevice->PageSizeBytes;
              for (uint i = 0; i < msgLength - 7; i++)
                temporaryBuffer[startIndex + i] = receiveBuffer[10 + i];

              pageNumber++;

              // Die Info für den Nutzer in der Statusleiste
              try {
                progress->setValue(pageNumber);
                progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(avrDevice->NumberPages) + " Pages");
              } catch (...) {
              }

              //naechste Seite
              if (pageNumber < avrDevice->NumberPages) {
                QByteArray msg;
                msg.append((char) Api_ISP_TransmitFirmware);
                msg.append((char) MsgGroup_IspFirmware);
                msg.append((char) MsgCode_IspProgrammer_TargetDevice_FlashPageRead);
                msg.append((char) (pageNumber >> 8));
                msg.append((char) (pageNumber >> 0));
                msg.append((char) (avrDevice->PageSizeBytes >> 8));
                msg.append((char) (avrDevice->PageSizeBytes >> 0));
                send_Message(msg);
              } else {
                sl_TextLog("DeviceRead complete.");
                QByteArray msg;
                msg.append((char) Api_ISP_TransmitFirmware);
                msg.append((char) MsgGroup_IspFirmware);
                msg.append((char) MsgCode_IspProgrammer_TargetDevice_EndTransaction);
                send_Message(msg);
                panelHexViewer->setBinary(temporaryBuffer);
                temporaryBuffer.clear();

                action_Binary_save->setEnabled(true);
              }
            } else {
              // Benutzer-Abbruch
              sl_TextLog("User-Abbort");
              QByteArray msg;
              msg.append((char) Api_ISP_TransmitFirmware);
              msg.append((char) MsgGroup_IspFirmware);
              msg.append((char) MsgCode_IspProgrammer_TargetDevice_EndTransaction);
              send_Message(msg);
            }
          } else {
            //if(avrDevice!=0 || pageReadError)
            sl_TextLog("PageReadError, pageNumber=" + QString::number(pageNumber, 16));
          }
          break;
        }
        case MsgCode_IspProgrammer_TargetDevice_FlashPageWrite: {
          // Nachrichten-Format:
          // ----------------------
          // 0x00 - StartDelemiter
          // 0x01 - Length_HighByte
          // 0x02 - Length_LowByte
          // 0x03 - API-Identifier
          // ----------------------
          // 0x04 - MsgGroup
          // 0x05 - MsgCode_ResponsePaket
          // 0x06 - MsgCode_IspProgrammer_TargetDevice_SignatureBytesWrite
          // 0x07 - pageNumberH
          // 0x08 - pageNumberL
          // 0x09 - responseState

          uint pageNumber = ((QByte) receiveBuffer[7] << 8) + (QByte) receiveBuffer[8];
          uint responseState = (QByte) receiveBuffer[9];

          if (avrDevice != 0) {
            if (responseState == 0 && !progress->wasCanceled()) {
              // Wenn kein Benutzer-Abbruch, dann weiter ...

              // waehle naechste Seitennummer
              pageNumber++;

              // Ueberspringe alle leeren Seiten
              while (true) {
                // Zähle die Seitennummer so lange hoch, wie
                // Seiten vorhanden sind, diese nicht leer sind und
                // die Seitennummer kleiner der maximalen Seitennummer
                // des Zielsystemes betraegt!!!
                //
                // Damit werden dann nur Seiten geschrieben die nicht leer sind.
                // (In einer leeren Seite sind alle Bits gesetzt, jedes Byte betraegt alse 0xFF)
                if (!panelHexViewer->hasPage(pageNumber, avrDevice->PageSizeBytes))
                  break;
                if (!panelHexViewer->isPageEmpty(pageNumber, avrDevice->PageSizeBytes))
                  break;
                if (!(pageNumber < avrDevice->NumberPages))
                  break;
                pageNumber++;
              }

              // Die Info für den Nutzer in der Statusleiste
              try {
                progress->setValue(pageNumber);
                progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(avrDevice->NumberPages) + " Pages");
              } catch (...) {
              }

              if ((pageNumber < avrDevice->NumberPages) && panelHexViewer->hasPage(pageNumber, avrDevice->PageSizeBytes)) {
                if (panelHexViewer->hasPage(pageNumber, avrDevice->PageSizeBytes)) {
                  QByteArray msg;
                  msg.append((char) Api_ISP_TransmitFirmware);
                  msg.append((char) MsgGroup_IspFirmware);
                  msg.append((char) MsgCode_IspProgrammer_TargetDevice_FlashPageWrite);
                  msg.append((char) (pageNumber >> 8));
                  msg.append((char) (pageNumber >> 0));
                  msg.append((char) (avrDevice->PageSizeBytes >> 8));
                  msg.append((char) (avrDevice->PageSizeBytes >> 0));
                  msg.append(panelHexViewer->getPage(pageNumber, avrDevice->PageSizeBytes));
                  send_Message(msg);
                }
              } else {
                QByteArray msg;
                msg.append((char) Api_ISP_TransmitFirmware);
                msg.append((char) MsgGroup_IspFirmware);
                msg.append((char) MsgCode_IspProgrammer_TargetDevice_EndTransaction);
                send_Message(msg);
                sl_TextLog("Write Target-Device complete.");
              }
            } else {
              if (progress != NULL)
                progress->hide();

              // Schreiben der Seite nicht erfolgreich!!!
              sl_TextLog("Write Target-Device abbort, error");
              QByteArray msg;
              msg.append((char) Api_ISP_TransmitFirmware);
              msg.append((char) MsgGroup_IspFirmware);
              msg.append((char) MsgCode_IspProgrammer_TargetDevice_EndTransaction);
              send_Message(msg);
            }
          }
          break;
        }
        case MsgCode_IspProgrammer_TargetDevice_ChipErase: {
          sl_TextLog("Chip Erase complete.");

          // Nachrichten-Format:
          // ----------------------
          // 0x00 - StartDelemiter
          // 0x01 - Length_HighByte
          // 0x02 - Length_LowByte
          // 0x03 - API-Identifier
          // ----------------------
          // 0x04 - MsgGroup
          // 0x05 - MsgCode_ResponsePaket
          // 0x06 - MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead
          // 0x07 - actionCommand (evt. FlashPageWrite ?)
          // 0x08 - fuseBitsHigh
          // 0x09 - fuseBitsExt

          uint actionCommand = receiveBuffer[7];

          // Nach dem ChipErase mit Schreiben beginnen?
          if (0 < actionCommand) {
            // Beginne Schreiben mit der ersten nicht leeren Seite
            uint pageNumber = 0;
            // Ueberspringe alle leeren Seiten
            while (true) {
              // Zähle die Seitennummer so lange hoch, wie
              // Seiten vorhanden sind, diese nicht leer sind und
              // die Seitennummer kleiner der maximalen Seitennummer
              // des Zielsystemes betraegt!!!
              //
              // Damit werden dann nur Seiten geschrieben die nicht leer sind.
              // (In einer leeren Seite sind alle Bits gesetzt, jedes Byte betraegt alse 0xFF)
              if (!panelHexViewer->hasPage(pageNumber, avrDevice->PageSizeBytes))
                break;
              if (!panelHexViewer->isPageEmpty(pageNumber, avrDevice->PageSizeBytes))
                break;
              if (!(pageNumber < avrDevice->NumberPages))
                break;
              pageNumber++;
            }
            sl_TextLog("Begin Write Flash");
            QByteArray msg;
            msg.append((char) Api_ISP_TransmitFirmware);
            msg.append((char) MsgGroup_IspFirmware);
            msg.append((char) MsgCode_IspProgrammer_TargetDevice_FlashPageWrite);
            msg.append((char) (pageNumber >> 8));
            msg.append((char) (pageNumber >> 0));
            msg.append((char) (avrDevice->PageSizeBytes >> 8));
            msg.append((char) (avrDevice->PageSizeBytes >> 0));
            msg.append(panelHexViewer->getPage(pageNumber, avrDevice->PageSizeBytes));
            send_Message(msg);
          }
          break;
        }
        default: {
          sl_TextLog("ReturnCode = 0x" + QString::number((QByte) msgResponseCode, 16).toUpper());
          break;
        }
      }//switch(msgCode)
    } else {
      sl_TextLog("ReturnCode = 0x" + QString::number((QByte) msgCode, 16).toUpper());
    }
  }
  void MainWindow::bl_MessageHandler_Bootloader(QByteArray received_msg) {
    // Nachrichten-Format:
    // ----------------------
    // 0x00 - MsgGroup
    // 0x01 - MsgCode
    // 0x02 - data/parameter ...
    int msgGroup = (QByte) received_msg[0];
    int msgCode = (QByte) received_msg[1];

    //==============================================================================================
    // Werte keine Nachrichten aus, die nicht für den Bootloader sind!
    if (msgGroup != 0x00)
      return;

    switch (msgCode) {
      case MsgCode_ECB_SIGNAL_BootloaderStart: // Bootloader-Start
      {
        sl_TextLog("ECB-Bootloader-Start");

        switch (nextOperationState) {
          case NEXT_OP_ECB_BOOTLOADER_FLASH_READ: {
            nextOperationState = NEXT_OP_NONE;
            ECB_OperationRetries = 0;

            progress = new QProgressDialog("0 Pages", "Cancel", 0, ECB_AtMEGA128_NumberOfPages, this);
            progress->setWindowTitle("Read Flash ...");
            progress->setWindowModality(Qt::WindowModal);
            progress->setFixedWidth(300);
            progress->show();

            sl_TextLog("Begin Read Flash");

            QByteArray msg;
            msg.append((char) MsgCode_ECB_Command_FLASH_PageRead);
            msg.append((char) 0); // pageNumberH
            msg.append((char) 0); // pageNumber
            msg.append((char) 0); // pageSegmentNumber
            send_Message(msg);
            break;
          }
          case NEXT_OP_ECB_BOOTLOADER_FLASH_WRITE: {
            nextOperationState = NEXT_OP_NONE;
            ECB_OperationRetries = 0;

            progress = new QProgressDialog("0 Pages", "Cancel", 0, ECB_AtMEGA128_NumberOfPages, this);
            progress->setWindowTitle("Write Flash ...");
            progress->setWindowModality(Qt::WindowModal);
            progress->setFixedWidth(300);
            progress->show();

            sl_TextLog("Begin Write Flash");
            // Beginne Schreiben mit der ersten nicht leeren Seite
            int pageNumber = 0;
            // Überspringe alle leeren Seiten
            while (true) {
              // Zähle die Seitennummer so lange hoch, wie
              // Seiten vorhanden sind, diese nicht leer sind und
              // die Seitennummer kleiner der maximalen Seitennummer
              // des Zielsystemes betraegt!!!
              //
              // Damit werden dann nur Seiten geschrieben die nicht leer sind.
              // (In einer leeren Seite sind alle Bits gesetzt, jedes QByte betraegt alse 0xFF)
              if (!panelHexViewer->hasPage(pageNumber, ECB_AtMega128_PageSize_Byte))
                break;
              if (!panelHexViewer->isPageEmpty(pageNumber, ECB_AtMega128_PageSize_Byte))
                break;
              if (!(pageNumber < ECB_AtMEGA128_NumberOfPages))
                break;
              pageNumber++;
            }

            // Sende nun die Page an den Bootloader, jedoch in 4 Segmente unterteilt:
            // Das Funkmodul XBeeS2 kann nur maximal 72 Zeichen in einem Packet uebertragen!
            // Sende nun das erste Segment und erwarte daraufhin eine Antwort vom Bootloader!
            // Sende dann auf die Bestätigung vom Bootloader die weiteren Segmente.
            QByteArray msg;
            msg.append((char) MsgCode_ECB_Command_FLASH_PageWrite);
            msg.append((char) (pageNumber >> 8));
            msg.append((char) (pageNumber >> 0));
            msg.append((char) 0);// PageSegmentNumber
            msg.append(panelHexViewer->getPage(pageNumber * 4, ECB_AtMega128_PageSize_Byte / 4));
            send_Message(msg);
            break;
          }
          case NEXT_OP_ECB_BOOTLOADER_EEPROM_READ: {
            nextOperationState = NEXT_OP_NONE;
            ECB_OperationRetries = 0;

            progress = new QProgressDialog("0 Byte", "Cancel", 0, ECB_ATMEGA128_EEPROM_TOTALSIZE, this);
            progress->setWindowTitle("Read EEPROM ...");
            progress->setWindowModality(Qt::WindowModal);
            progress->setFixedWidth(300);
            progress->show();

            sl_TextLog("Begin Read EEPROM");

            QByteArray msg;
            msg.append((char) MsgCode_ECB_Command_EEPROM_BlockRead);
            msg.append((char) 0); // eepromAddressH
            msg.append((char) 0); // eepromAddressL
            send_Message(msg);
            break;
          }
          case NEXT_OP_ECB_BOOTLOADER_EEPROM_WRITE: {
            nextOperationState = NEXT_OP_NONE;
            ECB_OperationRetries = 0;

            if (panelHexViewer->hasPage(0, ECB_ATMEGA128_EEPROM_BlockSize)) {
              sl_TextLog("Begin Write EEPROM");

              progress = new QProgressDialog("0 Byte", "Cancel", 0, ECB_ATMEGA128_EEPROM_TOTALSIZE, this);
              progress->setWindowTitle("Write EEPROM ...");
              progress->setWindowModality(Qt::WindowModal);
              progress->setFixedWidth(300);
              progress->show();

              // Sende nun die Page an den Bootloader, jedoch in 4 Segmente unterteilt:
              // Das Funkmodul XBeeS2 kann nur maximal 72 Zeichen in einem Packet uebertragen!
              // Sende nun das erste Segment und erwarte daraufhin eine Antwort vom Bootloader!
              // Sende dann auf die Bestätigung vom Bootloader die weiteren Segmente.
              QByteArray msg;
              msg.append((char) MsgCode_ECB_Command_EEPROM_BlockWrite);
              msg.append((char) 0); // eepromAddressH
              msg.append((char) 0); // eepromAddressL
              msg.append(panelHexViewer->getPage(0, ECB_ATMEGA128_EEPROM_BlockSize));
              send_Message(msg);
            }
            break;
          }
          default: {
            // Wenn keine der vorherigen Aktionen zutrifft, dann beende den Bootloader!
            QByteArray msg;
            msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
            send_Message(msg);
          }
        }//switch(nextOperationState)
        break;
      }//end case Bootloader_Start


      case MsgCode_ECB_Signal_BootloaderEnd: // Bootloader-Ende
      {
        sl_TextLog("ECB-Bootloader-End");
        if (progress != 0) {
          if (!progress->isHidden()) {
            sl_TextLog("Operation Error.");
            sl_TextLog("Operation NOT complete.");
          }
          progress->hide();
          progress->setValue(progress->maximum());
        }

        break;
      }//end case Bootloader_End
        //-------------------------------------------------------------------------------------------------------------
      case MsgCode_ResponsePacket: {
        // Nachrichten-Format:
        // ----------------------
        // 0x00 - MsgGroup
        // 0x01 - MsgCode_ResponsePacket
        // 0x02 - MsgResponseCode
        // 0x03 - data/parameter ...
        int msgResponseCode = (QByte) received_msg[2];

        switch (msgResponseCode) {
          //-------------------------------------------------------------------------------------------------------------
          // PageRead:
          case MsgCode_ECB_Command_FLASH_PageRead: {
            // Nachrichten-Format:
            // ----------------------
            // 0x00 - MsgGroup
            // 0x01 - MsgCode_ResponsePacket
            // 0x02 - MsgCode_ECB_Command_FlashPageRead
            // 0x03 - pageNumberH
            // 0x04 - pageNumberL
            // 0x05 - pageSegmentNumber
            // 0x06 - operationState
            // 0x07 - pageSegmentData...(64Byte)
            // ----------------------
            uint pageNumber = ((QByte) received_msg[3] << 8) + (QByte) received_msg[4];
            uint pageSegment = (QByte) received_msg[5];
            uint operationState = (QByte) received_msg[6];

            if (operationState != state_OperationSucceeded) {
              // Das angeforderte Segment einer Seite konnte (wiederholt) nicht ausgelesen werden.
              // Erhöhe den Wiederholungszähler
              ECB_OperationRetries++;

              // Solange die maximale Wiederholrate (für erfolglose Operationen) noch nicht erreicht wurde,
              // wird die Operation wiederholt.
              if (ECB_OperationRetries < ECB_OperationRetriesMax) {
                // Sende erneut Anfrage zum Auslesen dieser Seite aus dem Flash!
                QByteArray msg;
                msg.append((char) MsgCode_ECB_Command_FLASH_PageRead);
                msg.append((char) (pageNumber >> 8));
                msg.append((char) (pageNumber >> 0));
                msg.append((char) pageSegment);// PageSegmentNumber
                send_Message(msg);
              } else {
                printMessageErrorCode(operationState);
                sl_TextLog("Read Flash Error.");
                sl_TextLog("Read End.");
                QByteArray msg;
                msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                send_Message(msg);
                if (progress != 0) {
                  progress->hide();
                  progress->setValue(progress->maximum());
                }
              }
            } else {
              // PageRead: Das angeforderte Segment einer Seite wurde erfolgreich aus
              // dem Programmspeicher ausgelesen und übersendet.
              // Fahre mit dem nächsten Segment/ der nächsten Seite fort.

              int startIndex = pageNumber * ECB_AtMega128_PageSize_Byte + 64 * pageSegment;

              // Gab es einen Benutzer-Abbruch?
              if (progress->wasCanceled()) {
                // Benutzer-Abbruch
                sl_TextLog("User-Abbort");
                QByteArray msg;
                msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                send_Message(msg);
              } else {
                // Kein Benutzer-Abbruch, also weiter ...

                // Wurde der Speicherbereich zur Aufnahme des Programmspeicher aus dem ECB schon initialisiert?
                if (temporaryBuffer.length() == 0) {
                  // Reserviere einen Speicherbereich von der Größe des Programmspeicher des Mikrokontroller!
                  int iFlashSize = ECB_AtMega128_PageSize_Byte * ECB_AtMEGA128_NumberOfPages;
                  temporaryBuffer.resize(iFlashSize);
                  temporaryBuffer.fill(0xFF);
                }

                // Trage das erhaltene PageSegment in den temporären Puffer ein
                for (int i = 0; i < 64; i++)
                  temporaryBuffer[startIndex + i] = received_msg[7 + i];

                // Wenn noch nicht alle Segmente dieser Seite erhalten wurden, dann nächstes Segment vom ECB anfordern!
                if (pageSegment < 3) {
                  ECB_OperationRetries = 0; // Setze den Wiederholungszähler für Misserfolge zurück!
                  // Fordere das nächste Segment an!
                  QByteArray msg;
                  msg.append((char) MsgCode_ECB_Command_FLASH_PageRead);
                  msg.append((char) (pageNumber >> 8));
                  msg.append((char) (pageNumber >> 0));
                  msg.append((char) pageSegment + 1);// PageSegmentNumber
                  send_Message(msg);
                  break;
                }

                // Fordere nun die nächste Seite an.
                pageNumber++;

                // Die Info für den Nutzer in der Statusleiste
                try {
                  progress->setValue(pageNumber);
                  progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(ECB_AtMEGA128_NumberOfPages) + " Pages");
                } catch (...) {
                }

                // Nächste Seite anfordern, wenn noch nicht alle Seiten ausgelesen wurden!
                if (pageNumber < ECB_AtMEGA128_NumberOfPages) {
                  QByteArray msg;
                  msg.append((char) MsgCode_ECB_Command_FLASH_PageRead);
                  msg.append((char) (pageNumber >> 8));
                  msg.append((char) (pageNumber >> 0));
                  msg.append((char) 0);// PageSegmentNumber
                  send_Message(msg);
                } else {
                  // Der Programmspeicher des Mikrokontroller wurde kommplett ausgelesen.
                  sl_TextLog("Read complete.");
                  panelHexViewer->setBinary(temporaryBuffer);
                  action_Binary_save->setEnabled(true);
                  temporaryBuffer.clear();
                  QByteArray msg;
                  msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                  send_Message(msg);
                  if (progress != 0) {
                    progress->hide();
                    progress->setValue(progress->maximum());
                  }
                } // nächste Seite Anfordern
              } // kein Benutzer-Abbruch
            } // keine Fehlermeldung
            break;
          }
            //-------------------------------------------------------------------------------------------------------------
            // PageWrite
          case MsgCode_ECB_Command_FLASH_PageWrite: {
            // Nachrichten-Format:
            // ----------------------
            // 0x00 - MsgGroup
            // 0x01 - MsgCode_ResponsePacket
            // 0x02 - MsgCode_ECB_Command_FlashPageWrite
            // 0x03 - pageNumberH
            // 0x04 - pageNumberL
            // 0x05 - pageSegmentNumber
            // 0x06 - responseState (operation succeed vs failed)
            // ----------------------
            uint pageNumber = ((QByte) received_msg[3] << 8) + (QByte) received_msg[4];
            uint pageSegment = (QByte) received_msg[5];
            uint operationState = (QByte) received_msg[6];

            switch (operationState) {
              //-------------------------------------------------------------------------------
              case state_OperationSucceeded: {
                // Operation erfolgreich!

                // Gab es einen Benutzer-Abbruch?
                if (progress->wasCanceled()) {
                  // Benutzer-Abbruch: Schreiben der Seite nicht erfolgreich!!!
                  sl_TextLog("Write Flash Abort.");
                  QByteArray msg;
                  msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                  send_Message(msg);
                  if (progress != 0) {
                    progress->hide();
                    progress->setValue(progress->maximum());
                  }
                } else {
                  // Wenn kein Benutzer-Abbruch, dann weiter ...

                  // wähle nächste Seitennummer
                  pageNumber++;

                  // Ueberspringe alle leeren Seiten
                  while (true) {
                    // Zähle die Seitennummer so lange hoch, wie
                    // Seiten vorhanden sind, diese nicht leer sind und
                    // die Seitennummer kleiner der maximalen Seitennummer
                    // des Zielsystemes betraegt!!!
                    //
                    // Damit werden dann nur Seiten geschrieben die nicht leer sind.
                    // (In einer leeren Seite sind alle Bits gesetzt, jedes QByte betraegt alse 0xFF)
                    if (!panelHexViewer->hasPage(pageNumber, ECB_AtMega128_PageSize_Byte))
                      break;
                    if (!panelHexViewer->isPageEmpty(pageNumber, ECB_AtMega128_PageSize_Byte))
                      break;
                    if (!(pageNumber < ECB_AtMEGA128_NumberOfPages))
                      break;
                    pageNumber++;
                  }

                  // Die Info für den Nutzer in der Statusleiste
                  try {
                    progress->setValue(pageNumber);
                    progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(ECB_AtMEGA128_NumberOfPages) + " Pages");
                  } catch (...) {
                  }

                  if ((pageNumber < ECB_AtMEGA128_NumberOfPages) && panelHexViewer->hasPage(pageNumber, ECB_AtMega128_PageSize_Byte)) {
                    // Sende nun die Page an den Bootloader, jedoch in 4 Segmente unterteilt:
                    // Das Funkmodul XBeeS2 kann nur maximal 72 Zeichen in einem Packet uebertragen!
                    // Sende erstes Segment der Seite
                    {
                      ECB_OperationRetries = 0;
                      QByteArray msg;
                      msg.append((char) MsgCode_ECB_Command_FLASH_PageWrite);
                      msg.append((char) (pageNumber >> 8));
                      msg.append((char) (pageNumber >> 0));
                      msg.append((char) pageSegment);// PageSegmentNumber
                      msg.append(panelHexViewer->getPage(pageNumber * 4, ECB_AtMega128_PageSize_Byte / 4));
                      send_Message(msg);
                    }// end for Segments
                  } else {
                    QByteArray msg;
                    msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                    send_Message(msg);
                    sl_TextLog("Write complete.");
                    if (progress != 0) {
                      progress->hide();
                      progress->setValue(progress->maximum());
                    }
                  }
                } // End kein Benutzer-Abbruch
                break;
              }
                //-------------------------------------------------------------------------------
              case state_Confirm_PageSegmentReceive: {
                // PageWrite: Das ECB hat den Empfang eines Page-Segmentes bestätigt.
                // Sende nun das nächste Segment. Mit Erhalt des letzten (vierten)
                // Segmentes einer Seite, wird diese in den Programmspeicher geschrieben.

                pageSegment++;
                ECB_OperationRetries = 0;
                QByteArray msg;
                msg.append((char) MsgCode_ECB_Command_FLASH_PageWrite);
                msg.append((char) (pageNumber >> 8));
                msg.append((char) (pageNumber >> 0));
                msg.append((char) pageSegment);// PageSegmentNumber
                msg.append(panelHexViewer->getPage(pageNumber * 4 + pageSegment, ECB_AtMega128_PageSize_Byte / 4));
                send_Message(msg);
                break;
              }
                //-------------------------------------------------------------------------------
              case state_FlashPageWriteError: {
                // Operation war nicht erfolgreich.
                ECB_OperationRetries++;

                // nochmals versuchen?
                if (ECB_OperationRetries < ECB_OperationRetriesMax) {
                  // Sende wiederholt die gesammte Page!
                  // Das Funkmodul XBeeS2 kann nur maximal 72 Zeichen in einem Packet uebertragen!
                  // Übertrage nun das erstes Segment der Page.
                  QByteArray msg;
                  msg.append((char) MsgCode_ECB_Command_FLASH_PageWrite);
                  msg.append((char) (pageNumber >> 8));
                  msg.append((char) (pageNumber >> 0));
                  msg.append((char) pageSegment);// PageSegmentNumber
                  msg.append(panelHexViewer->getPage(pageNumber * 4, ECB_AtMega128_PageSize_Byte / 4));
                  send_Message(msg);
                } else {
                  // Die Seite konnte nicht in en Programmspeicher des ECB geschrieben werden, Abbruch durch Applikation!
                  sl_TextLog("Write Flash Error.");
                  sl_TextLog("Write End.");
                  QByteArray msg;
                  msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                  send_Message(msg);
                  if (progress != 0) {
                    progress->hide();
                    progress->setValue(progress->maximum());
                  }
                } // End - Abbruch, nach Fehler ...
                break;
              }
              default:
                printMessageErrorCode(received_msg[4]);
                sl_TextLog("Write End.");
                QByteArray msg;
                msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                send_Message(msg);
                if (progress != 0) {
                  progress->hide();
                  progress->setValue(progress->maximum());
                }
                break;
                //-------------------------------------------------------------------------------
            }
            break;
          } // End PageWrite
            //-------------------------------------------------------------------------------------------------------------
            // EEPROM-READ
          case MsgCode_ECB_Command_EEPROM_BlockRead: {
            // Nachrichten-Format:
            // ----------------------
            // 0x00 - MsgGroup
            // 0x01 - MsgCode_ResponsePacket
            // 0x02 - MsgCode_ECB_Command_EERPOM_BlockWrite
            // 0x03 - eepromAddressH
            // 0x04 - eepromAddressL
            // 0x05 - responseState (operation succeed vs failed)
            // 0x06 - eeprom-data... (64Byte)
            // ----------------------
            uint eepromBlockStartAddress = ((QByte) received_msg[3] << 8) + (QByte) received_msg[4];
            uint operationState = (QByte) received_msg[5];

            switch (operationState) {
              case state_OperationSucceeded: {
                // Gab es einen Benutzer-Abbruch?
                if (progress->wasCanceled()) {
                  // Benutzer-Abbruch
                  sl_TextLog("User-Abbort");
                  QByteArray msg;
                  msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                  send_Message(msg);
                } else {
                  // Kein Benutzer-Abbruch, also weiter ...

                  // Wurde der Speicherbereich zur Aufnahme des Programmspeicher aus dem ECB schon initialisiert?
                  if (eepromBlockStartAddress == 0) {
                    // Reserviere einen Speicherbereich von der Größe des EEPROM des Mikrokontroller!
                    int iEEPromSize = ECB_ATMEGA128_EEPROM_TOTALSIZE;
                    temporaryBuffer.resize(iEEPromSize);
                    temporaryBuffer.fill(0x00);
                  }

                  // Trage das erhaltene PageSegment in den temporären Puffer ein
                  for (int i = 0; i < ECB_ATMEGA128_EEPROM_BlockSize; i++)
                    temporaryBuffer[eepromBlockStartAddress + i] = received_msg[6 + i];
                  //TODO:
                  sl_TextLog("data copied into tempBuffer.");

                  // Die Info für den Nutzer in der Statusleiste
                  try {
                    progress->setValue(eepromBlockStartAddress);
                    progress->setLabelText(QString::number(eepromBlockStartAddress + ECB_ATMEGA128_EEPROM_BlockSize) + "/" + QString::number(
                        ECB_ATMEGA128_EEPROM_TOTALSIZE) + " Bytes");
                  } catch (...) {
                  }
                  // Nächster Block...
                  eepromBlockStartAddress += ECB_ATMEGA128_EEPROM_BlockSize;

                  // Nächsten Block anfordern, wenn noch nicht der gesamte EEPROM ausgelesen wurden!
                  if (eepromBlockStartAddress <= ECB_ATMEGA128_EEPROM_TOTALSIZE - ECB_ATMEGA128_EEPROM_BlockSize) {
                    QByteArray msg;
                    msg.append((char) MsgCode_ECB_Command_EEPROM_BlockRead);
                    msg.append((char) (eepromBlockStartAddress >> 8));
                    msg.append((char) (eepromBlockStartAddress >> 0));
                    send_Message(msg);
                  } else {
                    // Der Programmspeicher des Mikrokontroller wurde kommplett ausgelesen.
                    sl_TextLog("Read EEPROM complete.");
                    panelHexViewer->setBinary(temporaryBuffer);
                    action_Binary_save->setEnabled(true);
                    temporaryBuffer.clear();
                    QByteArray msg;
                    msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                    send_Message(msg);
                    if (progress != 0) {
                      progress->hide();
                      progress->setValue(progress->maximum());
                    }
                  } // nächste Seite Anfordern
                } // kein Benutzer-Abbruch
                break;
              }
              case state_EEPromAddressNumberError: {
                QByteArray msg;
                msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                send_Message(msg);
                sl_TextLog("Read EEPROM abort, wrong address requested.");
                if (progress != 0) {
                  progress->hide();
                  progress->setValue(progress->maximum());
                }
                break;
              }
            }//switch(operationState)
            break;
          }
            //-------------------------------------------------------------------------------------------------------------
            // EEPROM-WRITE
          case MsgCode_ECB_Command_EEPROM_BlockWrite: {
            // Nachrichten-Format:
            // ----------------------
            // 0x00 - MsgGroup
            // 0x01 - MsgCode_ResponsePacket
            // 0x02 - MsgCode_ECB_Command_EERPOM_BlockWrite
            // 0x03 - eepromAddressH
            // 0x04 - eepromAddressL
            // 0x05 - responseState (operation succeed vs failed)
            // ----------------------
            uint eepromBlockStartAddress = ((QByte) received_msg[3] << 8) + (QByte) received_msg[4];
            uint operationState = (QByte) received_msg[5];

            switch (operationState) {
              case state_OperationSucceeded: {
                // Operation erfolgreich!
                //------------------------------------------
                // Gab es einen Benutzer-Abbruch?
                if (progress->wasCanceled()) {
                  // Benutzer-Abbruch: Schreiben der Seite nicht erfolgreich!!!
                  sl_TextLog("Write EEPROM Abort.");
                  QByteArray msg;
                  msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                  send_Message(msg);
                  if (progress != 0) {
                    progress->hide();
                    progress->setValue(progress->maximum());
                  }
                } else {
                  // Wenn kein Benutzer-Abbruch, dann weiter ...
                  eepromBlockStartAddress += ECB_ATMEGA128_EEPROM_BlockSize;
                  // Die Info für den Nutzer in der Statusleiste
                  try {
                    progress->setValue(eepromBlockStartAddress);
                    progress->setLabelText(QString::number(eepromBlockStartAddress) + "/" + QString::number(ECB_ATMEGA128_EEPROM_TOTALSIZE) + " Bytes");
                  } catch (...) {
                  }

                  if ((eepromBlockStartAddress <= ECB_ATMEGA128_EEPROM_TOTALSIZE - ECB_ATMEGA128_EEPROM_BlockSize) && panelHexViewer->hasPage(
                      eepromBlockStartAddress, ECB_ATMEGA128_EEPROM_BlockSize)) {
                    // Sende nun den Block an den Bootloader
                    {
                      ECB_OperationRetries = 0;
                      QByteArray msg;
                      msg.append((char) MsgCode_ECB_Command_EEPROM_BlockWrite);
                      msg.append((char) (eepromBlockStartAddress >> 8));
                      msg.append((char) (eepromBlockStartAddress >> 0));
                      msg.append(panelHexViewer->getPage(eepromBlockStartAddress, ECB_ATMEGA128_EEPROM_BlockSize));
                      send_Message(msg);
                    }
                  } else {
                    QByteArray msg;
                    msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                    send_Message(msg);
                    sl_TextLog("Write EEPROM complete.");
                    if (progress != 0) {
                      progress->hide();
                      progress->setValue(progress->maximum());
                    }
                  }
                } // End kein Benutzer-Abbruch
                break;
              }
              case state_EEPromAddressNumberError: {
                QByteArray msg;
                msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                send_Message(msg);
                sl_TextLog("Write EEPROM abort, wrong address requested.");
                if (progress != 0) {
                  progress->hide();
                  progress->setValue(progress->maximum());
                }
                break;
              }
              case state_EEPromBlockWriteError: {
                QByteArray msg;
                msg.append((char) MsgCode_ECB_Command_Bootloader_Deactivate);
                send_Message(msg);
                sl_TextLog("Physical write error of EEPROM.");
                if (progress != 0) {
                  progress->hide();
                  progress->setValue(progress->maximum());
                }
                break;
              }
            }
          }
        }
        break;
      }
        //-------------------------------------------------------------------------------------------------------------
      default: {
        QString s;
        s.append(QString::number((QByte) (msgCode >> 4) & 0x0F, 16).toUpper());
        s.append(QString::number((QByte) (msgCode >> 0) & 0x0F, 16).toUpper());
        sl_TextLog("Unknown MessageCode = 0x" + s + " received.");
        break;
      }
    }// end switch
  }
  void MainWindow::bl_MessageHandler_XBeeCommandResponse(QByteArray receiveBuffer) {
    QWord msgLength = (receiveBuffer[1] & 0xFF) << 8 | (receiveBuffer[2] & 0xFF);

    QString Command;

    Command.append((char) receiveBuffer[5]);
    Command.append((char) receiveBuffer[6]);

    if (Command.compare("HV") == 0) {
      quint16 HardwareVersionNumber = 0;

      HardwareVersionNumber += ((quint16) receiveBuffer[8] & 0xFF) << 1 * 8;
      HardwareVersionNumber += ((quint16) receiveBuffer[9] & 0xFF) << 0 * 8;

      switch (HardwareVersionNumber) {
        case 0x180B:
        case 0x1842:
          USBDeviceXBeeType = XBeeType_Serie1;
          panelSetting->setUSBDeviceXBeeType("XBeeSerie1");
          break;
        case 0x1942:
          USBDeviceXBeeType = XBeeType_Serie2;
          panelSetting->setUSBDeviceXBeeType("XBeeSerie2");
          break;
      }
      return;
    }

    if (Command.compare("ND") == 0) // NodeIdentifier-Response
    {
      if ((int) receiveBuffer[7] != 0) // Status OK?
      {
        sl_TextLog("Error occured while identifing nodes.");

        // disable MenuItemsBL;
        action_ECB_reset->setEnabled(false);
        action_ECB_Bootloader_Flash_read->setEnabled(false);
        action_ECB_Bootloader_Flash_write->setEnabled(false);

        return;
      }

      QString nodeId;
      quint16 ECB_XBeeAddress16_tmp = 0;
      quint64 ECB_XBeeAddress64_tmp = 0;

      // Das XBee sendet als antwort auch ein Paket mit Länge 5,
      // jedoch ohne nützliche Informationen
      if (msgLength <= 5)
        return;

      ECB_XBeeAddress16_tmp += ((quint16) receiveBuffer[8] & 0xFF) << 1 * 8;
      ECB_XBeeAddress16_tmp += ((quint16) receiveBuffer[9] & 0xFF) << 0 * 8;

      ECB_XBeeAddress64_tmp += ((quint64) receiveBuffer[10] & 0xFF) << 7 * 8;
      ECB_XBeeAddress64_tmp += ((quint64) receiveBuffer[11] & 0xFF) << 6 * 8;
      ECB_XBeeAddress64_tmp += ((quint64) receiveBuffer[12] & 0xFF) << 5 * 8;
      ECB_XBeeAddress64_tmp += ((quint64) receiveBuffer[13] & 0xFF) << 4 * 8;
      ECB_XBeeAddress64_tmp += ((quint64) receiveBuffer[14] & 0xFF) << 3 * 8;
      ECB_XBeeAddress64_tmp += ((quint64) receiveBuffer[15] & 0xFF) << 2 * 8;
      ECB_XBeeAddress64_tmp += ((quint64) receiveBuffer[16] & 0xFF) << 1 * 8;
      ECB_XBeeAddress64_tmp += ((quint64) receiveBuffer[17] & 0xFF) << 0 * 8;

      nodeId.append("(");
      nodeId.append(QString::number((ECB_XBeeAddress16_tmp >> 12) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress16_tmp >> 8) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress16_tmp >> 4) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress16_tmp >> 0) & 0x0F, 16).toUpper());
      nodeId.append(":");
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 60) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 56) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 52) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 48) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 44) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 40) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 36) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 32) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 28) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 24) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 20) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 16) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 12) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 8) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 4) & 0x0F, 16).toUpper());
      nodeId.append(QString::number((ECB_XBeeAddress64_tmp >> 0) & 0x0F, 16).toUpper());
      nodeId.append(") '");

      // Lese den NodeIdentifier-String aus.
      switch (USBDeviceXBeeType) {
        case XBeeType_Serie1: {
          for (int i = 0; i < msgLength - 17; i++)
            nodeId.append((char) receiveBuffer[19 + i]);
          nodeId.append("'");

          panelSetting->setXBeeRemoteNodeIdentifier(nodeId);
          break;
        }
        case XBeeType_Serie2: {
          for (int i = 18; i < msgLength - 6; i++)
            nodeId.append((char) receiveBuffer[i]);
          nodeId.append("'");

          panelSetting->setXBeeRemoteNodeIdentifier(nodeId);
          break;
        }
      }//end switch

      // Enable MenuItems BL
      action_ECB_reset->setEnabled(true);
      action_ECB_Bootloader_Flash_read->setEnabled(true);
    }
  }

  void MainWindow::printMessageErrorCode(int errorCode) {
    // Werte die Fehlerursache aus.
    switch (errorCode) {
      case state_FlashPageWriteError:
        statusBar()->showMessage("FLASH-WriteError", 5000);
        sl_TextLog("FLASH: Physical Write Error.");
        break;
      case state_FlashPageNumberError:
        statusBar()->showMessage("PageNumberError", 5000);
        sl_TextLog("Wrong Page Number. Supported PageNumberRange - 0 ... 479.");
        break;
      case state_TransmissionSegmentError:
        statusBar()->showMessage("TransmissionSegmentError", 5000);
        sl_TextLog("Transmission-Segment-Error, splitted Transmission incomplete.");
        break;
      case state_EEPromAddressNumberError:
        statusBar()->showMessage("EERPOM-AddressNumberError", 5000);
        sl_TextLog("The address of the requested EEPROM-Cell is not avaliable.");
        break;
      case state_EEPromBlockWriteError:
        statusBar()->showMessage("EEPROM-BlockWriteError", 5000);
        sl_TextLog("EEPROM: Physical Write Error.");
        break;
      default:
        statusBar()->showMessage("unknown error-code", 5000);
        sl_TextLog("unknown error-code received");
        break;
    }
  }

  int MainWindow::getDefaultBaudrateByName(QString actDeviceName) {
    if (actDeviceName.startsWith("USB-XBEE-Adapter", Qt::CaseInsensitive))
      return 57600;
    if (actDeviceName.startsWith("USB-USART-Adapter", Qt::CaseInsensitive))
      return 460800;
    if (actDeviceName.startsWith("USB-ISP-Adapter", Qt::CaseInsensitive))
      return 921600;
    return 0;
  }

  void MainWindow::sl_eventHandler_ispProgrammer(int eventCode) {

    switch (eventCode) {
      case EVENT_ISP_AVRDEVICE_SIGNATURE_READ: {
        QByteArray msg;
        msg.append((char) Api_ISP_TransmitFirmware);
        msg.append((char) MsgGroup_IspFirmware);
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_BeginTransaction);
        msg.append((char) (programmerWaitCycles >> 8));
        msg.append((char) (programmerWaitCycles >> 0));
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead);
        send_Message(msg);
        timerParams = transmitTimerLastAction_SendMessageISP;
        break;
      }
      case EVENT_ISP_AVRDEVICE_CALIBRATION_READ: {
        QByteArray msg;
        msg.append((char) Api_ISP_TransmitFirmware);
        msg.append((char) MsgGroup_IspFirmware);
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_BeginTransaction);
        msg.append((char) (programmerWaitCycles >> 8));
        msg.append((char) (programmerWaitCycles >> 0));
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_CalibrationBytesRead);
        send_Message(msg);
        timerParams = transmitTimerLastAction_SendMessageISP;
        break;
      }
      case EVENT_ISP_AVRDEVICE_FUSES_EDITOR_SHOW: {
        if (avrDevice != NULL) {
          avrFuseDialog->setAvrDevice(avrDevice);
          avrFuseDialog->exec();
        }
        break;
      }
      case EVENT_ISP_AVRDEVICE_FUSES_READ: {
        QByteArray msg;
        msg.append((char) Api_ISP_TransmitFirmware);
        msg.append((char) MsgGroup_IspFirmware);
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_BeginTransaction);
        msg.append((char) (programmerWaitCycles >> 8));
        msg.append((char) (programmerWaitCycles >> 0));
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_FuseBitsRead);
        send_Message(msg);
        timerParams = transmitTimerLastAction_SendMessageISP;
        break;
      }
      case EVENT_ISP_AVRDEVICE_FUSES_WRITE: {
        QByteArray msg;
        msg.append((char) Api_ISP_TransmitFirmware);
        msg.append((char) MsgGroup_IspFirmware);
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_BeginTransaction);
        msg.append((char) (programmerWaitCycles >> 8));
        msg.append((char) (programmerWaitCycles >> 0));
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_FuseBitsWrite);
        send_Message(msg);
        timerParams = transmitTimerLastAction_SendMessageISP;
        break;
      }
      case EVENT_ISP_AVRDEVICE_FLASH_READ: {
        QByteArray msg;
        msg.append((char) Api_ISP_TransmitFirmware);
        msg.append((char) MsgGroup_IspFirmware);
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_BeginTransaction);
        msg.append((char) (programmerWaitCycles >> 8));
        msg.append((char) (programmerWaitCycles >> 0));
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_FlashPageRead);
        send_Message(msg);
        timerParams = transmitTimerLastAction_SendMessageISP;
        break;
      }
      case EVENT_ISP_AVRDEVICE_FLASH_WRITE: {
        int targetFlashSize = avrDevice->NumberPages * avrDevice->PageSizeBytes;
        int programSize = panelHexViewer->getBinary().length();
        if (targetFlashSize < programSize) {
          // Das zu Schreibende Programm pass nicht komplett in das Zeilsystem hinein!!
          QMessageBox::warning(this, tr("ProgrammSizeError"), tr(
              "The loaded programm-size is larger than the available programm-space in targetsystem. Action abborted."), QMessageBox::Ok, QMessageBox::Ok);
          return;
        }
        QByteArray msg;
        msg.append((char) Api_ISP_TransmitFirmware);
        msg.append((char) MsgGroup_IspFirmware);
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_BeginTransaction);
        msg.append((char) (programmerWaitCycles >> 8));
        msg.append((char) (programmerWaitCycles >> 0));
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_FlashPageWrite);
        send_Message(msg);
        timerParams = transmitTimerLastAction_SendMessageISP;
        break;
      }
      case EVENT_ISP_AVRDEVICE_FLASH_UPDATEWRITE: {
        if (curFileName != NULL)
          loadFile(curFileName);
        int targetFlashSize = avrDevice->NumberPages * avrDevice->PageSizeBytes;
        int programSize = panelHexViewer->getBinary().length();
        if (targetFlashSize < programSize) {
          // Das zu Schreibende Programm pass nicht komplett in das Zeilsystem hinein!!
          QMessageBox::warning(this, tr("ProgrammSizeError"), tr(
              "The loaded programm-size is larger than the available programm-space in targetsystem. Action abborted."), QMessageBox::Ok, QMessageBox::Ok);
          return;
        }
        QByteArray msg;
        msg.append((char) Api_ISP_TransmitFirmware);
        msg.append((char) MsgGroup_IspFirmware);
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_BeginTransaction);
        msg.append((char) (programmerWaitCycles >> 8));
        msg.append((char) (programmerWaitCycles >> 0));
        msg.append((char) MsgCode_IspProgrammer_TargetDevice_FlashPageWrite);
        send_Message(msg);
        timerParams = transmitTimerLastAction_SendMessageISP;
        break;
      }
        //---------------------------------------
      case EVENT_ISP_PROGRAMMER_SOFTWARE_VERSION: {
        QByteArray msg;
        msg.append((char) Api_ISP_TransmitFirmware);
        msg.append((char) MsgGroup_IspFirmware);
        msg.append((char) MsgCode_IspProgrammer_Firmware_SoftwareVersionRead);
        send_Message(msg);
        timerParams = transmitTimerLastAction_SendMessageISP;
        break;
      }
      case EVENT_ISP_PROGRAMMER_RESET: {
        ft232manager->setDTR(1);
        sleep(1);
        ft232manager->setDTR(0);
        break;
      }
      case EVENT_ISP_PROGRAMMER_FLASH_READ: {
        nextOperationState = NEXT_OP_ISP_PROGRAMMER_FLASH_READ;
        ft232manager->setDTR(1);
        sleep(1);
        ft232manager->setDTR(0);
        break;
      }
      case EVENT_ISP_PROGRAMMER_FLASH_WRITE: {
        int targetFlashSize = ProgrammerNumberFlashPages * ProgrammerFlashPageSize;
        int programSize = panelHexViewer->getBinary().length();
        if (targetFlashSize < programSize) {
          // Das zu Schreibende Programm pass nicht komplett in das Zeilsystem hinein!!
          QMessageBox::warning(this, tr("ProgrammSizeError"), tr(
              "The loaded programm-size is larger than the available programm-space in programmer. Action abborted."), QMessageBox::Ok, QMessageBox::Ok);
          return;
        }
        nextOperationState = NEXT_OP_ISP_PROGRAMMER_FLASH_WRITE;
        ft232manager->setDTR(1);
        sleep(1);
        ft232manager->setDTR(0);
        break;
      }
    }
  }
  void MainWindow::sl_eventHandler_ecbBootloader(int eventCode) {

    switch (eventCode) {
      case EVENT_ECB_RESET: {
        send_ECB_Reset();
        break;
      }
      case EVENT_ECB_BOOTLOADER_FLASH_READ: {
        nextOperationState = NEXT_OP_ECB_BOOTLOADER_FLASH_READ;
        send_ECB_Reset();
        break;
      }
      case EVENT_ECB_BOOTLOADER_FLASH_WRITE: {
        nextOperationState = NEXT_OP_ECB_BOOTLOADER_FLASH_WRITE;
        send_ECB_Reset();
        break;
      }
      case EVENT_ECB_BOOTLOADER_FLASH_UPDATEWRITE: {
        if (curFileName != NULL) {
          loadFile(curFileName);
          nextOperationState = NEXT_OP_ECB_BOOTLOADER_FLASH_WRITE;
          send_ECB_Reset();
        }
        break;
      }
      case EVENT_ECB_BOOTLOADER_EEPROM_READ: {
        nextOperationState = NEXT_OP_ECB_BOOTLOADER_EEPROM_READ;
        send_ECB_Reset();
        break;
      }
      case EVENT_ECB_BOOTLOADER_EEPROM_WRITE: {
        nextOperationState = NEXT_OP_ECB_BOOTLOADER_EEPROM_WRITE;
        send_ECB_Reset();
        break;
      }
    }
  }
  void MainWindow::sl_eventHandler_application(int eventCode) {

    switch (eventCode) {
      case EVENT_APPLICATION_BINARY_OPEN: {
        fileDialog->setAcceptMode(QFileDialog::AcceptOpen);
        fileDialog->setFileMode(QFileDialog::ExistingFile);
        if (fileDialog->exec()) {
          QString fileName = fileDialog->selectedFiles().at(0);
          if (loadFile(fileName)) {
            hasBinary = true;
            action_Binary_save->setEnabled(true);
            action_Programmer_Flash_write->setEnabled(true);
            action_ECB_Bootloader_Flash_write->setEnabled(true);
            action_ECB_Bootloader_Flash_update_write->setEnabled(true);
            action_ECB_Bootloader_EEPROM_write->setEnabled(true);
            if (avrDevice != NULL && hasAVRDeviceIdedentified) {
              action_Target_Flash_write->setEnabled(true);
              action_Target_Flash_update_write->setEnabled(true);
            }
          }
        }
        break;
      }
      case EVENT_APPLICATION_BINARY_SAVE: {
        fileDialog->setAcceptMode(QFileDialog::AcceptSave);
        fileDialog->setFileMode(QFileDialog::AnyFile);
        if (fileDialog->exec()) {
          QString fileName = fileDialog->selectedFiles().at(0);
          if (!saveFile(fileName)) {
            QMessageBox::warning(this, tr("LUPAE_V2"), tr("The document could not been saved."), QMessageBox::Close);
          }
        }
        break;
      }
      case EVENT_APPLICATION_LOGVIEW_CLEAR: {
        panelLogView->clearLogViewText();
        break;
      }
      case EVENT_APPLICATION_CLOSE: {
        close();
        break;
      }
      case EVENT_APPLICATION_ABOUT: {
        QMessageBox::about(
            this,
            tr("About the Application"),
            tr(
                "LUPAE V2.0 (<b>L</b>pzRobots <b>u</b>niversal <b>p</b>rogramm <b>a</b>pplication for <b>E</b>CB) \n is an <b>Application</b> to (re)programm AVR-Microcontrollers."));
        break;
      }
      case EVENT_APPLICATION_SCAN_USBDEVICE: {
        // Sichere den Namen des aktuellen USBDevices
        QString actDeviceName = panelSetting->getUSBDeviceName();
        QString actBaudrate = panelSetting->getUSBDeviceBaudrate();

        ft232manager->closeDevice(); // Serial Port schließen
        panelSetting->stopSignaling(); // Keine Nachrichten über die Veränderungen der Auswahl der Combo-Boxen
        // Die Liste aller verfügbaren Seriellen Ports holen
        QStringList deviceNames = ft232manager->getDeviceList();
        panelSetting->setUSBDeviceNames(deviceNames);

        if (deviceNames.contains(actDeviceName, Qt::CaseInsensitive)) {
          // Öffne wieder das gleiche Device
          panelSetting->setUSBDeviceName(actDeviceName);
          panelSetting->setUSBDeviceBaudrate(actBaudrate);
          ft232manager->openDeviceByName(actDeviceName, actBaudrate.toInt());
        } else {
          // das bisherige USBDevice ist nicht weiter verfügbar,
          // öffne das erste USBDevice, welches das panelSettings anzeigt mit der default-baudrate.
          // Hole den Namen des angezeigten USB-Devices aus dem PANEL-SETTING und öffne dieses Device!
          actDeviceName = panelSetting->getUSBDeviceName();
          int defBaudRate = panelSetting->getUSBDeviceBaudrate().toInt();
          ft232manager->openDeviceByName(actDeviceName, defBaudRate);
        }
        panelSetting->startSignaling(); // Aktiviere wieder alle Nachrichten über die Veränderungen der Auswahl der Combo-Boxen
        break;
      }
    }
  }

  void MainWindow::sl_TextLog(QString sText) {
    statusBar()->showMessage(sText, 5000);
    panelLogView->appendLogViewText(sText);
  }
  void MainWindow::sl_DispatchMessage(QByteArray receiveBuffer) {

    if (debug) {
      printBuffer(receiveBuffer);
    }

    switch (applicationMode) {
      case APPLICATION_MODE_ISP_Adapter: {
        // Stoppe TransmitTimer
        timer->stop();

        // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
        // ----------------------
        // 0x00 - StartDelemiter
        // 0x01 - Length_HighByte
        // 0x02 - Length_LowByte
        // 0x03 - API-Identifier
        // ----------------------
        // 0x04 - MsgGroup
        // 0x05 - MsgCode
        // 0x06 - data/params...

        QByte msgApiIdentifier = receiveBuffer[3];
        if (msgApiIdentifier == Api_ISP_TransmitBootloader || msgApiIdentifier == Api_ISP_TransmitFirmware) {
          QByte msgGroup = receiveBuffer[4];
          switch (msgGroup) {
            case MsgGroup_IspBootloader: // Bootloader
            {
              isp_MessageHandler_Bootloader(receiveBuffer);
              break;
            }
            case MsgGroup_IspFirmware: {
              isp_MessageHandler_Firmware(receiveBuffer);
              break;
            }
            default:
              sl_TextLog("<DispatchMessage_ISP> unknown message received");
              break;
          }
        }// if(ApiIdentifier)
        break;
      }
      case APPLICATION_MODE_USART_Adapter:
      case APPLICATION_MODE_XBEE_Adapter: {
        // Stoppe TransmitTimer
        timer->stop();

        // Eine Nachricht vom Microcontroller wurde empfangen
        // --------------------------------------------------
        //  0 QByte StartDelimiter;
        //  1 QByte Length_MSB;
        //  2 QByte Length_LSB;
        //  3 QByte API_ID;
        //  4 ...

        uint msgApi_Id = receiveBuffer[3] & 0xFF;
        QByteArray received_msg;
        switch (msgApi_Id) {
          case API_XBee_AT_Command_Response:
            bl_MessageHandler_XBeeCommandResponse(receiveBuffer);
            break;
          case API_Cable_TransmitReceive:
            // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
            // ----------------------
            // 0x00 - StartDelemiter
            // 0x01 - Length_HighByte
            // 0x02 - Length_LowByte
            // ----------------------
            // 0x03 - API-Identifier
            // ----------------------
            // 0x04 - MsgGroup
            // 0x05 - MsgCode
            // 0x06 - data/params...
            // ----------------------
            bl_MessageHandler_Bootloader(receiveBuffer.mid(0x04));
            break;
          case API_XBee_Receive_Packet_16Bit:
            // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
            // ----------------------
            // 0x00 - StartDelemiter
            // 0x01 - Length_HighByte
            // 0x02 - Length_LowByte
            // ----------------------
            // 0x03 - API-Identifier
            // 0x04 - sourceAddress16_1
            // 0x05 - sourceAddress16_0
            // 0x06 - rssi
            // 0x07 - options
            // ----------------------
            // 0x08 - MsgGroup
            // 0x09 - MsgCode
            // 0x0A - data/params...
            // ----------------------
            bl_MessageHandler_Bootloader(receiveBuffer.mid(0x08));
            break;
          case API_XBeeS2_ZigBee_Receive_Packet:
            // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
            // ----------------------
            // 0x00 - StartDelemiter
            // 0x01 - Length_HighByte
            // 0x02 - Length_LowByte
            // ----------------------
            // 0x03 - API-Identifier
            // 0x04 - sourceAddress64_7
            // 0x05 - sourceAddress64_6
            // 0x06 - sourceAddress64_5
            // 0x07 - sourceAddress64_4
            // 0x08 - sourceAddress64_3
            // 0x09 - sourceAddress64_2
            // 0x0A - sourceAddress64_1
            // 0x0B - sourceAddress64_0
            // 0x0C - sourceAddress16_1
            // 0x0D - sourceAddress16_0
            // 0x0E - options
            // ----------------------
            // 0x0F - MsgGroup
            // 0x10 - MsgCode
            // 0x11 - data/params...
            // ----------------------
            bl_MessageHandler_Bootloader(receiveBuffer.mid(0x0F));
            break;
          default: {
            QString s;
            s.append(QString::number((QByte) (msgApi_Id >> 4) & 0x0F, 16).toUpper());
            s.append(QString::number((QByte) (msgApi_Id >> 0) & 0x0F, 16).toUpper());
            sl_TextLog("Unknown Api-Code = 0x" + s + " received.");
            printBuffer(receiveBuffer);
            return;
          }
        } //end switch api
        break;
      }
    }
  }

  void MainWindow::sl_USBDevice_Name_changed(QString usbDeviceName) {
    QString actDeviceName = ft232manager->getDeviceName();

    if (actDeviceName.compare(usbDeviceName) != 0) {
      ft232manager->closeDevice();

      int baudrate = 57600;
      baudrate = panelSetting->getUSBDeviceBaudrate().toInt();
      if (debug)
        sl_TextLog("slot_OnSignal_USBDeviceNameChanged: baudrate=" + panelSetting->getUSBDeviceBaudrate());
      ft232manager ->openDeviceByName(usbDeviceName, baudrate);
    }
  }
  void MainWindow::sl_USBDevice_Baudrate_changed(QString sBaudrate) {
    int actBaudrate = ft232manager->getBaudrate();
    int baudrate = 57600;
    baudrate = sBaudrate.toInt();

    if (actBaudrate != baudrate) {
      ft232manager->setBaudrate(baudrate);
    }
  }
  void MainWindow::sl_AVRDevice_AccessSpeed_changed(QString sTargetSpeed) {
    setProgrammerWaitCycles(sTargetSpeed);
  }
  void MainWindow::sl_USBDeviceXBeeType_Changed(QString name) {
    QStringList emptyList;

    if (name.compare("unknown") == 0) {
      panelSetting->setNodeIdentifierSettingEnabled(false);
      panelSetting->setXBeeRemoteNodeIdentifiers(emptyList);
      USBDeviceXBeeType = XBeeType_unknown;
      return;
    }
    if (name.compare("XBeeSerie1") == 0) {
      panelSetting->setNodeIdentifierSettingEnabled(true);
      panelSetting->setXBeeRemoteNodeIdentifiers(emptyList);
      USBDeviceXBeeType = XBeeType_Serie1;
      send_XBeeATND();
      return;
    }
    if (name.compare("XBeeSerie2") == 0) {
      panelSetting->setNodeIdentifierSettingEnabled(true);
      panelSetting->setXBeeRemoteNodeIdentifiers(emptyList);
      USBDeviceXBeeType = XBeeType_Serie2;
      send_XBeeATND();
      return;
    }

  }
  void MainWindow::sl_XBeeRemoteNodeIdentifier_changed(QString name) {
    bool parse_ok;
    quint16 ret16 = name.mid(1, 4).toUShort(&parse_ok, 16);
    if (parse_ok)
      ECB_XBeeAddress16 = ret16;
    else
      ECB_XBeeAddress16 = 0x00FE;
    if (debug)
      sl_TextLog("ECB_XBeeAddress16=" + QString::number(ECB_XBeeAddress16, 16).toUpper());

    quint64 ret64 = name.mid(6, 16).toULongLong(&parse_ok, 16);
    if (parse_ok)
      ECB_XBeeAddress64 = ret64;
    else
      ECB_XBeeAddress64 = 0x0000000000FFFF;
    if (debug)
      sl_TextLog("ECB_XBeeAddress64=" + QString::number(ECB_XBeeAddress64, 16).toUpper());
  }
  void MainWindow::sl_TimerExpired() {
    timer->stop();

    // Was war die letzte Aktion, welche
    // den Transmit Timer aktiviert hatte?
    switch (timerParams) {
      case transmitTimerLastAction_none:
        break;
      case transmitTimerLastAction_initialize: {

        readSettings();
        loadTargetDeviceParameter();

        QTimer::singleShot(1, this, SLOT(sl_ScanUSBDevices()));
        break;
      }
      case transmitTimerLastAction_XBeeCommand:
        sl_TextLog("Could not address to XBee-RF-module.");
        sl_TextLog("Please check USB-XBee-Adapter and XBee-RF-modul.");
        break;
      case transmitTimerLastAction_XBeeRemoteCommand:
        sl_TextLog("Could not address to XBee-RF-module.");
        sl_TextLog("Please check USB-XBee-Adapter and XBee-RF-modul");
        sl_TextLog("as well remote XBee-Node.");
        break;
      case transmitTimerLastAction_SendMessageBL:
        sl_TextLog("No response from bootloader!");
        sl_TextLog("Please check connection to ECB,");
        sl_TextLog("as well as supply-voltage.");
        break;
      case transmitTimerLastAction_SendMessageISP:
        sl_TextLog("No response from programmer!");
        sl_TextLog("Please check connection to programmer.");
        break;
      default:
        sl_TextLog("unknown error!");
        break;
    }// switch(lastAction)

    nextOperationState = NEXT_OP_NONE;

    if (progress != 0) {
      progress->hide();
      progress->setValue(progress->maximum());
    }

  }
  void MainWindow::sl_USBDevice_opened() {
    // Sende Commando zum Abfragen der Hardware-Version eines XBee-Modules
    panelSetting->stopSignaling();
    panelSetting->setUSBDeviceName(ft232manager->getDeviceName());
    panelSetting->setUSBDeviceBaudrate(QString::number(ft232manager->getBaudrate()));
    panelSetting->startSignaling();

    if (ft232manager->getDeviceName().startsWith("USB-ISP-Adapter")) {
      panelSetting->setAVRDeviceName("");

      // ISP-Adapter: (ISP-Programmer)
      applicationMode = APPLICATION_MODE_ISP_Adapter;
      createMenus(applicationMode);
      action_Target_SignatureBytes_read->setEnabled(true);
      action_Target_CalibrationBytes_read->setEnabled(false);
      action_Target_ShowFuseDialog->setEnabled(false);
      action_Target_Flash_read->setEnabled(false);
      action_Target_Flash_write->setEnabled(false);
      action_Target_Flash_update_write->setEnabled(false);
      action_Programmer_SoftwareVersion->setEnabled(true);
      action_Programmer_reset->setEnabled(true);
      action_Programmer_Flash_read->setEnabled(true);
      action_Programmer_Flash_write->setEnabled(false);
    } else if (ft232manager->getDeviceName().startsWith("USB-USART-Adapter")) {
      // USART-Adapter:
      applicationMode = APPLICATION_MODE_USART_Adapter;
      createMenus(applicationMode);
      action_ECB_reset->setEnabled(true);
      action_ECB_Bootloader_Flash_read->setEnabled(true);
      action_ECB_Bootloader_Flash_write->setEnabled(false);
    } else if (ft232manager->getDeviceName().startsWith("USB-XBEE-Adapter")) {
      QStringList emptyList;
      panelSetting->setXBeeRemoteNodeIdentifiers(emptyList);
      panelSetting->setUSBDeviceXBeeType("unknown");

      // Dies ist ein Funkmodul, sperre die Menu-Einträge
      // Die Freischaltung der Menu-Einträge erfolgt wenn ein Remote-Knoten gefunden wurde!
      applicationMode = APPLICATION_MODE_XBEE_Adapter;
      createMenus(applicationMode);
      action_ECB_reset->setEnabled(false);
      action_ECB_Bootloader_Flash_read->setEnabled(false);
      action_ECB_Bootloader_Flash_write->setEnabled(false);

      // Frage nun die Hardware-Version des auf dem USB-XBee-Adapter aufgesteckten
      // XBee-Funkmodules ab. (Im Anschluss kann dann geziehlt nach Remote-Knoten
      // gesucht werden.)
      push_Frame(0x7E); // Startsymbol
      push_FrameEscaped(0x00); // Length MSB
      push_FrameEscaped(0x04); // Length LSB
      push_FrameEscaped(0x08); // API_ID - AT_Command
      push_FrameEscaped((QByte) 'R'); // Frame_ID - 'R' -> erwarte Antwort
      push_FrameEscaped((QByte) 'H'); // AT-Command QByte 1
      push_FrameEscaped((QByte) 'V'); // AT-Command QByte 2
      transmit(transmitTimerLastAction_XBeeCommand);
    }
  }

  void MainWindow::setProgrammerWaitCycles(const QString targetSpeed) {
    if (targetSpeed.compare("slow") == 0) {
      programmerWaitCycles = 100;
      return;
    }
    if (targetSpeed.compare("medium") == 0) {
      programmerWaitCycles = 10;
      return;
    }
    if (targetSpeed.compare("fast") == 0) {
      programmerWaitCycles = 0;
      return;
    }

    // Sollte keines der obigen Fälle zutreffen, dann setze Defaultwert
    programmerWaitCycles = 100;
  }

  bool MainWindow::loadFile(const QString &fileName) {
    QFile file(fileName);
    if (!file.open(QFile::ReadOnly)) {
      QMessageBox::warning(this, tr("Application"), tr("Cannot read file %1:\n%2.") .arg(fileName) .arg(file.errorString()));
      return false;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    QByteArray binary = file.readAll();
    file.close();
    panelHexViewer->setBinary(binary);
    QApplication::restoreOverrideCursor();

    setCurrentFile(fileName);
    statusBar()->showMessage("Binary opended", 2000);
    sl_TextLog("Binary '" + fileName + "' opended.");

    return true;
  }
  bool MainWindow::saveFile(const QString &fileName) {
    QFile file(fileName);
    if (!file.open(QFile::WriteOnly)) {
      QMessageBox::warning(this, tr("Application"), tr("Cannot write file %1:\n%2.") .arg(fileName) .arg(file.errorString()));
      return false;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    file.write(panelHexViewer->getBinary());
    file.close();
    QApplication::restoreOverrideCursor();

    setCurrentFile(fileName);
    statusBar()->showMessage(tr("File saved"), 2000);
    return true;
  }
  void MainWindow::setCurrentFile(const QString &fileName) {
    curFileName = fileName;
    setWindowModified(false);

    QString shownName;
    if (curFileName.isEmpty())
      shownName = "untitled.txt";
    else
      shownName = QFileInfo(curFileName).fileName();
    setWindowTitle(tr("%1[*] - %2").arg(shownName).arg(tr("LUPAE V2.0c")));
  }

} // namespace lpzrobots

