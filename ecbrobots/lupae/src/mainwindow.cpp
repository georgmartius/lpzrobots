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
 *   Revision 1.1  2010-11-03 13:05:27  wrabe
 *   -new version 2.0 uses ftdi driver library (libftdi and libusb)
 *   -special string identifiers (device descriptor) of usb devices (FT232RL), hard coded!
 *                                              *
 *                                                                         *
 ***************************************************************************/

#include "mainwindow.h"

MainWindow::MainWindow() {
  QTimer::singleShot(1, this, SLOT(sl_Initialize()));
}
void MainWindow::sl_Initialize() {
  panelHexViewer = new QPanelHexViewer();
  avrFuseDialog = new QAvrFuseDialog();
  ft232manager = new QFT232DeviceManager();
  panelSetting = new QPanelSetting();
  panelLogView = new QPanelLogView();
  timer = new QTimer();
  progress = 0;
  automaticMode = false;
  hasBinary = false;

  // Der Transmit-Puffer
  timerParams = transmitTimerLastAction_none;
  transmitBuffer.clear();
  transmitBufferCheckSum = 0;
  ECB_Bootloader_FlashPage_Read = false;
  ECB_Bootloader_FlashPage_Write = false;
  ECB_OperationRetries = 0;
  ECB_OperationRetriesMax = 3;
  applicationMode = APPLICATION_MODE_None;

  programerFlashRead = false;
  programerFlashWrite = false;
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
  grid->addWidget(panelSetting, 0, 0, Qt::AlignLeft);

  tabWidget = new QTabWidget;
  tabWidget->addTab(panelLogView, tr("Report"));
  tabWidget->addTab(panelHexViewer, tr("Binary"));
  grid->addWidget(tabWidget, 1, 0, Qt::AlignLeft);

  setMinimumWidth(600);
  setMaximumWidth(600);

  initPanelSetting();
  createActions();
  createMenus();
  createToolBars();
  createStatusBar();
  setCurrentFile("");

  connect(avrFuseDialog, SIGNAL(readFuseBits()), this, SLOT(sl_ISP_AVRDevice_FuseBytes_read()));
  connect(avrFuseDialog, SIGNAL(writeFuseBits()), this, SLOT(sl_ISP_AVRDevice_FuseBytes_write()));
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

  sl_ScanUSBDevices();

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
  panelSetting->setUSBDeviceXBeeType(connections);
  panelSetting->setUSBDeviceXBeeType(connections.at(0));

  USBDeviceXBeeType = XBeeType_unknown;

}
void MainWindow::createActions() {
  action_Binary_open = new QAction(QIcon(":/images/open.png"), tr("&Open..."), this);
  //action_Binary_open = new QAction(tr("&Open..."), this);
  action_Binary_open->setShortcut(tr("Ctrl+O"));
  action_Binary_open->setStatusTip(tr("Open binary"));
  connect(action_Binary_open, SIGNAL(triggered()), this, SLOT(sl_Binary_open()));

  action_Binary_save = new QAction(QIcon(":/images/save.png"), tr("&Save"), this);
  //action_Binary_save = new QAction(tr("&Save"), this);
  action_Binary_save->setShortcut(tr("Ctrl+S"));
  action_Binary_save->setStatusTip(tr("Save the binary"));
  action_Binary_save->setEnabled(false);
  connect(action_Binary_save, SIGNAL(triggered()), this, SLOT(sl_Binary_save()));

  action_Exit = new QAction(tr("&Quit"), this);
  action_Exit->setShortcut(tr("Ctrl+Q"));
  action_Exit->setStatusTip(tr("Exit the application"));
  connect(action_Exit, SIGNAL(triggered()), this, SLOT(close()));

  // Actions for Target Device
  action_Target_SignatureBytes_read = new QAction(tr("&Identify Target Device"), this);
  action_Target_SignatureBytes_read->setShortcut(tr("Ctrl+I"));
  action_Target_SignatureBytes_read->setStatusTip(tr("Read Target Device Signature Bytes"));
  action_Target_SignatureBytes_read->setEnabled(false);
  connect(action_Target_SignatureBytes_read, SIGNAL(triggered()), this, SLOT(sl_ISP_AVRDevice_SignatureBytes_read()));

  action_Target_CalibrationBytes_read = new QAction(tr("Read Calibration-Bytes"), this);
  action_Target_CalibrationBytes_read->setStatusTip(tr("Read Target Device Calibration Bytes"));
  action_Target_CalibrationBytes_read->setEnabled(false);
  connect(action_Target_CalibrationBytes_read, SIGNAL(triggered()), this, SLOT(sl_ISP_AVRDevice_CalibrationBytes_read()));

  action_Target_ShowFuseDialog = new QAction(tr("Show FuseBit-Editor"), this);
  action_Target_ShowFuseDialog->setStatusTip(tr("Open the Fuse-Bit-Editor."));
  action_Target_ShowFuseDialog->setEnabled(false);
  connect(action_Target_ShowFuseDialog, SIGNAL(triggered()), this, SLOT(sl_ISP_ShowFuseDialog()));

  action_Target_Flash_read = new QAction(tr("&Read Flash"), this);
  action_Target_Flash_read->setStatusTip(tr("Read Target Device Programm-Space"));
  action_Target_Flash_read->setShortcut(tr("Ctrl+R"));
  action_Target_Flash_read->setEnabled(false);
  connect(action_Target_Flash_read, SIGNAL(triggered()), this, SLOT(sl_ISP_AVRDevice_Flash_read()));

  action_Target_Flash_write = new QAction(tr("&WriteFlash"), this);
  action_Target_Flash_write->setShortcut(tr("Ctrl+W"));
  action_Target_Flash_write->setStatusTip(tr("Write Target Device Programm-Space"));
  action_Target_Flash_write->setEnabled(false);
  connect(action_Target_Flash_write, SIGNAL(triggered()), this, SLOT(sl_ISP_AVRDevice_Flash_write()));

  action_Target_Flash_update_write = new QAction(tr("&Update/Flash"), this);
  action_Target_Flash_update_write->setShortcut(tr("Ctrl+U"));
  action_Target_Flash_update_write->setStatusTip(tr("Reload the binary and write Target Device Programm-Space"));
  action_Target_Flash_update_write->setEnabled(false);
  connect(action_Target_Flash_update_write, SIGNAL(triggered()), this, SLOT(sl_ISP_AVRDevice_Flash_update_write()));

  // Actions for Programmer
  action_Programmer_ping = new QAction(tr("Ping"), this);
  action_Programmer_ping->setStatusTip(tr("Pings the USB-Programmer"));
  connect(action_Programmer_ping, SIGNAL(triggered()), this, SLOT(sl_ISP_Programmer_ping()));

  action_Programmer_reset = new QAction(tr("Reset"), this);
  action_Programmer_reset->setStatusTip(tr("Resets the USB-Programmer"));
  connect(action_Programmer_reset, SIGNAL(triggered()), this, SLOT(sl_ISP_Programmer_reset()));

  action_Programmer_Flash_read = new QAction(tr("ReadFlash"), this);
  action_Programmer_Flash_read->setStatusTip(tr("Read Programm from Programmer"));
  connect(action_Programmer_Flash_read, SIGNAL(triggered()), this, SLOT(sl_ISP_Programmer_Flash_read()));

  action_Programmer_Flash_write = new QAction(tr("WriteFlash"), this);
  action_Programmer_Flash_write->setStatusTip(tr("Write new Programm to Programmer"));
  action_Programmer_Flash_write->setEnabled(false);
  connect(action_Programmer_Flash_write, SIGNAL(triggered()), this, SLOT(sl_ISP_Programmer_Flash_write()));

  // Actions for the Bootloader
  action_ECB_reset = new QAction(tr("Reset ECB"), this);
  action_ECB_reset->setShortcut(tr("Ctrl+Shift+R"));
  action_ECB_reset->setStatusTip(tr("Send Command to RESET the Controller"));
  action_ECB_reset->setEnabled(false);
  connect(action_ECB_reset, SIGNAL(triggered()), this, SLOT(sl_ECB_reset()));

  action_ECB_Bootloader_Flash_read = new QAction(tr("&Read ProgramSpace"), this);
  action_ECB_Bootloader_Flash_read->setShortcut(tr("Ctrl+R"));
  action_ECB_Bootloader_Flash_read->setStatusTip(tr("Read the Programm-Space of the specified Controller"));
  action_ECB_Bootloader_Flash_read->setEnabled(false);
  connect(action_ECB_Bootloader_Flash_read, SIGNAL(triggered()), this, SLOT(sl_ECB_Bootloader_Flash_read()));

  action_ECB_Bootloader_Flash_write = new QAction(tr("&Write ProgramSpace"), this);
  action_ECB_Bootloader_Flash_write->setShortcut(tr("Ctrl+W"));
  action_ECB_Bootloader_Flash_write->setStatusTip(tr("Write the loaded Binary to the Programm-Space of the connected ECB"));
  action_ECB_Bootloader_Flash_write->setEnabled(false);
  connect(action_ECB_Bootloader_Flash_write, SIGNAL(triggered()), this, SLOT(sl_ECB_Bootloader_Flash_write()));

  action_ECB_Bootloader_Flash_update_write = new QAction(tr("&Update/Write ProgramSpace"), this);
  action_ECB_Bootloader_Flash_update_write->setShortcut(tr("Ctrl+U"));
  action_ECB_Bootloader_Flash_update_write->setStatusTip(tr("Reload the binary and write in into the Programm-Space of the connected ECB"));
  action_ECB_Bootloader_Flash_update_write->setEnabled(false);
  connect(action_ECB_Bootloader_Flash_update_write, SIGNAL(triggered()), this, SLOT(sl_ECB_Bootloader_Flash_update_write()));

  // Extras...
  action_ClearLogView = new QAction(tr("&Clear LogView"), this);
  action_ClearLogView->setShortcut(tr("Ctrl+Shift+C"));
  action_ClearLogView->setStatusTip(tr("Clear the LogView"));
  connect(action_ClearLogView, SIGNAL(triggered()), this, SLOT(sl_ClearLogView()));

  action_SerialPorts_refresh = new QAction(tr("ScanUSBDevices"), this);
  action_SerialPorts_refresh->setStatusTip(tr("Scan for connected USB-Tools."));
  action_SerialPorts_refresh->setShortcut(Qt::Key_F5);
  action_SerialPorts_refresh->setEnabled(true);
  connect(action_SerialPorts_refresh, SIGNAL(triggered()), this, SLOT(sl_ScanUSBDevices()));

  // Actions About
  action_About = new QAction(tr("&About"), this);
  action_About->setStatusTip(tr("Show the application's About box"));
  connect(action_About, SIGNAL(triggered()), this, SLOT(sl_About()));

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
      ispProgrammerMenu->addAction(action_Programmer_ping);
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
  QSettings settings(applicationPath + QString(".ini"), QSettings::IniFormat);
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
  QSettings settings(applicationPath + QString(".ini"), QSettings::IniFormat);
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
bool MainWindow::transmit() {
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
  return ret;
}
void MainWindow::send_Message(QByteArray command) {
  QWord length = command.length();

  push_Frame(0x7E); // Startsymbol
  push_FrameEscaped((QByte) (length >> 8)); // Length MSB
  push_FrameEscaped((QByte) (length >> 0)); // Length LSB
  for (int i = 0; i < length; i++)
    push_FrameEscaped(command[i]);
  transmit();
  timerParams = transmitTimerLastAction_SendMessageRaw;

}
void MainWindow::send_Message(uchar msgCode, uchar msgParam1, uchar msgParam2, uchar msgParam3, uchar msgParam4) {
  switch (applicationMode) {
    case APPLICATION_MODE_ISP_Adapter: {
      push_Frame(0x7E); // Startsymbol
      push_FrameEscaped(0x00); // Length MSB
      push_FrameEscaped(0x05); // Length LSB
      push_FrameEscaped(msgCode); // MSG_Code
      push_FrameEscaped(msgParam1); // PARAMETER1
      push_FrameEscaped(msgParam2); // PARAMETER2
      push_FrameEscaped(msgParam3); // PARAMETER3
      push_FrameEscaped(msgParam4); // PARAMETER4
      transmit();
      timerParams = transmitTimerLastAction_SendMessageISP;
      break;
    }
    case APPLICATION_MODE_USART_Adapter: {
      QWord length = 1 + 6;

      push_Frame(0x7E); //  1: Startsymbol
      push_FrameEscaped((QByte) (length >> 8)); //  2: Length MSB
      push_FrameEscaped((QByte) (length >> 0)); //  3: Length LSB
      push_FrameEscaped(0x20); //  4: API_ID - Cable

      // AnwenderDaten
      push_FrameEscaped(0x00); // MessageGroup_ID - immer 0 -> Bootloader
      push_FrameEscaped(msgCode); // MessageCode
      push_FrameEscaped(msgParam1); // MessageParameter_1
      push_FrameEscaped(msgParam2); // MessageParameter_2
      push_FrameEscaped(msgParam3); // MessageParameter_3
      push_FrameEscaped(msgParam4); // MessageParameter_4
      transmit();
      timerParams = transmitTimerLastAction_SendMessageBL;
      break;
    }
    case APPLICATION_MODE_XBEE_Adapter: {
      switch (USBDeviceXBeeType) {
        case XBeeType_Serie1: {
          QWord length = 5 + 6;
          push_Frame(0x7E); //  1: Startsymbol
          push_FrameEscaped((QByte) (length >> 8)); //  2: Length MSB
          push_FrameEscaped((QByte) (length >> 0)); //  3: Length LSB
          push_FrameEscaped(0x01); //  4: API-ID
          push_FrameEscaped(0x00); //  5: Frame-ID - immer 0 -> kein ResponsePaket vom XBee
          push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 8)); //  6: DestinationAddress MSB
          push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 0)); //  7: DestinationAddress LSB
          push_FrameEscaped(0x01); //  8: Options - immer 1  -> kein ResponsePaket vom XBee
          break;
        }
        case XBeeType_Serie2: {
          QWord length = 14 + 6;
          push_Frame(0x7E); //  1: Startsymbol
          push_FrameEscaped((QByte) (length >> 8)); //  2: Length MSB
          push_FrameEscaped((QByte) (length >> 0)); //  3: Length LSB
          push_FrameEscaped(0x10); //  4: API_ID - TransmitRequest XBeeSerie2
          push_FrameEscaped(0x00); //  5: Frame-ID - immer 0 -> kein ResponsePaket vom XBee
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
          push_FrameEscaped(0x00); // 16: Broadcast-Range
          push_FrameEscaped(0x01); // 17: OptionsByte - immer 1  -> kein ResponsePaket vom XBee
          break;
        }
      }//end switch
      // AnwenderDaten
      push_FrameEscaped(0x00); // MessageGroup_ID - immer 0 -> Bootloader
      push_FrameEscaped(msgCode); // MessageCode
      push_FrameEscaped(msgParam1); // MessageParameter_1
      push_FrameEscaped(msgParam2); // MessageParameter_2
      push_FrameEscaped(msgParam3); // MessageParameter_3
      push_FrameEscaped(msgParam4); // MessageParameter_4
      transmit();
      timerParams = transmitTimerLastAction_SendMessageBL;
      break;
    }
  }
}
void MainWindow::send_Message(uchar msgCode, uchar msgParam1, uchar msgParam2, uchar msgParam3, uchar msgParam4, QByteArray pageBuffer) {
  switch (applicationMode) {
    case APPLICATION_MODE_ISP_Adapter: {
      int length = 5 + pageBuffer.length();
      push_Frame(0x7E); // Startsymbol
      push_FrameEscaped(length >> 8); // Length MSB
      push_FrameEscaped(length >> 0); // Length LSB
      push_FrameEscaped(msgCode); // MSG_Code
      push_FrameEscaped(msgParam1); // PARAMETER1
      push_FrameEscaped(msgParam2); // PARAMETER2
      push_FrameEscaped(msgParam3); // PARAMETER3
      push_FrameEscaped(msgParam4); // PARAMETER4
      // Die Page
      for (int i = 0; i < pageBuffer.length(); i++)
        push_FrameEscaped((QByte) pageBuffer[i]);
      transmit();
      timerParams = transmitTimerLastAction_SendMessageISP;
      break;
    }
    case APPLICATION_MODE_USART_Adapter: {
      int length = 1 + 6 + pageBuffer.length();
      push_Frame(0x7E); //  1: Startsymbol
      push_FrameEscaped((QByte) (length >> 8)); //  2: Length MSB
      push_FrameEscaped((QByte) (length >> 0)); //  3: Length LSB
      push_FrameEscaped(0x20); //  4: API_ID - Cable
      // AnwenderDaten
      push_FrameEscaped(0x00); // MessageGroup_ID - immer 0 -> Bootloader
      push_FrameEscaped(msgCode); // MessageCode
      push_FrameEscaped(msgParam1); // MessageParameter_1
      push_FrameEscaped(msgParam2); // MessageParameter_2
      push_FrameEscaped(msgParam3); // MessageParameter_3
      push_FrameEscaped(msgParam4); // MessageParameter_4

      // Die Page
      for (int i = 0; i < pageBuffer.length(); i++)
        push_FrameEscaped((QByte) pageBuffer[i]);
      transmit();
      timerParams = transmitTimerLastAction_SendMessageBL;
      break;
    }
    case APPLICATION_MODE_XBEE_Adapter: {
      switch (USBDeviceXBeeType) {
        case XBeeType_Serie1: {
          QWord length = 5 + 6 + pageBuffer.length();
          push_Frame(0x7E); //  1: Startsymbol
          push_FrameEscaped((QByte) (length >> 8)); //  2: Length MSB
          push_FrameEscaped((QByte) (length >> 0)); //  3: Length LSB
          push_FrameEscaped(0x01); //  4: API-ID
          push_FrameEscaped(0x00); //  5: Frame-ID - immer 0 -> kein ResponsePaket vom XBee
          push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 8)); //  6: DestinationAddress MSB
          push_FrameEscaped((QByte) (ECB_XBeeAddress16 >> 0)); //  7: DestinationAddress LSB
          push_FrameEscaped(0x01); //  8: Options - immer 1  -> kein ResponsePaket vom XBee
          break;
        }
        case XBeeType_Serie2: {
          QWord length = 14 + 6 + pageBuffer.length();
          push_Frame(0x7E); //  1: Startsymbol
          push_FrameEscaped((QByte) (length >> 8)); //  2: Length MSB
          push_FrameEscaped((QByte) (length >> 0)); //  3: Length LSB
          push_FrameEscaped(0x10); //  4: API_ID - TransmitRequest XBeeSerie2
          push_FrameEscaped(0x00); //  5: Frame-ID - immer 0 -> kein ResponsePaket vom XBee
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
          push_FrameEscaped(0x00); // 16: Broadcast-Range
          push_FrameEscaped(0x01); // 17: OptionsByte - immer 1  -> kein ResponsePaket vom XBee
          break;
        }
      }//end switch
      // AnwenderDaten
      push_FrameEscaped(0x00); // MessageGroup_ID - immer 0 -> Bootloader
      push_FrameEscaped(msgCode); // MessageCode
      push_FrameEscaped(msgParam1); // MessageParameter_1
      push_FrameEscaped(msgParam2); // MessageParameter_2
      push_FrameEscaped(msgParam3); // MessageParameter_3
      push_FrameEscaped(msgParam4); // MessageParameter_4

      // Die Page
      for (int i = 0; i < pageBuffer.length(); i++)
        push_FrameEscaped((QByte) pageBuffer[i]);
      transmit();
      timerParams = transmitTimerLastAction_SendMessageBL;
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
  transmit();
  timerParams = transmitTimerLastAction_XBeeCommand;
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
  transmit();
  timerParams = transmitTimerLastAction_XBeeCommand;
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
  transmit();
  timerParams = transmitTimerLastAction_XBeeRemoteCommand;

}
void MainWindow::send_ECB_Reset() {
  switch (applicationMode) {
    case APPLICATION_MODE_USART_Adapter: {
      // Cable-Mode, sende nur eine Nachricht an den Atmega8.
      send_Message(Msg_ECB_AtMega128_ResetCableMode, 0, 0, 0, 0);
      break;
    }
    case APPLICATION_MODE_XBEE_Adapter: {
      switch (USBDeviceXBeeType) {
        case XBeeType_Serie1: {
          if (ECB_XBeeAddress16 == 0xFFFE && ECB_XBeeAddress64 == 0x000000000000FFFF) {
            panelLogView->appendLogViewText("Bitte erst einen Knoten waehlen!");
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
            panelLogView->appendLogViewText("Bitte erst einen Knoten waehlen!");
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

void MainWindow::sl_TextLog(QString sText) {
  statusBar()->showMessage(sText, 5000);
  panelLogView->appendLogViewText(sText);
}
void MainWindow::sl_DispatchMessage(QByteArray receiveBuffer) {

  if (false) {
    printBuffer(receiveBuffer);
  }

  switch (applicationMode) {
    case APPLICATION_MODE_ISP_Adapter:
      DispatchMessage_ISP(receiveBuffer);
      break;
    case APPLICATION_MODE_USART_Adapter:
    case APPLICATION_MODE_XBEE_Adapter:
      DispatchMessage_BL(receiveBuffer);
      break;
  }
}

void MainWindow::DispatchMessage_ISP(QByteArray receiveBuffer) {
  // Eine Nachricht vom Microcontroller wurde empfangen
  uint msgLength = (((uint) receiveBuffer[MsgBuffer_LengthHigh] & 0xFF) << 8) | ((uint) receiveBuffer[MsgBuffer_LengthLow] & 0xFF);
  uint msgCode = (uint) receiveBuffer[MsgBuffer_MsgCode];

  timer->stop();

  // Ausgabe der Message im LogView
  if (false) {
    panelLogView->appendLogViewText("ISP_FUNKTIONALITY\n");
    printBuffer(receiveBuffer);
  }

  switch (msgCode) {
    // BootloaderActions
    //==========================================================================================================
    case Msg_Programmer_Bootloader_Start_Receive:
      panelLogView->appendLogViewText("Programmer-Bootloader Start");

      if (programerFlashRead) {
        programerFlashRead = false;

        progress = new QProgressDialog("0 Pages", "Cancel", 0, 112, this);
        progress->setWindowTitle("Read Flash ...");
        progress->setWindowModality(Qt::WindowModal);
        progress->setFixedWidth(300);
        progress->show();

        send_Message(Msg_Programmer_Bootloader_FlashPageRead_Send, 0, 0, 0, 0);
        panelLogView->appendLogViewText("Begin Programmer Read Flash");
        return;
      }
      if (programerFlashWrite) {
        programerFlashWrite = false;

        progress = new QProgressDialog("0 Pages", "Cancel", 0, 112, this);
        progress->setWindowTitle("Write Flash ...");
        progress->setWindowModality(Qt::WindowModal);
        progress->setFixedWidth(300);
        progress->show();

        send_Message(Msg_Programmer_Bootloader_FlashPageWrite_Send, 0, 0, 0, 0, panelHexViewer->getPage(0, ProgrammerFlashPageSize));

        panelLogView->appendLogViewText("Begin Programmer Write Flash");
        return;
      }
      send_Message(Msg_Programmer_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
      break;
    case Msg_Programmer_Bootloader_End_Receive:
      panelLogView->appendLogViewText("Programmer-Bootloader End");
      break;

    case Msg_Programmer_Bootloader_FlashPageReadResponse_Receive: {
      int pageNumberH = ((int) receiveBuffer[MsgBuffer_MsgParam1] & 0xFF) << 8;
      int pageNumberL = (int) receiveBuffer[MsgBuffer_MsgParam2] & 0xFF;
      int pageNumber = pageNumberH + pageNumberL;

      if (!progress->wasCanceled()) {
        if (temporaryFlashBuffer.length() == 0) {
          int PgmSpaceSize = ProgrammerFlashPageSize * ProgrammerNumberFlashPages;
          temporaryFlashBuffer.resize(PgmSpaceSize);
          temporaryFlashBuffer.fill(0xFF);
        }
        int startIndex = pageNumber * ProgrammerFlashPageSize;
        for (uint i = 0; i < msgLength - 5; i++)
          temporaryFlashBuffer[startIndex + i] = receiveBuffer[MsgBuffer_MsgPageStart + i];

        pageNumber++;

        // Die Info für den Nutzer in der Statusleiste
        try {
          progress->setValue(pageNumber);
          progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(ProgrammerNumberFlashPages) + " Pages");
        } catch (...) {
        }

        //naechste Seite
        if (pageNumber < ProgrammerNumberFlashPages) {
          send_Message(Msg_Programmer_Bootloader_FlashPageRead_Send, (QByte) (pageNumber >> 8), (QByte) pageNumber, 0, 0);
        } else {
          panelLogView->appendLogViewText("ProgrammerRead complete.");
          send_Message(Msg_Programmer_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
          panelHexViewer->setBinary(temporaryFlashBuffer);
          temporaryFlashBuffer.clear();

          action_Binary_save->setEnabled(true);
        }
      } else {
        // Benutzer-Abbruch
        panelLogView->appendLogViewText("User-Abbort");
        send_Message(Msg_Programmer_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
      }
      break;
    }//case ProgrammerRead

    case Msg_Programmer_Bootloader_FlashPageWriteResponse_Receive: {
      int pageNumberH = ((int) receiveBuffer[MsgBuffer_MsgParam1] & 0xFF) << 8;
      int pageNumberL = (int) receiveBuffer[MsgBuffer_MsgParam2] & 0xFF;
      int pageNumber = pageNumberH + pageNumberL;
      int responseState = receiveBuffer[MsgBuffer_MsgParam3];

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
            send_Message(Msg_Programmer_Bootloader_FlashPageWrite_Send, (QByte) (pageNumber >> 8), (QByte) pageNumber, 0, 0, panelHexViewer->getPage(
                pageNumber, ProgrammerFlashPageSize));
          }
        } else {
          send_Message(Msg_Programmer_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
          panelLogView->appendLogViewText("Write Programmer complete.");
          progress->hide();
        }
      } else {
        // Schreiben der Seite nicht erfolgreich!!!
        send_Message(Msg_Programmer_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
        panelLogView->appendLogViewText("Write Programmer abbort, error");
      }
      break;
    }//case ProgrammerWrite


      // TargetDeviceActions
      //==========================================================================================================
    case Msg_Programmer_BeginTransactionResponse_Receive: // Bestätigung erfolgreich/nicht erfolgreich EnableProgrammingMode
    {
      uint returnCode = receiveBuffer[MsgBuffer_MsgParam1];
      uint actionCommand = receiveBuffer[MsgBuffer_MsgParam2];

      if (returnCode == 0) {
        // Fuehre Aktionen durch wie: ReadFuseBits, ReadSignatureBytes, Read/Write Pages ...
        switch (actionCommand) {
          case Msg_Programmer_TargetSignatureBytesRead_Send:
            send_Message(Msg_Programmer_TargetSignatureBytesRead_Send, 0, 0, 0, 0);
            panelLogView->appendLogViewText("Begin Read SignatureBytes");
            break;

          case Msg_Programmer_TargetCalibrationBytesRead_Send:
            send_Message(Msg_Programmer_TargetCalibrationBytesRead_Send, 0, 0, 0, 0);
            panelLogView->appendLogViewText("Begin Read CalibrationBytes");
            break;

          case Msg_Programmer_TargetFuseBitsRead_Send:
            if (avrDevice != 0) {
              send_Message(Msg_Programmer_TargetFuseBitsRead_Send, avrDevice->NumberFuseBytes, 0, 0, 0);

              panelLogView->appendLogViewText("Begin Read FuseBits");
            }
            break;

          case Msg_Programmer_TargetFuseBitsWrite_Send:
            if (avrDevice != 0) {
              send_Message(Msg_Programmer_TargetFuseBitsWrite_Send, avrDevice->NumberFuseBytes, avrDevice->FuseBytes[0]->Value, avrDevice->FuseBytes[1]->Value,
                  avrDevice->FuseBytes[2]->Value);

              panelLogView->appendLogViewText("Begin Write FuseBits");
            }
            break;

          case Msg_Programmer_TargetFlashPageRead_Send:
            if (avrDevice != 0) {
              progress = new QProgressDialog("0 Pages", "Cancel", 0, avrDevice->NumberPages, this);
              progress->setWindowTitle("Read Flash ...");
              progress->setWindowModality(Qt::WindowModal);
              progress->setFixedWidth(300);
              progress->show();

              send_Message(Msg_Programmer_TargetFlashPageRead_Send, 0, 0, (QByte) (avrDevice->PageSizeBytes >> 8), (QByte) avrDevice->PageSizeBytes);
              panelLogView->appendLogViewText("Begin Read Flash");
            }
            break;

          case Msg_Programmer_TargetFlashPageWrite_Send:
            if (avrDevice != 0) {
              progress = new QProgressDialog("0 Pages", "Cancel", 0, avrDevice->NumberPages, this);
              progress->setWindowTitle("Write Flash ...");
              progress->setWindowModality(Qt::WindowModal);
              progress->setFixedWidth(300);
              progress->show();

              send_Message(Msg_Programmer_TargetChipErase_Send, 1, 0, 0, 0);
              panelLogView->appendLogViewText("Begin Chip Erase");
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
        panelLogView->appendLogViewText("Error TargetDevice-InitProgrammMode, please check Connection or TargetSpeed");
      }
      break;
    }
    case Msg_Programmer_EndTransactionResponse_Receive: // Bestaetigung erfolgreich/nicht erfolgreich EnableProgrammingMode
    {
      if (progress != 0) {
        progress->setValue(progress->maximum());
      }
      panelLogView->appendLogViewText("Function completed.");
      break;
    }
    case Msg_Programmer_TargetSignatureBytesReadResponse_Receive: {
      uint targetSignature_0 = (uint) (receiveBuffer[MsgBuffer_MsgParam1] & 0xFF);
      uint targetSignature_1 = (uint) (receiveBuffer[MsgBuffer_MsgParam2] & 0xFF);
      uint targetSignature_2 = (uint) (receiveBuffer[MsgBuffer_MsgParam3] & 0xFF);
      uint targetSignature = targetSignature_0 << 16 | targetSignature_1 << 8 | targetSignature_2;

      avrDevice = avrDeviceList->getDevice(targetSignature);

      if (avrDevice != 0) {
        action_Target_CalibrationBytes_read->setEnabled(true);
        action_Target_ShowFuseDialog->setEnabled(true);
        //action_Target_FuseBytes_write->setEnabled(true);
        action_Target_Flash_read->setEnabled(true);
        if (hasBinary)
          action_Target_Flash_write->setEnabled(true);

        QString s = *avrDevice->Name;
        panelLogView->appendLogViewText(s);
        panelSetting->setTargetDeviceName(s);
      } else {
        QString s = "unknown target device";
        panelLogView->appendLogViewText(s);
        panelSetting->setTargetDeviceName(s);

        action_Target_CalibrationBytes_read->setEnabled(false);
        action_Target_ShowFuseDialog->setEnabled(false);
        action_Target_Flash_read->setEnabled(false);
        action_Target_Flash_write->setEnabled(false);
      }

      send_Message(Msg_Programmer_EndTransaction_Send, 0, 0, 0, 0);

      // automaticMode
      if (automaticMode) {
        if ((avrDevice != 0 && avrDevice->Name->compare(targetNameArg) == 0)) {
          sl_ISP_AVRDevice_Flash_write();
        } else {
          QMessageBox::warning(this, tr("Application"), tr("Wrong Target found, aborted!"));
        }
      }
      break;
    }
    case Msg_Programmer_TargetCalibrationBytesReadResponse_Receive: {
      uint targetCalibrationByte_1 = (uint) (receiveBuffer[MsgBuffer_MsgParam1] & 0xFF);
      uint targetCalibrationByte_2 = (uint) (receiveBuffer[MsgBuffer_MsgParam2] & 0xFF);
      uint targetCalibrationByte_3 = (uint) (receiveBuffer[MsgBuffer_MsgParam3] & 0xFF);
      uint targetCalibrationByte_4 = (uint) (receiveBuffer[MsgBuffer_MsgParam4] & 0xFF);

      panelLogView->appendLogViewText("CalibrationsBytes received:");
      panelLogView->appendLogViewText("  1MHz: 0x" + QString::number(targetCalibrationByte_1, 16).toUpper());
      panelLogView->appendLogViewText("  2MHz: 0x" + QString::number(targetCalibrationByte_2, 16).toUpper());
      panelLogView->appendLogViewText("  4MHz: 0x" + QString::number(targetCalibrationByte_3, 16).toUpper());
      panelLogView->appendLogViewText("  8MHz: 0x" + QString::number(targetCalibrationByte_4, 16).toUpper());

      send_Message(Msg_Programmer_EndTransaction_Send, 0, 0, 0, 0);

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
    case Msg_Programmer_TargetFuseBitsReadResponse_Receive: {
      uint targetFuseBits_0 = (uint) (receiveBuffer[MsgBuffer_MsgParam1] & 0xFF);
      uint targetFuseBits_1 = (uint) (receiveBuffer[MsgBuffer_MsgParam2] & 0xFF);
      uint targetFuseBits_2 = (uint) (receiveBuffer[MsgBuffer_MsgParam3] & 0xFF);

      panelLogView->appendLogViewText("FuseBits received:");
      panelLogView->appendLogViewText("  Low: 0x" + QString::number(targetFuseBits_0, 16).toUpper());
      panelLogView->appendLogViewText(" High: 0x" + QString::number(targetFuseBits_1, 16).toUpper());
      panelLogView->appendLogViewText("  Ext: 0x" + QString::number(targetFuseBits_2, 16).toUpper());

      if (avrDevice != 0) {
        avrDevice->FuseBytes[0]->Value = targetFuseBits_0;
        avrDevice->FuseBytes[1]->Value = targetFuseBits_1;
        avrDevice->FuseBytes[2]->Value = targetFuseBits_2;
        avrFuseDialog->setAvrDevice(avrDevice);
      }

      send_Message(Msg_Programmer_EndTransaction_Send, 0, 0, 0, 0);
      break;
    }
    case Msg_Programmer_TargetFuseBitsWriteResponse_Receive: {
      send_Message(Msg_Programmer_EndTransaction_Send, 0, 0, 0, 0);
      break;
    }
    case Msg_Programmer_TargetFlashPageReadResponse_Receive: {
      int pageNumberH = ((int) receiveBuffer[MsgBuffer_MsgParam1] & 0xFF) << 8;
      int pageNumberL = (int) receiveBuffer[MsgBuffer_MsgParam2] & 0xFF;
      int pageNumber = pageNumberH + pageNumberL;

      if (avrDevice != 0) {
        if (!progress->wasCanceled()) {
          if (temporaryFlashBuffer.length() == 0) {
            int PgmSpaceSize = avrDevice->PageSizeBytes * avrDevice->NumberPages;
            temporaryFlashBuffer.resize(PgmSpaceSize);
            temporaryFlashBuffer.fill(0xFF);
          }
          int startIndex = pageNumber * avrDevice->PageSizeBytes;
          for (uint i = 0; i < msgLength - 5; i++)
            temporaryFlashBuffer[startIndex + i] = receiveBuffer[MsgBuffer_MsgPageStart + i];

          pageNumber++;

          // Die Info für den Nutzer in der Statusleiste
          try {
            progress->setValue(pageNumber);
            progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(avrDevice->NumberPages) + " Pages");
          } catch (...) {
          }

          //naechste Seite
          if (pageNumber < avrDevice->NumberPages) {
            send_Message(Msg_Programmer_TargetFlashPageRead_Send, (QByte) (pageNumber >> 8), (QByte) pageNumber, (QByte) (avrDevice->PageSizeBytes >> 8),
                (QByte) avrDevice->PageSizeBytes);
          } else {
            panelLogView->appendLogViewText("DeviceRead complete.");
            send_Message(Msg_Programmer_EndTransaction_Send, 0, 0, 0, 0);
            panelHexViewer->setBinary(temporaryFlashBuffer);
            temporaryFlashBuffer.clear();

            action_Binary_save->setEnabled(true);
          }
        } else {
          // Benutzer-Abbruch
          panelLogView->appendLogViewText("User-Abbort");
          send_Message(Msg_Programmer_EndTransaction_Send, 0, 0, 0, 0);
        }
      }//if(avrDevice!=0)
      break;
    }
    case Msg_Programmer_TargetFlashPageWriteResponse_Receive: {
      int pageNumberH = ((int) receiveBuffer[MsgBuffer_MsgParam1] & 0xFF) << 8;
      int pageNumberL = (int) receiveBuffer[MsgBuffer_MsgParam2] & 0xFF;
      int pageNumber = pageNumberH + pageNumberL;
      int responseState = receiveBuffer[MsgBuffer_MsgParam3];

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
              send_Message(Msg_Programmer_TargetFlashPageWrite_Send, (QByte) (pageNumber >> 8), (QByte) pageNumber, (QByte) (avrDevice->PageSizeBytes >> 8),
                  (QByte) avrDevice->PageSizeBytes, panelHexViewer->getPage(pageNumber, avrDevice->PageSizeBytes));
            }
          } else {
            send_Message(Msg_Programmer_EndTransaction_Send, 0, 0, 0, 0);
            panelLogView->appendLogViewText("Write Target-Device complete.");
            if (automaticMode)
              close();
          }
        } else {
          // Schreiben der Seite nicht erfolgreich!!!
          panelLogView->appendLogViewText("Write Target-Device abbort, error");
          send_Message(Msg_Programmer_EndTransaction_Send, 0, 0, 0, 0);
        }
      }
      break;
    }
    case Msg_Programmer_TargetChipEraseResponse_Receive: {
      panelLogView->appendLogViewText("Chip Erase complete.");

      int actionCommand = receiveBuffer[MsgBuffer_MsgParam1];
      ;

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

        send_Message(Msg_Programmer_TargetFlashPageWrite_Send, (QByte) (pageNumber >> 8), (QByte) pageNumber, (QByte) (avrDevice->PageSizeBytes >> 8),
            (QByte) avrDevice->PageSizeBytes, panelHexViewer->getPage(pageNumber, avrDevice->PageSizeBytes));
        panelLogView->appendLogViewText("Begin Write Flash");
      }
      break;
    }
    default: {
      panelLogView->appendLogViewText("ReturnCode = 0x" + QString::number((QByte) msgCode, 16).toUpper());
      break;
    }
  } // switch
}
void MainWindow::DispatchMessage_BL(QByteArray receiveBuffer) {
  // Eine Nachricht vom Microcontroller wurde empfangen
  // --------------------------------------------------
  //  0 QByte StartDelimiter;
  //  1 QByte Length_MSB;
  //  2 QByte Length_LSB;
  //  3 QByte API_ID;
  //  4 ...

  //int msgLength = ((QByte)receiveBuffer[1]&0xFF)<<8 | ((QByte)receiveBuffer[2]&0xFF);
  int msgApi_Id = ((QByte) receiveBuffer[3] & 0xFF);
  int msgGroup = -1;
  int msgCode = -1;
  int msgParam1 = -1;
  int msgParam2 = -1;
  int msgParam3 = -1;
  int msgParam4 = -1;
  int msgParam5 = -1;
  int msgPageSegmentStart = -1;

  // Stoppe TransmitTimer
  timer->stop();

  // Ausgabe der Message im LogView
  //panelLogView->appendLogViewText("IN:");
  //printBuffer(receiveBuffer);


  switch (msgApi_Id) {
    case API_XBee_AT_Command_Response:
      dispatch_XbeeCommandResponse(receiveBuffer);
      break;
    case API_Cable_TransmitReceive:
      // +----+---+----+-----+----------+---------+-----------+-----------+-----------+-----------+-----------+-----------+----+
      // | 7E | Length | API | msgGroup | msgCode | msgParam1 | msgParam2 | msgParam3 | msgParam4 | msgParam5 | (msgData) | CS |
      // +----+---+----+-----+----------+---------+-----------+-----------+-----------+-----------+-----------+-----------+----+
      msgGroup = (QByte) receiveBuffer[4];
      msgCode = (QByte) receiveBuffer[5];
      msgParam1 = (QByte) receiveBuffer[6];
      msgParam2 = (QByte) receiveBuffer[7];
      msgParam3 = (QByte) receiveBuffer[8];
      msgParam4 = (QByte) receiveBuffer[9];
      msgParam5 = (QByte) receiveBuffer[10];
      msgPageSegmentStart = 11;
      break;
    case API_XBee_Receive_Packet_16Bit:
      // +----+----+-----+-----+--------+---------+------+---------+----------+---------+-----------+-----------+-----------+-----------+-----------+-----------+----+
      // | 7E |  Length  | API | SourceAddr_16Bit | RSSI | Options | msgGroup | msgCode | msgParam1 | msgParam2 | msgParam3 | msgParam4 | msgParam5 | (msgData) | CS |
      // +----+----+-----+-----+--------+---------+------+---------+----------+---------+-----------+-----------+-----------+-----------+-----------+-----------+----+
      msgGroup = (QByte) receiveBuffer[8];
      msgCode = (QByte) receiveBuffer[9];
      msgParam1 = (QByte) receiveBuffer[10];
      msgParam2 = (QByte) receiveBuffer[11];
      msgParam3 = (QByte) receiveBuffer[12];
      msgParam4 = (QByte) receiveBuffer[13];
      msgParam5 = (QByte) receiveBuffer[14];
      msgPageSegmentStart = 15;
      break;
    case API_XBeeS2_ZigBee_Receive_Packet:
      // +----+----+-----+-----+--+--+--+--+--+--+--+--+-------+-------+---------+----------+---------+-----------+-----------+-----------+-----------+-----------+-----------+----+
      // | 7E |  Length  | API |     64Bit-Address     | 16Bit_Address | Options | msgGroup | msgCode | msgParam1 | msgParam2 | msgParam3 | msgParam4 | msgParam5 | (msgData) | CS |
      // +----+----+-----+-----+--+--+--+--+--+--+--+--+-------+-------+---------+----------+---------+-----------+-----------+-----------+-----------+-----------+-----------+----+
      msgGroup = (QByte) receiveBuffer[15];
      msgCode = (QByte) receiveBuffer[16];
      msgParam1 = (QByte) receiveBuffer[17];
      msgParam2 = (QByte) receiveBuffer[18];
      msgParam3 = (QByte) receiveBuffer[19];
      msgParam4 = (QByte) receiveBuffer[20];
      msgParam5 = (QByte) receiveBuffer[21];
      msgPageSegmentStart = 22;
      break;
    default: {
      QString s;
      s.append(QString::number((QByte) (msgApi_Id >> 4) & 0x0F, 16).toUpper());
      s.append(QString::number((QByte) (msgApi_Id >> 0) & 0x0F, 16).toUpper());
      panelLogView->appendLogViewText("Unknown Api-Code = 0x" + s + " received.");
      printBuffer(receiveBuffer);
      return;
    }
  } //end switch api

  //==============================================================================================
  // Werte keine Nachrichten aus, die nicht für den Bootloader sind!
  if (msgGroup != 0x00)
    return;

  switch (msgCode) {
    case Msg_ECB_Bootloader_Bootloader_Start_Received: // Bootloader-Start
    {
      QString line;

      line.append("ECB-Bootloader-Start: Extended Controller Board ");
      if (msgParam3 == 1)
        line.append("present.");
      else
        line.append("not present.");
      panelLogView->appendLogViewText(line);

      if (ECB_Bootloader_FlashPage_Read) {
        ECB_Bootloader_FlashPage_Read = false;
        ECB_OperationRetries = 0;

        progress = new QProgressDialog("0 Pages", "Cancel", 0, ECB_AtMega128_NumberOfPages, this);
        progress->setWindowTitle("Read Flash ...");
        progress->setWindowModality(Qt::WindowModal);
        progress->setFixedWidth(300);
        progress->show();

        panelLogView->appendLogViewText("Begin Read Flash");
        send_Message(Msg_ECB_Bootloader_FlashPageRead_Send, // MessageCode
            0, // PageNumber High
            0, // PageNumber Low
            0, // (Controller) wird nicht weiter unterstützt!
            0 // SegmentNumber
        );
        return;
      }
      if (ECB_Bootloader_FlashPage_Write) {
        ECB_Bootloader_FlashPage_Write = false;
        ECB_OperationRetries = 0;

        progress = new QProgressDialog("0 Pages", "Cancel", 0, ECB_AtMega128_NumberOfPages, this);
        progress->setWindowTitle("Write Flash ...");
        progress->setWindowModality(Qt::WindowModal);
        progress->setFixedWidth(300);
        progress->show();

        panelLogView->appendLogViewText("Begin Write Flash");
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
          if (!(pageNumber < ECB_AtMega128_NumberOfPages))
            break;
          pageNumber++;
        }

        // Das XbeeModul Serie1 ist noch beschäftigt
        // ('Plug and Play' Senden BootloaderStart im Bootloader)
        sleep(500);

        // Sende nun die Page an den Bootloader, jedoch in 4 Segmente unterteilt:
        // Das Funkmodul XBeeS2 kann nur maximal 72 Zeichen in einem Packet uebertragen!
        // Sende nun das erste Segment und erwarte daraufhin eine Antwort vom Bootloader!
        // Sende dann auf die Bestätigung vom Bootloader die weiteren Segmente.
        QByteArray pageSegmentBuffer = panelHexViewer->getPage(pageNumber * 4, ECB_AtMega128_PageSize_Byte / 4);
        send_Message(Msg_ECB_Bootloader_FlashPageWrite_Send, // MessageCode
            (QByte) (pageNumber >> 8), // PageNumber High
            (QByte) (pageNumber >> 0), // PageNumber Low
            (QByte) 0, // (Controller) wird nicht weiter unterstützt
            0, // PageSegmentNumber
            pageSegmentBuffer // PageSegmentData
        );
        return;
      }

      // Wenn keine der vorherigen Aktionen zutrifft, dann beende den Bootloader!
      send_Message(Msg_ECB_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
      break;
    }//end case Bootloader_Start


    case Msg_ECB_Bootloader_Bootloader_End_Received: // Bootloader-Ende
    {
      panelLogView->appendLogViewText("ECB-Bootloader-End");
      if (progress != 0) {
        if (!progress->isHidden()) {
          panelLogView->appendLogViewText("Operation Error.");
          panelLogView->appendLogViewText("Operation NOT complete.");
        }
        progress->hide();
        progress->setValue(progress->maximum());
      }

      break;
    }//end case Bootloader_End


      //-------------------------------------------------------------------------------------------------------------
      // PageRead:
    case Msg_ECB_Bootloader_FlashPageRead_ResponseReceived: {
      int pageNumber = msgParam1 << 8 | msgParam2;
      int controller = msgParam3;
      int pageSegment = msgParam4;
      int operationState = msgParam5;

      if (operationState != OperationSucceeded) {
        // Das angeforderte Segment einer Seite konnte (wiederholt) nicht ausgelesen werden.
        // Erhöhe den Wiederholungszähler
        ECB_OperationRetries++;

        // Solange die maximale Wiederholrate (für erfolglose Operationen) noch nicht erreicht wurde,
        // wird die Operation wiederholt.
        if (ECB_OperationRetries < ECB_OperationRetriesMax) {
          // Sende erneut Anfrage zum Auslesen dieser Seite aus dem Flash!
          send_Message(Msg_ECB_Bootloader_FlashPageRead_Send, // MessageCode
              msgParam1, // PageNumber High
              msgParam2, // PageNumber Low
              msgParam3, // Controller
              msgParam4 // SegmentNumber
          );
        } else {
          printMessageErrorCode(operationState);
          panelLogView->appendLogViewText("Read Flash Error.");
          panelLogView->appendLogViewText("Read End.");
          send_Message(Msg_ECB_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
          if (progress != 0) {
            progress->hide();
            progress->setValue(progress->maximum());
          }
        }
      } else {
        // PageRead: Das angeforderte Segment einer Seite wurde erfolgreich aus
        // dem Programmspeicher ausgelesen und übersendet.
        // Fahre mit dem nächsten Segment/ der nächsten Seite fort.

        if (pageNumber < 0 || pageSegment < 0)
          break;
        int startIndex = pageNumber * ECB_AtMega128_PageSize_Byte + 64 * pageSegment;

        // Gab es einen Benutzer-Abbruch?
        if (progress->wasCanceled()) {
          // Benutzer-Abbruch
          panelLogView->appendLogViewText("User-Abbort");
          send_Message(Msg_ECB_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
        } else {
          // Kein Benutzer-Abbruch, also weiter ...

          // Wurde der Speicherbereich zur Aufnahme des Programmspeicher aus dem ECB schon initialisiert?
          if (temporaryFlashBuffer.length() == 0) {
            // Reserviere einen Speicherbereich von der Größe des Programmspeicher des Mikrokontroller!
            int iFlashSize = ECB_AtMega128_PageSize_Byte * ECB_AtMega128_NumberOfPages;
            temporaryFlashBuffer.resize(iFlashSize);
            temporaryFlashBuffer.fill(0xFF);
          }

          // Trage das erhaltene PageSegment in den temporären Puffer ein
          for (int i = 0; i < 64; i++)
            temporaryFlashBuffer[startIndex + i] = receiveBuffer[i + msgPageSegmentStart];

          // Wenn noch nicht alle Segmente dieser Seite erhalten wurden, dann nächstes Segment vom ECB anfordern!
          if (pageSegment < 3) {
            ECB_OperationRetries = 0; // Setze den Wiederholungszähler für Misserfolge zurück!
            // Fordere das nächste Segment an!
            send_Message(Msg_ECB_Bootloader_FlashPageRead_Send, // MessageCode
                (QByte) (pageNumber >> 8), // PageNumber High
                (QByte) (pageNumber >> 0), // PageNumber Low
                (QByte) (controller), // Controller
                (QByte) (pageSegment + 1) // Segment
            );
            break;
          }

          // Fordere nun die nächste Seite an.
          pageNumber++;

          // Die Info für den Nutzer in der Statusleiste
          try {
            progress->setValue(pageNumber);
            progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(ECB_AtMega128_NumberOfPages) + " Pages");
          } catch (...) {
          }

          // Nächste Seite anfordern, wenn noch nicht alle Seiten ausgelesen wurden!
          if (pageNumber < ECB_AtMega128_NumberOfPages) {
            send_Message(Msg_ECB_Bootloader_FlashPageRead_Send, // MessageCode
                (QByte) (pageNumber >> 8), // PageNumber High
                (QByte) (pageNumber >> 0), // PageNumber Low
                0, // (Controller) wird nicht weiter unterstützt
                0 // Segment
            );
          } else {
            // Der Programmspeicher des Mikrokontroller wurde kommplett ausgelesen.
            panelLogView->appendLogViewText("Read complete.");
            panelHexViewer->setBinary(temporaryFlashBuffer);
            action_Binary_save->setEnabled(true);
            temporaryFlashBuffer.clear();
            send_Message(Msg_ECB_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
            if (progress != 0) {
              progress->hide();
              progress->setValue(progress->maximum());
            }
          } // nächste Seite Anfordern
        } // kein Benutzer-Abbruch
      } // keine Fehlermeldung
      break;
    } // End_PageReadResponse
      //-------------------------------------------------------------------------------------------------------------


      //-------------------------------------------------------------------------------------------------------------
      // PageWrite
    case Msg_ECB_Bootloader_FlashPageWrite_ResponseReceived: {
      int pageNumber = msgParam1 << 8 | msgParam2;
      int controller = msgParam3;
      int pageSegment = msgParam4;
      int operationState = msgParam5;

      // Werte den Status der Operation aus!
      if (operationState == PageWrite_SegmentReceive_Confirmed_from_ECB) {
        // PageWrite: Das ECB hat den Empfang eines Page-Segmentes bestätigt.
        // Sende nun das nächste Segment. Mit Erhalt des letzten (vierten)
        // Segmentes einer Seite, wird diese in den Programmspeicher geschrieben.

        pageSegment++;
        ECB_OperationRetries = 0;
        QByteArray pageSegmentBuffer = panelHexViewer->getPage(pageNumber * 4 + pageSegment, ECB_AtMega128_PageSize_Byte / 4);

        send_Message(Msg_ECB_Bootloader_FlashPageWrite_Send, // MessageCode
            (QByte) (pageNumber >> 8), // PageNumber High
            (QByte) (pageNumber >> 0), // PageNumber Low
            (QByte) controller, // Controller
            (QByte) pageSegment, // PageSegment
            pageSegmentBuffer // PageSegmentData
        );
      } else if (operationState != OperationSucceeded) {
        // Operation war nicht erfolgreich.
        ECB_OperationRetries++;

        // Nur bei Schreibfehler nochmals versuchen!
        if ((operationState == WriteError) && (ECB_OperationRetries < ECB_OperationRetriesMax)) {
          // Sende wiederholt die gesammte Page!
          // Das Funkmodul XBeeS2 kann nur maximal 72 Zeichen in einem Packet uebertragen!
          // Übertrage nun das erstes Segment der Page.
          QByteArray pageSegmentBuffer = panelHexViewer->getPage(pageNumber * 4, ECB_AtMega128_PageSize_Byte / 4);
          send_Message(Msg_ECB_Bootloader_FlashPageWrite_Send, // MessageCode
              (QByte) msgParam1, // PageNumber High
              (QByte) msgParam2, // PageNumber Low
              (QByte) msgParam3, // Controller
              0, // PageSegment
              pageSegmentBuffer // PageSegmentData
          );
        } else {
          // Die Seite konnte nicht in en Programmspeicher des ECB geschrieben werden, Abbruch durch Applikation!
          printMessageErrorCode(msgParam4);
          panelLogView->appendLogViewText("Write Flash Error.");
          panelLogView->appendLogViewText("Write End.");
          send_Message(Msg_ECB_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
          if (progress != 0) {
            progress->hide();
            progress->setValue(progress->maximum());
          }
        } // End - Abbruch, nach Fehler ...
      } else {
        // Operation erfolgreich!

        // Gab es einen Benutzer-Abbruch?
        if (progress->wasCanceled()) {
          // Benutzer-Abbruch: Schreiben der Seite nicht erfolgreich!!!
          panelLogView->appendLogViewText("Write Flash Abort.");
          send_Message(Msg_ECB_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
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
            if (!(pageNumber < ECB_AtMega128_NumberOfPages))
              break;
            pageNumber++;
          }

          // Die Info für den Nutzer in der Statusleiste
          try {
            progress->setValue(pageNumber);
            progress->setLabelText(QString::number(pageNumber) + "/" + QString::number(ECB_AtMega128_NumberOfPages) + " Pages");
          } catch (...) {
          }

          if ((pageNumber < ECB_AtMega128_NumberOfPages) && panelHexViewer->hasPage(pageNumber, ECB_AtMega128_PageSize_Byte)) {
            // Sende nun die Page an den Bootloader, jedoch in 4 Segmente unterteilt:
            // Das Funkmodul XBeeS2 kann nur maximal 72 Zeichen in einem Packet uebertragen!
            // Sende erstes Segment der Seite
            {
              ECB_OperationRetries = 0;
              QByteArray pageSegmentBuffer = panelHexViewer->getPage(pageNumber * 4, ECB_AtMega128_PageSize_Byte / 4);

              send_Message(Msg_ECB_Bootloader_FlashPageWrite_Send, // MessageCode
                  (QByte) (pageNumber >> 8), // PageNumber High
                  (QByte) (pageNumber >> 0), // PageNumber Low
                  0, // (Controller) wird nicht weiter unterstützt
                  0, // PageSegment
                  pageSegmentBuffer // PageSegmentData
              );
            }// end for Segments
          } else {
            send_Message(Msg_ECB_Bootloader_BootloaderEnd_Send, 0, 0, 0, 0);
            panelLogView->appendLogViewText("Write complete.");
            if (progress != 0) {
              progress->hide();
              progress->setValue(progress->maximum());
            }
          }
        } // End kein Benutzer-Abbruch
      } // End keine Fehlermeldung
      break;
    } // End PageWrite
      //-------------------------------------------------------------------------------------------------------------


    default: {
      QString s;
      s.append(QString::number((QByte) (msgCode >> 4) & 0x0F, 16).toUpper());
      s.append(QString::number((QByte) (msgCode >> 0) & 0x0F, 16).toUpper());
      panelLogView->appendLogViewText("Unknown MessageCode = 0x" + s + " received.");
      break;
    }
  }// end switch


}
void MainWindow::dispatch_XbeeCommandResponse(QByteArray receiveBuffer) {
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
      panelLogView->appendLogViewText("Error occured while identifing nodes.");

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
    case WriteError:
      statusBar()->showMessage("WriteError", 5000);
      panelLogView->appendLogViewText("Physical Write Error.");
      break;
    case PageNumberError:
      statusBar()->showMessage("PageNumberError", 5000);
      panelLogView->appendLogViewText("Wrong Page Number. Supported PageNumberRange - 0 ... 479.");
      break;
    case TransmissionSegmentError:
      statusBar()->showMessage("TransmissionSegmentError", 5000);
      panelLogView->appendLogViewText("Transmission-Segment-Error, splitted Transmission incomplete.");
      break;
    case ECBME_NotAvailable:
      statusBar()->showMessage("ECBME_NotAvailable", 5000);
      panelLogView->appendLogViewText("Second Controller Board not available.");
      break;
    case ECBME_Not_Responding:
      statusBar()->showMessage("ECBME_Not_Responding", 5000);
      panelLogView->appendLogViewText("Second Controller Board not responding.");
      break;
    case ECBME_UnknownFunktionParam:
      statusBar()->showMessage("ECBME_UnknownFunktionParam", 5000);
      panelLogView->appendLogViewText("Communication Error, unknown Function-Parameter.");
      break;
    case EXT_SRAM_busy:
      statusBar()->showMessage("EXT_SRAM_busy", 5000);
      panelLogView->appendLogViewText("Second Controller is busy.");
      break;
    case Unknown_Board_Number:
      statusBar()->showMessage("Unknown_Board_Number", 5000);
      panelLogView->appendLogViewText("Unknown Controller Board Number - allowed: (0, 1).");
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

void MainWindow::sl_ISP_AVRDevice_SignatureBytes_read() {
  send_Message(Msg_Programmer_BeginTransaction_Send, (QByte) (programmerWaitCycles >> 8), (QByte) (programmerWaitCycles >> 0),
      Msg_Programmer_TargetSignatureBytesRead_Send, 0);
}
void MainWindow::sl_ISP_AVRDevice_CalibrationBytes_read() {
  send_Message(Msg_Programmer_BeginTransaction_Send, (QByte) (programmerWaitCycles >> 8), (QByte) (programmerWaitCycles >> 0),
      Msg_Programmer_TargetCalibrationBytesRead_Send, 0);
}
void MainWindow::sl_ISP_ShowFuseDialog() {
  if (avrDevice != NULL) {
    avrFuseDialog->setAvrDevice(avrDevice);
    avrFuseDialog->exec();
  }
}
void MainWindow::sl_ISP_AVRDevice_FuseBytes_read() {
  send_Message(Msg_Programmer_BeginTransaction_Send, (QByte) (programmerWaitCycles >> 8), (QByte) (programmerWaitCycles >> 0),
      Msg_Programmer_TargetFuseBitsRead_Send, 0);
}
void MainWindow::sl_ISP_AVRDevice_FuseBytes_write() {
  send_Message(Msg_Programmer_BeginTransaction_Send, (QByte) (programmerWaitCycles >> 8), (QByte) (programmerWaitCycles >> 0),
      Msg_Programmer_TargetFuseBitsWrite_Send, 0);
}
void MainWindow::sl_ISP_AVRDevice_Flash_read() {
  send_Message(Msg_Programmer_BeginTransaction_Send, (QByte) (programmerWaitCycles >> 8), (QByte) (programmerWaitCycles >> 0),
      Msg_Programmer_TargetFlashPageRead_Send, 0);
}
void MainWindow::sl_ISP_AVRDevice_Flash_write() {
  int targetFlashSize = avrDevice->NumberPages * avrDevice->PageSizeBytes;
  int programSize = panelHexViewer->getBinary().length();
  if (targetFlashSize < programSize) {
    // Das zu Schreibende Programm pass nicht komplett in das Zeilsystem hinein!!
    QMessageBox::warning(this, tr("ProgrammSizeError"), tr(
        "The loaded programm-size is larger than the available programm-space in targetsystem. Action abborted."), QMessageBox::Ok, QMessageBox::Ok);
    return;
  }
  send_Message(Msg_Programmer_BeginTransaction_Send, (QByte) (programmerWaitCycles >> 8), (QByte) (programmerWaitCycles >> 0),
      Msg_Programmer_TargetFlashPageWrite_Send, 0);
}
void MainWindow::sl_ISP_AVRDevice_Flash_update_write() {

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
  send_Message(Msg_Programmer_BeginTransaction_Send, (QByte) (programmerWaitCycles >> 8), (QByte) (programmerWaitCycles >> 0),
      Msg_Programmer_TargetFlashPageWrite_Send, 0);
}
void MainWindow::sl_ISP_AVRDevice_EEPROM_read() {
  //  bool_Target_EEprom_read = true;
  //  send_TargetMessage(Msg_Programmer_BeginTransaction_Send);
}
void MainWindow::sl_ISP_AVRDevice_EEPROM_write() {
  //  bool_Target_EEProm_write = true;
  //  send_TargetMessage(Msg_Programmer_BeginTransaction_Send);
}

void MainWindow::sl_ECB_reset() {
  send_ECB_Reset();
}
void MainWindow::sl_ECB_Bootloader_Flash_read() {
  ECB_Bootloader_FlashPage_Read = true;
  send_ECB_Reset();
}
void MainWindow::sl_ECB_Bootloader_Flash_write() {
  ECB_Bootloader_FlashPage_Write = true;
  send_ECB_Reset();
}
void MainWindow::sl_ECB_Bootloader_Flash_update_write() {
  if (curFileName != NULL)
  {
    loadFile(curFileName);
    ECB_Bootloader_FlashPage_Write = true;
    send_ECB_Reset();
  }
}

void MainWindow::sl_ISP_Programmer_ping() {
  send_Message(Msg_Programmer_SignOfLive_Send, 0, 0, 0, 0);
}
void MainWindow::sl_ISP_Programmer_reset() {
  ft232manager->setDTR(1);
  sleep(1);
  ft232manager->setDTR(0);
}
void MainWindow::sl_ISP_Programmer_Flash_read() {
  programerFlashRead = true;
  ft232manager->setDTR(1);
  sleep(1);
  ft232manager->setDTR(0);
}
void MainWindow::sl_ISP_Programmer_Flash_write() {
  int targetFlashSize = ProgrammerNumberFlashPages * ProgrammerFlashPageSize;
  int programSize = panelHexViewer->getBinary().length();
  if (targetFlashSize < programSize) {
    // Das zu Schreibende Programm pass nicht komplett in das Zeilsystem hinein!!
    QMessageBox::warning(this, tr("ProgrammSizeError"), tr(
        "The loaded programm-size is larger than the available programm-space in programmer. Action abborted."), QMessageBox::Ok, QMessageBox::Ok);
    return;
  }
  programerFlashWrite = true;
  ft232manager->setDTR(1);
  sleep(1);
  ft232manager->setDTR(0);
}

void MainWindow::sl_Binary_open() {
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
      if (avrDevice != NULL) {
        action_Target_Flash_write->setEnabled(true);
        action_Target_Flash_update_write->setEnabled(true);
      }
    }
  }
}
bool MainWindow::sl_Binary_save() {
  fileDialog->setAcceptMode(QFileDialog::AcceptSave);
  fileDialog->setFileMode(QFileDialog::AnyFile);
  if (fileDialog->exec()) {
    QString fileName = fileDialog->selectedFiles().at(0);
    return saveFile(fileName);
  }
  return false;
}
void MainWindow::sl_ClearLogView() {
  panelLogView->clearLogViewText();
}
void MainWindow::sl_Close() {
  close();
}
void MainWindow::sl_ScanUSBDevices() {
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
}

void MainWindow::sl_About() {
  QMessageBox::about(
      this,
      tr("About the Application"),
      tr(
          "LUPAE V2.0 (<b>L</b>pzRobots <b>u</b>niversal <b>p</b>rogramm <b>a</b>pplication for <b>E</b>CB) \n is an <b>Application</b> to (re)programm AVR-Microcontrollers."));
}
void MainWindow::sl_USBDevice_Name_changed(QString usbDeviceName) {
  QString actDeviceName = ft232manager->getDeviceName();

  if (actDeviceName.compare(usbDeviceName) != 0) {
    ft232manager->closeDevice();

    int baudrate = 57600;
    baudrate = panelSetting->getUSBDeviceBaudrate().toInt();
    if (debug)
      panelLogView->appendLogViewText("slot_OnSignal_USBDeviceNameChanged: baudrate=" + panelSetting->getUSBDeviceBaudrate());
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
    panelLogView->appendLogViewText("ECB_XBeeAddress16=" + QString::number(ECB_XBeeAddress16, 16).toUpper());

  quint64 ret64 = name.mid(6, 16).toULongLong(&parse_ok, 16);
  if (parse_ok)
    ECB_XBeeAddress64 = ret64;
  else
    ECB_XBeeAddress64 = 0x0000000000FFFF;
  if (debug)
    panelLogView->appendLogViewText("ECB_XBeeAddress64=" + QString::number(ECB_XBeeAddress64, 16).toUpper());
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
      panelLogView->appendLogViewText("Could not address to XBee-RF-module.");
      panelLogView->appendLogViewText("Please check USB-XBee-Adapter and XBee-RF-modul.");
      break;
    case transmitTimerLastAction_XBeeRemoteCommand:
      panelLogView->appendLogViewText("Could not address to XBee-RF-module.");
      panelLogView->appendLogViewText("Please check USB-XBee-Adapter and XBee-RF-modul");
      panelLogView->appendLogViewText("as well remote XBee-Node.");
      break;
    case transmitTimerLastAction_SendMessageBL:
      panelLogView->appendLogViewText("No response from bootloader!");
      panelLogView->appendLogViewText("Please check connection to ECB,");
      panelLogView->appendLogViewText("as well as supply-voltage.");
      break;
    case transmitTimerLastAction_SendMessageISP:
      panelLogView->appendLogViewText("No response from programmer!");
      panelLogView->appendLogViewText("Please check connection to programmer.");
      break;
    default:
      panelLogView->appendLogViewText("unknown error!");
      break;
  }// switch(lastAction)

  ECB_Bootloader_FlashPage_Read = false;
  ECB_Bootloader_FlashPage_Write = false;

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
    // ISP-Adapter: (ISP-Programmer)
    applicationMode = APPLICATION_MODE_ISP_Adapter;
    createMenus(applicationMode);
    action_Target_SignatureBytes_read->setEnabled(true);
    action_Target_CalibrationBytes_read->setEnabled(false);
    action_Target_ShowFuseDialog->setEnabled(false);
    action_Target_Flash_read->setEnabled(false);
    action_Target_Flash_write->setEnabled(false);
    action_Programmer_ping->setEnabled(true);
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
    transmit();
    timerParams = transmitTimerLastAction_XBeeCommand;
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
  panelLogView->appendLogViewText("Binary '" + fileName + "' opended.");

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
  setWindowTitle(tr("%1[*] - %2").arg(shownName).arg(tr("LUPAE V2.0b")));
}

