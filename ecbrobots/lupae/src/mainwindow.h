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
 *                                                *
 *                                                                         *
 ***************************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QMainWindow>
#include <QTextEdit>
#include <QTimer>
#include <QStyleOptionProgressBarV2>
#include <qdom.h>
#include "types.h"
#include "panelHexViewer.h"
#include "panelSetting.h"
#include "panelLogView.h"
#include "QAvrFuseDialog.h"
#include "QFT232DeviceManager.h"
#include "avrDeviceList.h"
#include "constants.h"

class MainWindow : public QMainWindow {

  Q_OBJECT

  public:
    MainWindow();
    QString applicationPath;

  protected:
    void closeEvent(QCloseEvent *event);

  private slots:
    void sl_Initialize();
    void sl_TextLog(QString s);
    void sl_DispatchMessage(QByteArray msg);
    void sl_Close();
    void sl_Binary_open();
    bool sl_Binary_save();
    void sl_ClearLogView();
    void sl_About();
    void sl_ISP_AVRDevice_SignatureBytes_read();
    void sl_ISP_AVRDevice_CalibrationBytes_read();
    void sl_ISP_AVRDevice_FuseBytes_read();
    void sl_ISP_AVRDevice_FuseBytes_write();
    void sl_ISP_AVRDevice_Flash_read();
    void sl_ISP_AVRDevice_Flash_write();
    void sl_ISP_AVRDevice_Flash_update_write();
    void sl_ISP_AVRDevice_EEPROM_read();
    void sl_ISP_AVRDevice_EEPROM_write();
    void sl_ISP_ShowFuseDialog();
    void sl_ISP_Programmer_ping();
    void sl_ISP_Programmer_reset();
    void sl_ISP_Programmer_Flash_read();
    void sl_ISP_Programmer_Flash_write();
    void sl_ECB_reset();
    void sl_ECB_Bootloader_Flash_read();
    void sl_ECB_Bootloader_Flash_write();
    void sl_ECB_Bootloader_Flash_update_write();

    void sl_USBDevice_opened();
    void sl_USBDevice_Name_changed(QString name);
    void sl_USBDevice_Baudrate_changed(QString name);
    void sl_USBDeviceXBeeType_Changed(QString name);
    void sl_AVRDevice_AccessSpeed_changed(QString name);
    void sl_XBeeRemoteNodeIdentifier_changed(QString name);

    void sl_ScanUSBDevices();
    void sl_TimerExpired();

  private:
    void push_Frame(uchar c);
    void push_FrameEscaped(uchar c);
    bool transmit();
    void send_Message(QByteArray command);
    void send_Message(uchar msgCode, uchar Param1, uchar Param2, uchar Param3, uchar Param4);
    void send_Message(uchar msgCode, uchar Param1, uchar Param2, uchar Param3, uchar Param4, QByteArray buffer);
    void send_ECB_Reset();
    void send_XBeeCommand(QByteArray command);
    void send_XBeeRemoteCommand(QByte command[], int length);
    void send_XBeeATND();

    void printBuffer(QByteArray buffer);
    void printMessageErrorCode(int errorCode);
    void initPanelSetting();
    void createActions();
    void createToolBars();
    void createMenus(int applicationMode=APPLICATION_MODE_None);
    void createStatusBar();
    void readSettings();
    void writeSettings();
    void loadTargetDeviceParameter();
    bool loadFile(const QString &fileName);
    bool saveFile(const QString &fileName);
    void setCurrentFile(const QString &fileName);
    void setProgrammerWaitCycles(const QString stargetSpeed);
    void sleep(ulong msecs);

    // TODO:
    // the future is to combine the two methods ...
    // first is to rewrite the protocoll of the isp-adapter
    void DispatchMessage_ISP(QByteArray msg);
    void DispatchMessage_BL(QByteArray msg);
    void dispatch_XbeeCommandResponse(QByteArray receiveBuffer);

    void setMode(int mode);
    int getDefaultBaudrateByName(QString actDeviceName);

    QTabWidget *tabWidget;
    QPanelHexViewer *panelHexViewer;
    QPanelSetting *panelSetting;
    QPanelLogView *panelLogView;
    QAvrFuseDialog *avrFuseDialog;
    QFT232DeviceManager *ft232manager;
    QTimer *timer;
    QProgressDialog *progress;
    QFileDialog *fileDialog;
    QString curFileName;
    QString deviceFileName;

    QMenu *fileMenu;
    QMenu *ispMenu;
    QMenu *blMenu;
    QMenu *extraMenu;
    QMenu *ispProgrammerMenu;  /// hold the menu-items to control the programmer, i.e. flash the programmer itself!
    QMenu *helpMenu;

    QToolBar *fileToolBar;

    // --- Actions ----------------------------------
    // File-Menu
    QAction *action_Binary_open;
    QAction *action_Binary_save;
    QAction *action_Exit;
    // ISP-Adapter
    QAction *action_Target_SignatureBytes_read;
    QAction *action_Target_CalibrationBytes_read;
    QAction *action_Target_ShowFuseDialog;
    QAction *action_Target_Flash_read;
    QAction *action_Target_Flash_write;
    QAction *action_Target_Flash_update_write;
    QAction *action_Target_EEProm_read;
    QAction *action_Target_EEProm_write;
    // Bootloader
    QAction *action_ECB_reset;
    QAction *action_ECB_Bootloader_Flash_read;
    QAction *action_ECB_Bootloader_Flash_write;
    QAction *action_ECB_Bootloader_Flash_update_write;
    // Extras
    QAction *action_Programmer_ping;
    QAction *action_Programmer_reset;
    QAction *action_Programmer_Flash_read;
    QAction *action_Programmer_Flash_write;
    QAction *action_ClearLogView;
    QAction *action_SerialPorts_refresh;
    // Help-Menu
    QAction *action_About;

    //-- operative Variablen ----------------------------------
    QAVR_DeviceList *avrDeviceList;
    AVRDEVICE *avrDevice;
    QByteArray transmitBuffer;
    QByteArray temporaryFlashBuffer;
    QByte transmitBufferCheckSum;
    QWord USBDeviceXBeeType;
    QWord ECB_OperationRetriesMax;
    QWord ECB_OperationRetries;
    quint16 ECB_XBeeAddress16;
    quint64 ECB_XBeeAddress64;

    // Bootloader-Aktionen
    bool ECB_Bootloader_FlashPage_Read;
    bool ECB_Bootloader_FlashPage_Write;
    bool programerFlashRead;
    bool programerFlashWrite;
    bool automaticMode;
    bool hasBinary;
    ushort programmerWaitCycles;
    QString targetNameArg;

    // umschalten zwischen den beiden Funktionen zur Bediehnung des ISP-Programmer oder des Bootloader
    int applicationMode;
    int timerParams;

    int iDefaultBaudrate_isp;
    int iDefaultBaudrate_usart;
    int iDefaultBaudrate_xbee;
};

#endif
