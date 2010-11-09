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
#include "QExtAction.h"
#include "panelHexViewer.h"
#include "panelSetting.h"
#include "panelLogView.h"
#include "QAvrFuseDialog.h"
#include "QFT232DeviceManager.h"
#include "avrDeviceList.h"
#include "constants.h"

namespace lpzrobots {

  class MainWindow : public QMainWindow {

    Q_OBJECT

    public:
      MainWindow(QString applicationPathArg);
      QString applicationPath;

      enum ACTION_EVENT {
        //---------------------------------------
        EVENT_APPLICATION_BINARY_OPEN,
        EVENT_APPLICATION_BINARY_SAVE,
        EVENT_APPLICATION_LOGVIEW_CLEAR,
        EVENT_APPLICATION_CLOSE,
        EVENT_APPLICATION_ABOUT,
        EVENT_APPLICATION_SCAN_USBDEVICE,
        //---------------------------------------
        EVENT_ISP_AVRDEVICE_SIGNATURE_READ,
        EVENT_ISP_AVRDEVICE_CALIBRATION_READ,
        EVENT_ISP_AVRDEVICE_FUSES_EDITOR_SHOW,
        EVENT_ISP_AVRDEVICE_FUSES_READ,
        EVENT_ISP_AVRDEVICE_FUSES_WRITE,
        EVENT_ISP_AVRDEVICE_FLASH_READ,
        EVENT_ISP_AVRDEVICE_FLASH_WRITE,
        EVENT_ISP_AVRDEVICE_FLASH_UPDATEWRITE,
        //---------------------------------------
        EVENT_ISP_PROGRAMMER_SOFTWARE_VERSION,
        EVENT_ISP_PROGRAMMER_RESET,
        EVENT_ISP_PROGRAMMER_FLASH_READ,
        EVENT_ISP_PROGRAMMER_FLASH_WRITE,
        //---------------------------------------
        EVENT_ECB_RESET,
        EVENT_ECB_BOOTLOADER_FLASH_READ,
        EVENT_ECB_BOOTLOADER_FLASH_WRITE,
        EVENT_ECB_BOOTLOADER_FLASH_UPDATEWRITE,
        EVENT_ECB_BOOTLOADER_EEPROM_READ,
        EVENT_ECB_BOOTLOADER_EEPROM_WRITE
        //---------------------------------------
      };

      enum NEXT_OPERATION_IDENTIFIER{
          NEXT_OP_NONE,
          NEXT_OP_ISP_PROGRAMMER_FLASH_READ,
          NEXT_OP_ISP_PROGRAMMER_FLASH_WRITE,
          // Bootloader-Aktionen
          NEXT_OP_ECB_BOOTLOADER_FLASH_READ,
          NEXT_OP_ECB_BOOTLOADER_FLASH_WRITE,
          NEXT_OP_ECB_BOOTLOADER_EEPROM_READ,
          NEXT_OP_ECB_BOOTLOADER_EEPROM_WRITE
      };

    protected:
      void closeEvent(QCloseEvent *event);

    private slots:
      void sl_TextLog(QString s);
      void sl_DispatchMessage(QByteArray msg);

      void sl_eventHandler_ispProgrammer(int eventCode);
      void sl_eventHandler_ecbBootloader(int eventCode);
      void sl_eventHandler_application(int eventCode);

      void sl_USBDevice_opened();
      void sl_USBDevice_Name_changed(QString name);
      void sl_USBDevice_Baudrate_changed(QString name);
      void sl_USBDeviceXBeeType_Changed(QString name);
      void sl_AVRDevice_AccessSpeed_changed(QString name);
      void sl_XBeeRemoteNodeIdentifier_changed(QString name);

      void sl_TimerExpired();

    private:
      void push_Frame(uchar c);
      void push_FrameEscaped(uchar c);
      bool transmit(int timerExpiredCode);
      void send_Message(QByteArray command);
      void send_ECB_Reset();
      void send_XBeeCommand(QByteArray command);
      void send_XBeeRemoteCommand(QByte command[], int length);
      void send_XBeeATND();

      void printBuffer(QByteArray buffer);
      void printMessageErrorCode(int errorCode);
      void initPanelSetting();
      void createActions();
      void createToolBars();
      void createMenus(int applicationMode = APPLICATION_MODE_None);
      void createStatusBar();
      void readSettings();
      void writeSettings();
      void loadTargetDeviceParameter();
      bool loadFile(const QString &fileName);
      bool saveFile(const QString &fileName);
      void setCurrentFile(const QString &fileName);
      void setProgrammerWaitCycles(const QString stargetSpeed);
      void sleep(ulong msecs);

      void isp_MessageHandler_Bootloader(QByteArray msg);
      void isp_MessageHandler_Firmware(QByteArray msg);
      void bl_MessageHandler_Bootloader(QByteArray msg);
      void bl_MessageHandler_XBeeCommandResponse(QByteArray msg);

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
      QMenu *ispProgrammerMenu; /// hold the menu-items to control the programmer, i.e. flash the programmer itself!
      QMenu *helpMenu;

      QToolBar *fileToolBar;

      // --- Actions ----------------------------------
      // File-Menu
      QExtAction *action_Binary_open;
      QExtAction *action_Binary_save;
      QExtAction *action_Exit;
      // ISP-Adapter
      QExtAction *action_Target_SignatureBytes_read;
      QExtAction *action_Target_CalibrationBytes_read;
      QExtAction *action_Target_ShowFuseDialog;
      QExtAction *action_Target_Flash_read;
      QExtAction *action_Target_Flash_write;
      QExtAction *action_Target_Flash_update_write;
      // Bootloader
      QExtAction *action_ECB_reset;
      QExtAction *action_ECB_Bootloader_Flash_read;
      QExtAction *action_ECB_Bootloader_Flash_write;
      QExtAction *action_ECB_Bootloader_Flash_update_write;
      QExtAction *action_ECB_Bootloader_EEPROM_read;
      QExtAction *action_ECB_Bootloader_EEPROM_write;
      // Extras
      QExtAction *action_Programmer_SoftwareVersion;
      QExtAction *action_Programmer_reset;
      QExtAction *action_Programmer_Flash_read;
      QExtAction *action_Programmer_Flash_write;
      QExtAction *action_ClearLogView;
      QExtAction *action_SerialPorts_refresh;
      // Help-Menu
      QExtAction *action_About;

      //-- operative Variablen ----------------------------------
      QAVR_DeviceList *avrDeviceList;
      AVRDEVICE *avrDevice;
      QByteArray transmitBuffer;
      QByteArray temporaryBuffer;
      QByte transmitBufferCheckSum;
      QWord USBDeviceXBeeType;
      QWord ECB_OperationRetriesMax;
      QWord ECB_OperationRetries;
      quint16 ECB_XBeeAddress16;
      quint64 ECB_XBeeAddress64;

      int nextOperationState;
      bool hasBinary;
      bool hasAVRDeviceIdedentified;
      ushort programmerWaitCycles;
      QString targetNameArg;

      // umschalten zwischen den beiden Funktionen zur Bediehnung des ISP-Programmer oder des Bootloader
      int applicationMode;
      int timerParams;

      int iDefaultBaudrate_isp;
      int iDefaultBaudrate_usart;
      int iDefaultBaudrate_xbee;
  };

} // namespace lpzrobots


#endif
