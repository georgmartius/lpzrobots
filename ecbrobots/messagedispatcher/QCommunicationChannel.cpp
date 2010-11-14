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
 *   Revision 1.2  2010-11-14 20:39:37  wrabe
 *   - save current developent state
 *
 *   Revision 1.1  2010/11/11 15:35:59  wrabe
 *   -current development state of QMessageDispatchServer
 *   -introduction of QCommunicationChannels and QCCHelper
 *
 *                                                                         *
 ***************************************************************************/

#include "QCommunicationChannel.h"

namespace lpzrobots {
  
  QCommunicationChannel::QCommunicationChannel() {
    responseTimer.setInterval(2000);
    connect(&responseTimer, SIGNAL(timeout(uint)), this, SLOT(sl_ResponseTimerExpired(uint)));

    emit sig_TextLog(QCCHelper::toHexNumberString(0x0013A20040329CA0, 16));

    //scanUsbDevices();
  }
  
  QCommunicationChannel::~QCommunicationChannel() {
    clear_usbDeviceManagerList();
  }

  void QCommunicationChannel::sl_ResponseTimerExpired(uint eventId) {
    responseTimer.stop();

    switch (eventId){
      default:
        break;
    }
  }

  void QCommunicationChannel::sleep(ulong msecs) {
    QTime dieTime = QTime::currentTime().addMSecs(msecs);
    while (QTime::currentTime() < dieTime)
      QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
  }

  QStringList QCommunicationChannel::getUsbDeviceList() {
    QStringList sDeviceList = static_usbDeviceManager.getDeviceList();
    return sDeviceList;
  }

  void QCommunicationChannel::clear_usbDeviceManagerList() {
    foreach(QFT232DeviceManager* usbDeviceManager, usbDeviceManagerList)
      {
        usbDeviceManager->closeDevice();
        disconnect(usbDeviceManager, SIGNAL(sig_newData(QByteArray, QFT232DeviceManager*)));
        disconnect(usbDeviceManager, SIGNAL(sig_TextLog(QString)));
        delete (usbDeviceManager);
        usbDeviceManagerList.removeOne(usbDeviceManager);
      }
  }

  void QCommunicationChannel::scanUsbDevices() {
    clear_usbDeviceManagerList();
    QStringList usbDeviceNameList = static_usbDeviceManager.getDeviceList();
    emit sig_TextLog("number USB-Devices found " + QString::number(usbDeviceNameList.length()));
    foreach (QString usbDeviceName, usbDeviceNameList)
      {
        emit sig_TextLog(usbDeviceName);

        QFT232DeviceManager *usbDevice = new QFT232DeviceManager();
        usbDeviceManagerList.append(usbDevice);
        connect(usbDevice, SIGNAL(sig_newData(QByteArray, QFT232DeviceManager*)), this, SLOT(sl_messageReceived(QByteArray, QFT232DeviceManager*)));
        connect(usbDevice, SIGNAL(sig_TextLog(QString)), this, SIGNAL(sig_TextLog(QString)));
        usbDevice->openDeviceByName(usbDeviceName, QCCHelper::getDefaultBaudrateByName(usbDeviceName));

        int usbDeviceType = QCCHelper::getUsbDeviceTaypeByName(usbDeviceName);
        switch (usbDeviceType){
          case QCCHelper::USBDevice_ISP_ADAPTER: {
            QByteArray msg;
            msg.append((char) 0x01);
            msg.append((char) MsgCode_IspProgrammer_Firmware_SoftwareVersionRead);
            usbDevice->writeData(QCCHelper::toIspMessage(msg));
            break;
          }
          case QCCHelper::USBDevice_USART_ADAPTER: {
            QByteArray msg;
            msg.append((char) QCCHelper::MsgCode_ECB_DNSName_Read);
            usbDevice->writeData(QCCHelper::toUsartMessage(msg));
            break;
          }
          case QCCHelper::USBDevice_XBEE_ADAPTER: {
            QByteArray msg;
            msg.append((char) 'H'); // XBEE-AT-COMMAND: read hardware version
            msg.append((char) 'V');
            usbDevice->writeData(QCCHelper::toXBeeATCommand(msg));
            break;
          }
          default: {
            emit sig_TextLog("unknown usb-device detected.");
            break;
          }
        }
      }
  }

  void QCommunicationChannel::send_XBeeATND() {
    static_usbDeviceManager.writeData(QCCHelper::toXBeeATCommand((QByteArray){(QByte) 'N', (QByte) 'D'}));
    responseTimer.start(transmitTimerLastAction_XBeeCommand);
  }

  void QCommunicationChannel::send_XBeeCommand(QByteArray command) {
    static_usbDeviceManager.writeData(QCCHelper::toXBeeATCommand(command));
    responseTimer.start(transmitTimerLastAction_XBeeCommand);
  }

  void QCommunicationChannel::send_XBeeRemoteCommand(QByteArray command) {
    static_usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand(command, ECB_XBeeAddress16, ECB_XBeeAddress64));
    responseTimer.start(transmitTimerLastAction_XBeeRemoteCommand);
  }

  void QCommunicationChannel::send_ECB_Reset() {
    switch (QCCHelper::getApplicationModeByName(static_usbDeviceManager.getDeviceName())){
      case APPLICATION_MODE_USART_Adapter: {
        QByteArray msg;
        msg.append((char) MsgGroup_Identifier_ECBRobotFirmware);
        msg.append((char) MsgCode_ECB_CommandResetCableMode);
        static_usbDeviceManager.writeData(QCCHelper::toUsartMessage(msg));
        break;
      }
      case APPLICATION_MODE_XBEE_Adapter: {
        switch (USBDeviceXBeeType){
          case XBeeType_Serie1:
          case XBeeType_Serie2: {
            if (ECB_XBeeAddress16 == 0xFFFE && ECB_XBeeAddress64 == 0x000000000000FFFF){
              emit sig_TextLog("Bitte erst einen Knoten waehlen!");
              return;
            }
            QByteArray commandToSet_D0_high;
            commandToSet_D0_high.append((char) 'D');
            commandToSet_D0_high.append((char) '0');
            commandToSet_D0_high.append((char) 0x05);

            QByteArray commandToSet_D0_low;
            commandToSet_D0_low.append((char) 'D');
            commandToSet_D0_low.append((char) '0');
            commandToSet_D0_low.append((char) 0x00);

            static_usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand(commandToSet_D0_high, ECB_XBeeAddress16, ECB_XBeeAddress64));
            sleep(100);
            static_usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand(commandToSet_D0_low, ECB_XBeeAddress16, ECB_XBeeAddress64));
            break;
          }
        }//end switch
      }
    }
  }

  void QCommunicationChannel::printMessage(QByteArray buffer) {
    QString hex;
    QString line;

    for (int i = 0; i < buffer.length(); i++){
      line.append(QCCHelper::toHexNumberString(buffer[i], 2));
      line.append(" ");
    }
    emit sig_TextLog(line);
  }

  void QCommunicationChannel::sl_messageReceived(QByteArray received_msg, QFT232DeviceManager *usbDevice) {

    printMessage(received_msg);


    // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
    // ----------------------
    // 0x00 - StartDelemiter
    // 0x01 - Length_HighByte
    // 0x02 - Length_LowByte
    // 0x03 - API-Identifier
    uint api_Identifier = received_msg[3];

    switch (api_Identifier){
      case QCCHelper::API_Cable_TransmitReceive: {
        // Stoppe TransmitTimer
        responseTimer.stop();

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


        // mögliche Quellen:
        // USB-ISP-Adapter   (Bootloader)
        // USB-ISP-Adapter   (Firmware)
        // USB-USART-Adapter (Bootloader)
        // USB-USART-Adapter (ECB_Robot_Firmware)
        break;
      }
      case QCCHelper::API_XBeeS1_Receive_Packet_16Bit: {
        // Stoppe TransmitTimer
        responseTimer.stop();

        // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
        // ----------------------
        // 0x00 - StartDelemiter
        // 0x01 - Length_HighByte
        // 0x02 - Length_LowByte
        // 0x03 - API-Identifier
        // ----------------------
        // 0x04 - SourceAddress16_1
        // 0x05 - SourceAddress16_0
        // 0x06 - RSSI
        // 0x07 - Options
        // ----------------------
        // 0x08 - MsgGroup
        // 0x09 - MsgCode
        // 0x0A - data/params...

        break;
      }
      case QCCHelper::API_XBeeS2_ZigBee_Receive_Packet: {
        // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
        // ----------------------
        // 0x00 - StartDelemiter
        // 0x01 - Length_HighByte
        // 0x02 - Length_LowByte
        // 0x03 - API-Identifier
        // ----------------------
        // 0x04 - Frame-Id
        // ----------------------
        // 0x05 - SourceAddress64_7
        // 0x06 - SourceAddress64_6
        // 0x07 - SourceAddress64_5
        // 0x08 - SourceAddress64_4
        // 0x09 - SourceAddress64_3
        // 0x0A - SourceAddress64_2
        // 0x0B - SourceAddress64_1
        // 0x0C - SourceAddress64_0
        // ----------------------
        // 0x0D - SourceAddress16_1
        // 0x0E - SourceAddress16_0
        // ----------------------
        // 0x0F - Options
        // ----------------------
        // 0x10 - MsgGroup
        // 0x11 - MsgCode
        // 0x12 - data/params...

        break;
      }
      case QCCHelper::API_XBee_AT_Command_Response: {
        // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
        // ----------------------
        // 0x00 - StartDelemiter
        // 0x01 - Length_HighByte
        // 0x02 - Length_LowByte
        // 0x03 - API-Identifier
        // ----------------------
        // 0x04 - Frame-Id
        // ----------------------
        // 0x05 - AT-Command
        // 0x06 - AT-Command
        // 0x07 - Options (0=ok, 1=error, 2=invalid command, 3=invalid param)
        // 0x08 - data...

        break;
      }
      default: {
        emit sig_TextLog("Unknown Api-Code received: 0x" + QCCHelper::toHexNumberString(api_Identifier, 2));
        emit
        sig_TextLog(received_msg);
        return;
      }
    } //end switch api
  }

/*

 void QCommunicationChannel::dispatch_XbeeCommandResponse(QByteArray commandResponse) {

 // Nachrichten-Format:
 // ----------------------
 // 0x00 - StartDelemiter
 // 0x01 - Length_HighByte
 // 0x02 - Length_LowByte
 // 0x03 - API-Identifier  = 0x88
 // ----------------------
 // 0x04 - Frame-Id
 // 0x05 - AT-Command
 // 0x06 - AT-Command
 // 0x07 - Status (0=ok, 1=error, 2=invalid command, 3=invalid parameter)
 // 0x08 - value...

 QWord msgLength = (commandResponse[1]) << 8 | (commandResponse[2] << 0);

 QString Command;
 Command.append((char) commandResponse[5]);
 Command.append((char) commandResponse[6]);

 if (Command.compare("HV") == 0){
 quint16 HardwareVersionNumber = 0;

 HardwareVersionNumber += ((quint16) commandResponse[8] & 0xFF) << 1 * 8;
 HardwareVersionNumber += ((quint16) commandResponse[9] & 0xFF) << 0 * 8;

 switch (HardwareVersionNumber){
 case 0x180B:
 case 0x1842:
 USBDeviceXBeeType = XBeeType_Serie1;
 break;
 case 0x1942:
 USBDeviceXBeeType = XBeeType_Serie2;
 break;
 }
 return;
 }

 if (Command.compare("ND") == 0) // NodeIdentifier-Response
 {
 if ((int) commandResponse[7] != 0) // Status OK?
 {
 emit sig_TextLog("Error occured while identifing nodes.");

 return;
 }

 QString nodeId;
 quint16 ECB_XBeeAddress16_tmp = 0;
 quint64 ECB_XBeeAddress64_tmp = 0;

 // Das XBee sendet als antwort auch ein Paket mit Länge 5,
 // jedoch ohne nützliche Informationen
 if (msgLength <= 5) return;

 ECB_XBeeAddress16_tmp += ((quint16) commandResponse[8] & 0xFF) << 1 * 8;
 ECB_XBeeAddress16_tmp += ((quint16) commandResponse[9] & 0xFF) << 0 * 8;

 ECB_XBeeAddress64_tmp += ((quint64) commandResponse[10] & 0xFF) << 7 * 8;
 ECB_XBeeAddress64_tmp += ((quint64) commandResponse[11] & 0xFF) << 6 * 8;
 ECB_XBeeAddress64_tmp += ((quint64) commandResponse[12] & 0xFF) << 5 * 8;
 ECB_XBeeAddress64_tmp += ((quint64) commandResponse[13] & 0xFF) << 4 * 8;
 ECB_XBeeAddress64_tmp += ((quint64) commandResponse[14] & 0xFF) << 3 * 8;
 ECB_XBeeAddress64_tmp += ((quint64) commandResponse[15] & 0xFF) << 2 * 8;
 ECB_XBeeAddress64_tmp += ((quint64) commandResponse[16] & 0xFF) << 1 * 8;
 ECB_XBeeAddress64_tmp += ((quint64) commandResponse[17] & 0xFF) << 0 * 8;

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
 switch (USBDeviceXBeeType){
 case XBeeType_Serie1: {
 for (int i = 0; i < msgLength - 17; i++)
 nodeId.append((char) commandResponse[19 + i]);
 nodeId.append("'");
 break;
 }
 case XBeeType_Serie2: {
 for (int i = 18; i < msgLength - 6; i++)
 nodeId.append((char) commandResponse[i]);
 nodeId.append("'");
 break;
 }
 }//end switch

 }
 }

 void QECBMessageDispatchServer::scanUsbDevices() {
 //managerList.at(0)->createDeviceList();
 QStringList deviceList = managerList.at(0)->getDeviceList();
 deviceList.removeAll("USB-ISP-Adapter");

 foreach(QString sDeviceName, deviceList)
 {
 QFT232DeviceManager *devMan = new QFT232DeviceManager();
 devMan->openDeviceByName(sDeviceName, QCCHelper::getDefaultBaudrateByName(sDeviceName));

 if (sDeviceName.startsWith("USB-USART-Adapter")){
 // sendMessage zum ECB (direkt verbunden)
 // request_dnsName
 QByteArray msg;

 msg.append();
 send_Message(devMan, msg);

 }else if (sDeviceName.startsWith("USB-XBEE-Adapter")){

 }
 }
 }
 */

}
