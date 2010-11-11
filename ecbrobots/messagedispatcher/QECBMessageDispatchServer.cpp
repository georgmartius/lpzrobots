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
 *   Revision 1.1  2010-11-11 15:35:59  wrabe
 *   -current development state of QMessageDispatchServer
 *   -introduction of QCommunicationChannels and QCCHelper
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "QECBMessageDispatchServer.h"
#include <selforg/stl_adds.h>
#include "constants.h"
#include "QCCHelper.h"

namespace lpzrobots {

  QECBMessageDispatchServer::QECBMessageDispatchServer() :
    QAbstractMessageDispatchServer() {
    managerList.append(new QFT232DeviceManager());
  }

  QECBMessageDispatchServer::~QECBMessageDispatchServer() {
    FOREACH(QList<QFT232DeviceManager*>, managerList, man) {
      delete (*man);
    }
  }


  void QECBMessageDispatchServer::sl_sendMessage(struct _communicationMessage msg) {
  }


  void QECBMessageDispatchServer::scanUsbDevices() {
    //managerList.at(0)->createDeviceList();
    QStringList deviceList = managerList.at(0)->getDeviceList();
    deviceList.removeAll("USB-ISP-Adapter");

    foreach(QString sDeviceName, deviceList)
    {
      QFT232DeviceManager *devMan = new QFT232DeviceManager();
      devMan->openDeviceByName(sDeviceName, QCCHelper::getDefaultBaudrateByName(sDeviceName));

      if(sDeviceName.startsWith("USB-USART-Adapter"))
      {
        // sendMessage zum ECB (direkt verbunden)
        // request_dnsName
        QByteArray msg;

        msg.append();
        send_Message(devMan, msg);

      }else
      if(sDeviceName.startsWith("USB-XBEE-Adapter"))
      {


      }
    }
  }

  void QECBMessageDispatchServer::sl_receiveMessageFromUsbDeviceManager(QString usbDeviceName, QByteArray received_msg)
  {
    if(usbDeviceName.startsWith("USB-ISP-Adapter"))
    {
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

      QByte msgApiIdentifier = received_msg[3];
      if (msgApiIdentifier == Api_ISP_TransmitBootloader || msgApiIdentifier == Api_ISP_TransmitFirmware)
      {
        QByte msgGroup = received_msg[4];
        switch (msgGroup)
        {
          default:
            emit sig_TextLog("<DispatchMessage_ISP> unknown message received");
            break;
        }
      }// if(ApiIdentifier)
    }else
    if(usbDeviceName.startsWith("USB-USART-Adapter") || usbDeviceName.startsWith("USB-XBEE-Adapter"))
    {
      // Stoppe TransmitTimer
      timer->stop();

      // Eine Nachricht vom Microcontroller wurde empfangen
      // --------------------------------------------------
      //  0 QByte StartDelimiter;
      //  1 QByte Length_MSB;
      //  2 QByte Length_LSB;
      //  3 QByte API_ID;
      //  4 ...

      uint msgApi_Id = received_msg[3];
      QByteArray received_msg;
      switch (msgApi_Id)
      {
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
        default:
        {
          QString s;
          s.append(QString::number((QByte) (msgApi_Id >> 4) & 0x0F, 16).toUpper());
          s.append(QString::number((QByte) (msgApi_Id >> 0) & 0x0F, 16).toUpper());
          emit sig_TextLog("Unknown Api-Code = 0x" + s + " received.");
          printBuffer(receiveBuffer);
          return;
        }
      } //end switch api
    }
  }

  void QECBMessageDispatchServer::printBuffer(QByteArray buffer) {
    QString hex;
    QString line;

    for (int i = 0; i < buffer.length(); i++)
    {
      line.append(QString::number((buffer[i] >> 4) & 0x0F, 16).toUpper());
      line.append(QString::number((buffer[i] >> 0) & 0x0F, 16).toUpper());
      line.append(" ");
    }
    emit sig_TextLog(line);
  }


  void QECBMessageDispatchServer::push_Frame(uchar c) {
    transmitBuffer.append(c);
  }
  void QECBMessageDispatchServer::push_FrameEscaped(uchar c) {
    // Von der Prüfsumme ausgeschlossen sind das Startsymbol und das Längenfeld,
    // deswegen erst ab dem 3. Zeichen die Prüfsumme bilden!
    if (2 < transmitBuffer.length())
      transmitBufferCheckSum += c;

    // Ist fuer dieses Zeichen eine Ausnahmebehandlung notwendig?
    if (c == 0x7E || c == 0x7D || c == 0x13 || c == 0x11)
    {
      transmitBuffer.append(0x7D);
      transmitBuffer.append((QByte) (c ^ 0x20));
    } else
    {
      transmitBuffer.append(c);
    }
  }
  bool QECBMessageDispatchServer::transmit(QFT232DeviceManager *ft232manager) {
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
  void QECBMessageDispatchServer::send_Message(QFT232DeviceManager *ft232manager, QByteArray command) {
    QWord length = command.length();

    push_Frame(0x7E); // Startsymbol
    push_FrameEscaped((QByte) (length >> 8)); // Length MSB
    push_FrameEscaped((QByte) (length >> 0)); // Length LSB
    for (int i = 0; i < length; i++)
      push_FrameEscaped(command[i]);
    transmit(ft232manager);
    timerParams = transmitTimerLastAction_SendMessageRaw;

  }
  /*void QECBMessageDispatchServer::send_Message(uchar msgCode, uchar msgParam1, uchar msgParam2, uchar msgParam3, uchar msgParam4) {
    switch (applicationMode)
    {
      case APPLICATION_MODE_ISP_Adapter:
      {
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
      case APPLICATION_MODE_USART_Adapter:
      {
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
      case APPLICATION_MODE_XBEE_Adapter:
      {
        switch (USBDeviceXBeeType)
        {
          case XBeeType_Serie1:
          {
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
          case XBeeType_Serie2:
          {
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
  }*/
  /*void QECBMessageDispatchServer::send_Message(uchar msgCode, uchar msgParam1, uchar msgParam2, uchar msgParam3, uchar msgParam4, QByteArray pageBuffer) {
    switch (applicationMode)
    {
      case APPLICATION_MODE_ISP_Adapter:
      {
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
      case APPLICATION_MODE_USART_Adapter:
      {
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
      case APPLICATION_MODE_XBEE_Adapter:
      {
        switch (USBDeviceXBeeType)
        {
          case XBeeType_Serie1:
          {
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
          case XBeeType_Serie2:
          {
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
  }*/

} // namespace lpzrobots

