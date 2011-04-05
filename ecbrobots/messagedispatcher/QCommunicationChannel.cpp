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
 *   Revision 1.10  2011-04-05 12:16:03  guettler
 *   - new tabWidget
 *   - all found DNS devices are shown in tabWidget with a QDNSDeviceWidget each
 *   - QDNSDeviceWidget shows DNS device name, USB adapter name and type,
 *     response time and incoming/outgoing status (if messages are currently sent
 *     or received)
 *
 *   Revision 1.9  2011/02/08 11:30:29  guettler
 *   - cosmetic changes
 *
 *   Revision 1.8  2011/02/04 13:02:10  wrabe
 *   - removed unnecessary debug message
 *   - sendMessage does not set time of responseTimer anymore
 *
 *   Revision 1.7  2011/01/24 16:24:46  guettler
 *   -use new QLog feature
 *
 *   Revision 1.6  2010/11/26 12:22:36  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *   Revision 1.5  2010/11/23 11:08:06  guettler
 *   - some helper functions
 *   - bugfixes
 *   - better event handling
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
 *
 *                                                                         *
 ***************************************************************************/

#include "QCommunicationChannel.h"
#include "QLog.h"

#include <QHash>

namespace lpzrobots {
  
  QCommunicationChannel::QCommunicationChannel(QString usbDeviceName) :
    initialisedState(QCCHelper::STATE_NOT_INITIALISED) {
    responseTimer.setInterval(QCCHelper::RESPONSE_TIME_DEFAULT);
    connect(&responseTimer, SIGNAL(timeout(uint)), this, SLOT(sl_ResponseTimerExpired(uint)));
    connect(&usbDeviceManager, SIGNAL(sig_newData(QByteArray)), this, SLOT(sl_messageReceived(QByteArray)));
    usbDeviceType = QCCHelper::getUsbDeviceTypeByName(usbDeviceName);
    openUsbDevice(usbDeviceName);

    QCCHelper::fillEventDescriptionMap();
  }
  
  QCommunicationChannel::~QCommunicationChannel() {
    clearXbeeRemoteList();
    close();
  }

  void QCommunicationChannel::openUsbDevice(QString usbDeviceName) {
    usbDeviceManager.openDeviceByName(usbDeviceName, QCCHelper::getDefaultBaudrateByName(usbDeviceName));
    if (usbDeviceManager.isDeviceOpened()) {
      initialisedState = QCCHelper::STATE_USBDEVICE_OPENED;
      // XBee HV + XBee Channel + XBee PanID
      // über Signals und Slots abarbeiten!
      QLogDebug("usbDevice opended: " + usbDeviceName);

      switch (usbDeviceType) {
        case QCCHelper::USBDevice_ISP_ADAPTER: {
          QByteArray msg;
          msg.append((char) 0x01);
          msg.append((char) MsgCode_IspProgrammer_Firmware_SoftwareVersionRead);
          usbDeviceManager.writeData(QCCHelper::toIspMessage(msg));
          //          initialisedState = QCCHelper::STATE_INITIALISED;
          //          responseTimer.start(1, QCCHelper::EVENT_TIMEOUT_INITIALISE); // use timer, not signal because you are in the constructor
          break;
        }
        case QCCHelper::USBDevice_USART_ADAPTER: {
          // do nothing
          initialisedState = QCCHelper::STATE_INITIALISED;
          responseTimer.start(1, QCCHelper::EVENT_TIMEOUT_INITIALISE); // use timer, not signal because you are in the constructor
          break;
        }
        case QCCHelper::USBDevice_XBEE_ADAPTER: {
          send_XBeeCommand(QString("HV").toAscii());
          initialisedState = QCCHelper::STATE_XBEE_WAIT_FOR_HV;
          // Timer setzen und auf Antwort warten (slot), dann dort Channel abfragen
          // am besten initializingState setzen
          responseTimer.start(250, QCCHelper::EVENT_TIMEOUT_INITIALISE); // use timer, not signal because you are in the constructor
          break;
        }
        default: {
          QLogDebug("unknown usb-device detected.");
          responseTimer.start(1, QCCHelper::EVENT_TIMEOUT_INITIALISE); // use timer, not signal because you are in the constructor
          break;
        }
      }
    } else {
      responseTimer.start(1, QCCHelper::EVENT_TIMEOUT_INITIALISE); // use timer, not signal because you are in the constructor
    }
  }

  void QCommunicationChannel::sl_ResponseTimerExpired(uint eventId) {
    responseTimer.stop();
    QLogDebug(QCCHelper::eventDescriptionMap[(QCCHelper::timerEvent_t) eventId]);
    switch (eventId) {
      case QCCHelper::EVENT_TIMEOUT_INITIALISE:
        emit sig_cc_initalised(this);
        break;
      case QCCHelper::EVENT_TIMEOUT_NODEDISCOVER:
        emit sig_cc_dns_name_resolved(this);
        break;
      case QCCHelper::EVENT_TIMEOUT_GENERAL:
      default:
        break;
    }
  }

  void QCommunicationChannel::sleep(ulong msecs) {
    QTime dieTime = QTime::currentTime().addMSecs(msecs);
    while (QTime::currentTime() < dieTime)
      QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
  }

  void QCommunicationChannel::close() {
    usbDeviceManager.closeDevice();
    disconnect(&usbDeviceManager, SIGNAL(sig_newData(QByteArray)));
  }

  QString QCommunicationChannel::getCCTypeString() {
    QString deviceTypeString;
    deviceTypeString.append("[" + usbDeviceManager.getDeviceName() + "]");
    switch (usbDeviceType) {
      case QCCHelper::USBDevice_ISP_ADAPTER: {
        break;
      }
      case QCCHelper::USBDevice_USART_ADAPTER: {
        break;
      }
      case QCCHelper::USBDevice_XBEE_ADAPTER: {
        // cut last 4 digits to get the same device name
        // this is because if channel and panid are identical for two QCC, they
        // serve the same (physical) network which causes problems and then only
        // one QCC of them is used
        deviceTypeString.truncate(16);
        deviceTypeString.append("]");
        switch (xbee.type) {
          case XBeeType_Serie1:
            deviceTypeString.append("[XBEE:");
            deviceTypeString.append("CH(" + QCCHelper::toHexNumberString(xbee.rf_channel, 2) + "):");
            deviceTypeString.append("PAN(" + QCCHelper::toHexNumberString(xbee.pan_identifier, 4) + ")]");
            break;
          case XBeeType_Serie2:
            deviceTypeString.append("[ZBEE:");
            deviceTypeString.append("CH(" + QCCHelper::toHexNumberString(xbee.rf_channel, 2) + "):");
            deviceTypeString.append("PAN(" + QCCHelper::toHexNumberString(xbee.pan_identifier, 4) + ")]");
            break;
        }
        break;
      }
      case QCCHelper::USBDevice_None: {
        deviceTypeString.append("unknown USBDeviceType");
        break;
      }
    }//switch(usbDeviceType)
    return deviceTypeString;
  }

  void QCommunicationChannel::sl_XBee_ReadDnsNames_Delayed() {
    foreach(struct QCCHelper::XBeeRemoteNode_t* xbeeRemoteNode, xbeeRemoteNodeList)
      {
        QLogDebug(usbDeviceManager.getDeviceName() + ":sl_XBee_ReadDnsNames_Delayed");

        if (usbDeviceType == QCCHelper::USBDevice_XBEE_ADAPTER) {
          switch (xbee.type) {
            case QCCHelper::XBeeType_SERIE_1: {
              QByteArray msg;
              msg.append(QCCHelper::MsgGroup_ECB_ROBOT_FIRMWARE);
              msg.append(QCCHelper::MsgCode_ECB_DNSName_Read);
              usbDeviceManager.writeData(QCCHelper::toXBeeS1Message(msg, xbeeRemoteNode->address16));
              break;
            }
            case QCCHelper::XBeeType_SERIE_2: {
              QByteArray msg;
              msg.append(QCCHelper::MsgGroup_ECB_ROBOT_FIRMWARE);
              msg.append(QCCHelper::MsgCode_ECB_DNSName_Read);
              usbDeviceManager.writeData(QCCHelper::toXBeeS2Message(msg, xbeeRemoteNode->address16,
                  xbeeRemoteNode->address64));
              break;
            }
            default:
              break;
          }// switch(XBeeType)
        }
      }
  }

  void QCommunicationChannel::clearXbeeRemoteList() {
    foreach(struct QCCHelper::XBeeRemoteNode_t* xbeeRemoteNode, xbeeRemoteNodeList)
      {
        free(xbeeRemoteNode);
        xbeeRemoteNodeList.removeOne(xbeeRemoteNode);
      }
  }

  void QCommunicationChannel::send_XBeeCommand(QByteArray command) {
    usbDeviceManager.writeData(QCCHelper::toXBeeATCommand(command));
    responseTimer.start(QCCHelper::EVENT_TIMEOUT_XBEE_COMMAND);
  }

  void QCommunicationChannel::send_XBeeRemoteCommand(QByteArray command, struct QCCHelper::XBeeRemoteNode_t* node) {
    usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand(command, node->address16, node->address64));
    responseTimer.start(QCCHelper::EVENT_TIMEOUT_XBEE_REMOTE_COMMAND);
  }

  void QCommunicationChannel::send_ECB_Reset(struct QCCHelper::XBeeRemoteNode_t* node) {
    switch (usbDeviceType) {
      case QCCHelper::USBDevice_USART_ADAPTER: {
        QByteArray msg;
        msg.append((char) MsgGroup_Identifier_ECBRobotFirmware);
        msg.append((char) MsgCode_ECB_CommandResetCableMode);
        usbDeviceManager.writeData(QCCHelper::toUsartMessage(msg));
        break;
      }
      case QCCHelper::USBDevice_XBEE_ADAPTER: {
        switch (xbee.type) {
          case QCCHelper::XBeeType_SERIE_1:
          case QCCHelper::XBeeType_SERIE_2: {
            if (node->address16 == 0xFFFE && node->address64 == 0x000000000000FFFF) {
              QLogWarning("Please select a node first!");
              return;
            }

            QLogDebug(usbDeviceManager.getDeviceName() + ":" + node->Identifier + ":reset");

            QByteArray commandToSet_D0_high;
            commandToSet_D0_high.append((char) 'D');
            commandToSet_D0_high.append((char) '0');
            commandToSet_D0_high.append((char) 0x05);
            usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand(commandToSet_D0_high, node->address16,
                node->address64));
            sleep(100);
            QByteArray commandToSet_D0_low;
            commandToSet_D0_low.append((char) 'D');
            commandToSet_D0_low.append((char) '0');
            commandToSet_D0_low.append((char) 0x00);
            usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand(commandToSet_D0_low, node->address16,
                node->address64));
            sleep(100);
            break;
          }
        }//end switch
      }
      case QCCHelper::USBDevice_ISP_ADAPTER:
      case QCCHelper::USBDevice_None:
      default:
        break;
    }
  }

  void QCommunicationChannel::sl_switchResetOff() {
    QByteArray commandToSet_D0_low;
    commandToSet_D0_low.append((char) 'D');
    commandToSet_D0_low.append((char) '0');
    commandToSet_D0_low.append((char) 0x00);
    usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand(commandToSet_D0_low, resetted_xbee->address16,
        resetted_xbee->address64));
  }

  void QCommunicationChannel::printMessage(QString s, QByteArray buffer) {
    QString hex;
    QString line;

    for (int i = 0; i < buffer.length(); i++) {
      line.append(QCCHelper::toHexNumberString(buffer[i], 2));
      line.append(" ");
    }
    QLogDebug(s + ": " + line);
  }

  void QCommunicationChannel::sl_messageReceived(QByteArray received_msg) {
    printMessage(usbDeviceManager.getDeviceName() + "(" + QString::number(usbDeviceType) + ") :msgReceived ",
        received_msg);
    switch (usbDeviceType) {
      case QCCHelper::USBDevice_ISP_ADAPTER: {
        // Stoppe TransmitTimer
        responseTimer.stop();
        dispatch_isp(received_msg);
        break;
      }
      case QCCHelper::USBDevice_USART_ADAPTER: {
        // Stoppe TransmitTimer
        responseTimer.stop();
        dispatch_usart(received_msg);
        break;
      }
      case QCCHelper::USBDevice_XBEE_ADAPTER: {
        // Stoppe TransmitTimer
        responseTimer.stop();
        dispatch_xbee(received_msg);
        break;
      }
      case QCCHelper::USBDevice_None:
        break;

    }
  }

  void QCommunicationChannel::dispatch_isp(QByteArray received_msg) {
    // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
    // ----------------------
    // 0x00 - StartDelimiter
    // 0x01 - Length_HighByte
    // 0x02 - Length_LowByte
    // 0x03 - API-Identifier
    // ----------------------
    // 0x04 - MsgGroup
    // 0x05 - MsgCode
    // 0x06 - data/params...
    uint8 msgGroup = received_msg[4];
    uint8 msgCode = received_msg[5];
    switch (msgGroup) {
      case MsgGroup_Identifier_ISP_ADAPTER_BOOTLOADER: // Bootloader
        break;
      case MsgGroup_Identifier_ISP_ADAPTER_FIRMWARE: // Firmware
        switch (msgCode) {
          case MsgCode_ResponsePacket: {
            uint8 msgResponseCode = received_msg[6];
            switch (msgResponseCode) {
              case MsgCode_IspProgrammer_Firmware_SoftwareVersionRead: {
                // Nachrichten-Format:
                // ----------------------
                // 0x00 - StartDelimiter
                // 0x01 - Length_HighByte
                // 0x02 - Length_LowByte
                // 0x03 - API-Identifier
                // ----------------------
                // 0x04 - MsgGroup
                // 0x05 - MsgCode_ResponsePaket
                // 0x06 - MsgCode_IspProgrammer_Firmware_SoftwareVersionRead
                // 0x07 - some chars ....
                QLogDebug("USB-ISP-Adapter: SoftwareVersion = " + QString(received_msg.mid(7,
                    received_msg.length() - 6)));
                break;
              }
              default:
                break;
            }//switch(msgResponseCode)
            break;
          }
          default:
            printMessage("dispatch_isp", received_msg);
            break;
        }//switch(msgCode)
        break;
    }//switch(msgGroup)

  }
  void QCommunicationChannel::dispatch_usart(QByteArray received_msg) {
    // Eine Nachricht vom USB-ISP-Programmer wurde empfangen
    // ----------------------
    // 0x00 - StartDelimiter
    // 0x01 - Length_HighByte
    // 0x02 - Length_LowByte
    // 0x03 - API-Identifier
    // ----------------------
    // 0x04 - MsgGroup
    // 0x05 - MsgCode
    // 0x06 - data/params...
    if ((QByte) received_msg[0x04] == (QByte) MsgGroup_Identifier_ECBRobotFirmware) { // msgGroup-Identifier
      if ((QByte) received_msg[0x05] == (QByte) MsgCode_ResponsePacket) { // msgCode
        switch (received_msg[0x06]) { // msgResponseCode
          case MsgCode_ECB_Command_get_DNS_Name: {
            // MessageStructure
            // ----------------------
            // 0x04 - MsgGroup
            // 0x05 - MsgCode_ResponsePacket
            // 0x06 - MsgCode_ECB_Command_get_DNS_Name
            // 0x07 - responseState (0=ok, 1=error)
            // 0x08 - dns_name...
            if ((QByte) received_msg[0x07] == (QByte) 0) { // responseState
              uint16 dns_name_length = ((received_msg[1] & 0xFF) << 8) + received_msg[2] - 5;
              QString dnsName;
              dnsName.append(received_msg.mid(0x08, dns_name_length));
              QCCHelper::DNSDevice_t* dnsDeviceStruct = new QCCHelper::DNSDevice_t();
              dnsDeviceStruct->dns_name = dnsName;
              dnsDeviceStruct->channel = this;
              dnsDeviceList.append(dnsDeviceStruct);
              QString line;
              line.append("[" + usbDeviceManager.getDeviceName() + "]");
              line.append("[" + dnsName + "]");
              QLogDebug(line);

              emit sig_cc_dns_name_resolved(this);
            }
            break;
          }
          default: {
            // dispatch packet (find out corresponding DNS device) and send it to the MessageDispatchServer
            if (dnsDeviceList.isEmpty()) {
              QLogWarning("[QCC] Received message from DNS device, but not recognized while scanned for DNS devices.");
              printMessage(usbDeviceManager.getDeviceName() + ":dispatch_xbee ", received_msg);
              return;
            }
            struct _communicationMessage message;
            message.ecb_dns_name = dnsDeviceList[0]->dns_name;
            message.data = received_msg.mid(4);
            emit
            sig_messageReceived(message);
            break;
          }
        }//switch(msgResponseCode)
      } else {
        // dispatch packet (find out corresponding DNS device) and send it to the MessageDispatchServer
        if (dnsDeviceList.isEmpty()) {
          QLogWarning("[QCC] Received message from DNS device, but not recognized while scanned for DNS devices.");
          printMessage(usbDeviceManager.getDeviceName() + ":dispatch_xbee ", received_msg);
          return;
        }
        struct _communicationMessage message;
        message.ecb_dns_name = dnsDeviceList[0]->dns_name;
        message.data = received_msg.mid(4);
        emit
        sig_messageReceived(message);
      }
    }
  }


  void QCommunicationChannel::dispatch_xbee(QByteArray received_msg) {
    // Eine Nachricht vom USB-XBEE wurde empfangen
    // ----------------------
    // 0x00 - StartDelimiter
    // 0x01 - Length_HighByte
    // 0x02 - Length_LowByte
    // 0x03 - API-Identifier
    uint api_Identifier = received_msg[3] & 0xFF;

    QCCHelper::XBeeRemoteNode_t* xbeeRemoteNode = QCCHelper::getXBeeRemoteNode(received_msg, xbeeRemoteNodeList);

    //TODO:
    QLogDebug(usbDeviceManager.getDeviceName() + ":dispatch_xbee: " + QString::number(api_Identifier, 16));

    switch (api_Identifier) {
      case QCCHelper::API_XBee_AT_Command_Response: {
        dispatch_xbee_command(received_msg);
        break;
      }
      case QCCHelper::API_XBee_Remote_AT_Command_Response: {
        //TODO:
        if (debug)
          printMessage(usbDeviceManager.getDeviceName() + ":dispatch_xbee ", received_msg);
        break;
      }
      case QCCHelper::API_XBeeS1_Receive_Packet_16Bit:
      case QCCHelper::API_XBeeS2_ZigBee_Receive_Packet: {
        if (xbeeRemoteNode == NULL) {
          QLogWarning("[QCC] Received message from XBee node, but not recognized while discovering nodes.");
          printMessage(usbDeviceManager.getDeviceName() + ":dispatch_xbee ", received_msg);
          return;
        }
        // Eine Nachricht wurde über ein USB-XBee-Adapter::XBeeSerie1 empfangen
        // ----------------------
        // 0x00 - StartDelimiter
        // 0x01 - Length_HighByte
        // 0x02 - Length_LowByte
        // 0x03 - API-Identifier
        // ----------------------
        // 0x04 - SourceAddress16_1
        // 0x05 - SourceAddress16_0
        // 0x06 - RSSI
        // 0x07 - Options
        // 0x08 - MsgGroup
        // 0x09 - MsgCode
        // 0x0A - data/params...
        uint indexMsgGroup = 0x08;
        uint indexMsgCode = 0x09;
        uint indexData = 0x0A;
        if (api_Identifier == QCCHelper::API_XBeeS2_ZigBee_Receive_Packet) {
          // Eine Nachricht wurde über ein USB-XBee-Adapter::XBeeSerie2 empfangen
          // ----------------------
          // 0x00 - StartDelimiter
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
          indexMsgGroup = 0x10;
          indexMsgCode = 0x11;
          indexData = 0x12;
        }
        if ((QByte) received_msg[indexMsgGroup] == (QByte) MsgGroup_Identifier_ECBRobotFirmware) { // msgGroup-Identifier
          if ((QByte) received_msg[indexMsgCode] == (QByte) MsgCode_ResponsePacket) { // msgCode
            switch (received_msg[indexData]) { // msgResponseCode
              case MsgCode_ECB_Command_get_DNS_Name: {
                // MessageStructure
                // ----------------------
                // Xbee1 Xbee2 indexData+
                // 0x08  0x10  ----------  - MsgGroup
                // 0x09  0x11   0          - MsgCode_ResponsePacket
                // 0x0A  0x12   1          - MsgCode_ECB_Command_get_DNS_Name
                // 0x0B  0x13   2          - responseState (0=ok, 1=error)
                // 0x0C  0x14   3          - dns_name...
                if ((QByte) received_msg[indexData + 1] == (QByte) 0) {
                  uint16 dns_name_length = ((received_msg[1] & 0xFF) << 8) + received_msg[2] - 9;
                  QString dnsName;
                  dnsName.append(received_msg.mid(indexData + 2, dns_name_length));
                  xbeeRemoteNode->dns_name = dnsName;
                  xbeeRemoteNode->channel = this;
                  QCCHelper::printXbeeRemoteNodeInfo(usbDeviceManager.getDeviceName(), xbeeRemoteNode);
                }
                break;
              }
              default: {
                // dispatch packet (find out corresponding DNS device) and send it to the MessageDispatchServer
                struct _communicationMessage message ;
                message.ecb_dns_name = xbeeRemoteNode->dns_name;
                message.data = received_msg.mid(indexMsgGroup);
                emit sig_messageReceived(message);
              }
                break;
            } //switch(msgResponseCode)
          } else { // if message are initiated by the ECB to the client application
            // dispatch packet (find out corresponding DNS device) and send it to the MessageDispatchServer
            struct _communicationMessage message ;
            message.ecb_dns_name = xbeeRemoteNode->dns_name;
            message.data = received_msg.mid(indexMsgGroup);
            emit sig_messageReceived(message);
          }
        }
        break;
      }
    }
  }

  void QCommunicationChannel::dispatch_xbee_command(QByteArray received_command) {
    // Eine Nachricht wurde über ein USB-XBee-Adapter::XBeeSerie1 empfangen
    // ----------------------
    // 0x00 - StartDelimiter
    // 0x01 - Length_HighByte
    // 0x02 - Length_LowByte
    // 0x03 - API-Identifier
    // 0x04 - Frame-Identifier
    // 0x05 - AT-Command
    // 0x06 - AT-Command
    // 0x07 - Status (0=ok, 1=error, 2=invalid command, 3=invalid parameter)
    // 0x08 - data/params
    // ----------------------

    //TODO: emit sig_TextLog(usbDeviceManager.getDeviceName() + " dispatch_xbee_command");

    uint msgLength = (((uint) received_command[1] & 0xFF) << 8) + (received_command[2] & 0xFF);
    QString msgCommand;
    msgCommand.append((char) received_command[5]);
    msgCommand.append((char) received_command[6]);
    uint msgState = received_command[7];

    if (msgCommand.compare("HV") == 0 && msgState == 0) {
      xbee.hardwareVersion = (((uint16) received_command[8] & 0xFF) << 8) + (received_command[9] & 0xFF);
      switch (xbee.hardwareVersion) {
        case 0x180B:
        case 0x1842:
          xbee.type = XBeeType_Serie1;
          //TODO: emit sig_TextLog(usbDeviceManager.getDeviceName() + " XBeeType=XBeeSerie1");
          break;
        case 0x1942:
          xbee.type = XBeeType_Serie2;
          //TODO: emit sig_TextLog(usbDeviceManager.getDeviceName() + " XBeeType=XBeeSerie2");
          break;
        default:
          xbee.type = XBeeType_unknown;
          emit sig_cc_initalised(this);
          //TODO: emit sig_TextLog(usbDeviceManager.getDeviceName() + " XBeeType=unknown, HV=" + QCCHelper::toHexNumberString(xbee.type, 4));
      }//switch(xbee.hardwareVersion)
      // read the operation channel on witch RF connections are made between RF modules:
      send_XBeeCommand(QString("CH").toAscii());
      initialisedState = QCCHelper::STATE_XBEE_WAIT_FOR_CHANNEL;
      return;
    }

    if (msgCommand.compare("CH") == 0 && msgState == 0) {
      xbee.rf_channel = ((uint) received_command[8] & 0xFF);
      // read the PAN-Identifier of the xbee-module:
      send_XBeeCommand(QString("ID").toAscii());
      initialisedState = QCCHelper::STATE_XBEE_WAIT_FOR_PANID;
      return;
    }

    if (msgCommand.compare("ID") == 0 && msgState == 0) {
      xbee.pan_identifier = (((uint16) received_command[8] & 0xFF) << 8) + (received_command[9] & 0xFF);
      responseTimer.stop();
      initialisedState = QCCHelper::STATE_INITIALISED;
      emit
      sig_cc_initalised(this);
      return;
    }

    if (msgCommand.compare("ND") == 0 && msgState == 0) // NodeIdentifier-Response
    {
      // Das XBee sendet als antwort auch ein Paket mit Länge 5,
      // jedoch ohne nützliche Informationen, dieses beendet die Suche nach RemoteKnoten!
      if (msgLength <= 5) {
        sl_XBee_ReadDnsNames_Delayed();
        return;
      }

      struct QCCHelper::XBeeRemoteNode_t* xbeeRemoteNode = new QCCHelper::XBeeRemoteNode_t();
      xbeeRemoteNode->address16 = 0;
      xbeeRemoteNode->address16 += ((uint64) received_command[8] & 0xFF) << 1 * 8;
      xbeeRemoteNode->address16 += ((uint64) received_command[9] & 0xFF) << 0 * 8;
      xbeeRemoteNode->address64 = 0;
      xbeeRemoteNode->address64 += ((uint64) received_command[10] & 0xFF) << 7 * 8;
      xbeeRemoteNode->address64 += ((uint64) received_command[11] & 0xFF) << 6 * 8;
      xbeeRemoteNode->address64 += ((uint64) received_command[12] & 0xFF) << 5 * 8;
      xbeeRemoteNode->address64 += ((uint64) received_command[13] & 0xFF) << 4 * 8;
      xbeeRemoteNode->address64 += ((uint64) received_command[14] & 0xFF) << 3 * 8;
      xbeeRemoteNode->address64 += ((uint64) received_command[15] & 0xFF) << 2 * 8;
      xbeeRemoteNode->address64 += ((uint64) received_command[16] & 0xFF) << 1 * 8;
      xbeeRemoteNode->address64 += ((uint64) received_command[17] & 0xFF) << 0 * 8;
      xbeeRemoteNode->Identifier.clear();
      // Lese den NodeIdentifier-String aus.
      switch (xbee.type) {
        case XBeeType_Serie1: {
          for (uint i = 0; i < msgLength - 17; i++)
            xbeeRemoteNode->Identifier.append((char) received_command[19 + i]);
          break;
        }
        case XBeeType_Serie2: {
          for (uint i = 18; i < msgLength - 6; i++)
            xbeeRemoteNode->Identifier.append((char) received_command[i]);
          break;
        }
      }//end switch
      // ist dieser Remote-Knoten bereits in der Liste vorhanden?
      bool compare = false;
      foreach(struct QCCHelper::XBeeRemoteNode_t* xbeeNode, xbeeRemoteNodeList)
        {
          // the address64 is the serial number of the xbee-rf-adapter,
          // therefore the address64 is distinct
          // -> check the address64 only is sufficient
          if (xbeeNode->address64 == xbeeRemoteNode->address64) {
            compare = true;
            break;
          }
        }
      if (!compare) {
        xbeeRemoteNodeList.append(xbeeRemoteNode);
        dnsDeviceList.append(xbeeRemoteNode);
        QString line;
        line.append("[" + usbDeviceManager.getDeviceName() + "]");
        line.append("[" + QCCHelper::toHexNumberString(xbeeRemoteNode->address16, 4) + ":");
        line.append(QCCHelper::toHexNumberString(xbeeRemoteNode->address64, 16));
        line.append(":" + xbeeRemoteNode->Identifier + "]");
        QLogDebug(line);
      }

    }
  }

  void QCommunicationChannel::scanDNSDevices() {
    switch (usbDeviceType) {
      case QCCHelper::USBDevice_ISP_ADAPTER: {
        // no DNSDevice can be connected to this adapter at the moment
        break;
      }
      case QCCHelper::USBDevice_USART_ADAPTER: {
        QByteArray msg;
        msg.append((char) QCCHelper::MsgGroup_ECB_ROBOT_FIRMWARE);
        msg.append((char) QCCHelper::MsgCode_ECB_DNSName_Read);
        usbDeviceManager.writeData(QCCHelper::toUsartMessage(msg));
        break;
      }
      case QCCHelper::USBDevice_XBEE_ADAPTER: {
        send_XBeeCommand(QString("ND").toAscii());
        QTimer::singleShot(3000, this, SLOT(sl_DNSScanTimeout()));
        break;
      }
      default: {
        QLogDebug("unknown usb-device detected.");
        break;
      }
    }
  }

  bool QCommunicationChannel::isDeviceInitialised() {
    if (initialisedState == QCCHelper::STATE_INITIALISED)
      return true;
    return false;
  }

  QStringList QCommunicationChannel::getDNSDeviceStringList() {
    QStringList list;
    QLogDebug("[" + usbDeviceManager.getDeviceName() + "]: getDNSDeviceList(), number of devices: "
        + QString::number(dnsDeviceList.size()));
    foreach (QCCHelper::DNSDevice_t* dnsDevice, dnsDeviceList)
      {
        list.append(dnsDevice->dns_name);
        QLogDebug("deviceName = " + dnsDevice->dns_name);
      }
    return list;
  }

  QCCHelper::usbDeviceType_t QCommunicationChannel::getUSBDeviceType() {
    return usbDeviceType;
  }

  QString QCommunicationChannel::getUSBDeviceName() {
    return usbDeviceManager.getDeviceName();
  }

  void QCommunicationChannel::sl_DNSScanTimeout() {
    sl_ResponseTimerExpired(QCCHelper::EVENT_TIMEOUT_NODEDISCOVER);
  }

  uint QCommunicationChannel::getResponseTime() {
    return responseTimer.getTimeRan();
  }

  void QCommunicationChannel::sendMessage(struct _communicationMessage& msg) {
    switch (usbDeviceType) {
      case QCCHelper::USBDevice_USART_ADAPTER: {
        usbDeviceManager.writeData(QCCHelper::toUsartMessage(msg.data));
        responseTimer.start(QCCHelper::EVENT_TIMEOUT_XBEE_SEND_MESSAGE_CABLE);
        break;
      }
      case QCCHelper::USBDevice_XBEE_ADAPTER: {
        foreach(struct QCCHelper::XBeeRemoteNode_t* xbeeRemoteNode, xbeeRemoteNodeList)
          {
            if (xbeeRemoteNode->dns_name == msg.ecb_dns_name) {
              switch (xbee.type) {
                case QCCHelper::XBeeType_SERIE_1:
                  usbDeviceManager.writeData(QCCHelper::toXBeeS1Message(msg.data, xbeeRemoteNode->address16));
                  break;
                case QCCHelper::XBeeType_SERIE_2:
                  usbDeviceManager.writeData(QCCHelper::toXBeeS2Message(msg.data, xbeeRemoteNode->address16,
                      xbeeRemoteNode->address64));
                  break;
              }
              break;
            }
          }
        responseTimer.start(QCCHelper::EVENT_TIMEOUT_XBEE_SEND_MESSAGE_XBEE);
        break;
      } //end case XBEE_ADAPTER
      case QCCHelper::USBDevice_ISP_ADAPTER:
      case QCCHelper::USBDevice_None:
      default:
        QLogWarning("Message to send to DNS device, but wrong ADAPTER (USART, unknown) choosed.");
        break;
    }
  }

}// namespace lpzrobots
