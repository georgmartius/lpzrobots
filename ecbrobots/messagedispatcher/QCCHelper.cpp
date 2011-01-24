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
 *   Revision 1.6  2011-01-24 16:24:03  guettler
 *   -use new QLog feature
 *
 *   Revision 1.5  2010/11/26 12:22:36  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *   Revision 1.4  2010/11/23 11:08:06  guettler
 *   - some helper functions
 *   - bugfixes
 *   - better event handling
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

#include "QCCHelper.h"
#include "constants.h"
#include "QCommunicationChannel.h"
#include "QLog.h"
#include "QECBMessageDispatchServer.h"

#include <QHash>

namespace lpzrobots {
  
  int QCCHelper::getDefaultBaudrateByName(QString& actDeviceName) {
    if (actDeviceName.startsWith("USB-XBEE-Adapter", Qt::CaseInsensitive))
      return 57600;
    if (actDeviceName.startsWith("USB-USART-Adapter", Qt::CaseInsensitive))
      return 460800;
    if (actDeviceName.startsWith("USB-ISP-Adapter", Qt::CaseInsensitive))
      return 921600;
    return 0;
  }

  QByteArray QCCHelper::toIspMessage(QByteArray& msgToFormat) {
    QExtByteArray msg;
    uint length = 1 + msgToFormat.length();
    msg.append(0x7E); // 0x00: Startsymbol
    msg.appendEscaped((QByte) (length >> 8)); // 0x01: Length MSB
    msg.appendEscaped((QByte) (length >> 0)); // 0x02: Length LSB
    msg.appendEscapedChecksum(API_Cable_TransmitReceive); // 0x03: API-ID
    for (int i = 0; i < msgToFormat.length(); i++) // 0x04: data ...
      msg.appendEscapedChecksum(msgToFormat[i]);
    msg.appendChecksum(); // packet checksum
    return (QByteArray) msg;
  }
  QByteArray QCCHelper::toUsartMessage(QByteArray& msgToFormat) {
    QExtByteArray msg;
    uint length = 1 + msgToFormat.length();
    msg.append(0x7E); // 0x00: Startsymbol
    msg.appendEscaped((QByte) (length >> 8)); // 0x01: Length MSB
    msg.appendEscaped((QByte) (length >> 0)); // 0x02: Length LSB
    msg.appendEscapedChecksum(API_Cable_TransmitReceive); // 0x03: API-ID
    for (int i = 0; i < msgToFormat.length(); i++) // 0x04: data ....
      msg.appendEscapedChecksum(msgToFormat[i]);
    msg.appendChecksum(); // packet checksum
    return (QByteArray) msg;
  }

  QByteArray QCCHelper::toXBeeS1Message(QByteArray& msgToFormat, uint16 address16) {
    QExtByteArray msg;
    uint length = 5 + msgToFormat.length();
    msg.append(0x7E); // 0x00: Startsymbol
    msg.appendEscaped((QByte) (length >> 8)); // 0x01: Length MSB
    msg.appendEscaped((QByte) (length >> 0)); // 0x02: Length LSB
    msg.appendEscapedChecksum(API_XBeeS1_Transmit_Request_16Bit); // 0x03: API-ID
    msg.appendEscapedChecksum(0x00); // 0x04: Frame-ID - immer 0 -> kein ResponsePaket vom XBee
    msg.appendEscapedChecksum((char) (address16 >> 1 * 8)); // 0x05: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address16 >> 0 * 8)); // 0x06: DestinationAddress LSB
    msg.appendEscapedChecksum(0x01); // 0x07: Options - immer 1  -> kein ResponsePaket vom XBee
    for (int i = 0; i < msgToFormat.length(); i++) // 0x08: data ....
      msg.appendEscapedChecksum(msgToFormat[i]);
    msg.appendChecksum(); // packet checksum
    return (QByteArray) msg;
  }

  QByteArray QCCHelper::toXBeeS2Message(QByteArray& msgToFormat, uint16 address16, uint64 address64) {
    QExtByteArray msg;
    uint length = 5 + msgToFormat.length();
    msg.append(0x7E); // 0x00: Startsymbol
    msg.appendEscaped((QByte) (length >> 8)); // 0x01: Length MSB
    msg.appendEscaped((QByte) (length >> 0)); // 0x02: Length LSB
    msg.appendEscapedChecksum(API_XBeeS2_ZigBee_Transmit_Request); // 0x03: API-ID
    msg.appendEscapedChecksum(0x00); // 0x04: Frame-ID - immer 0 -> kein ResponsePaket vom XBee
    msg.appendEscapedChecksum((char) (address64 >> 7 * 8)); // 0x05: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 6 * 8)); // 0x06: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 5 * 8)); // 0x07: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 4 * 8)); // 0x08: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 3 * 8)); // 0x09: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 2 * 8)); // 0x0A: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 1 * 8)); // 0x0B: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 0 * 8)); // 0x0C: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address16 >> 1 * 8)); // 0x0D: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address16 >> 0 * 8)); // 0x0E: DestinationAddress LSB
    msg.appendEscapedChecksum(0x00); // 0x0F: Broadcast-Range
    msg.appendEscapedChecksum(0x01); // 0x10: OptionsByte - immer 1  -> kein ResponsePaket vom XBee
    for (int i = 0; i < msgToFormat.length(); i++) // 0x11: data ....
      msg.appendEscapedChecksum(msgToFormat[i]);
    msg.appendChecksum(); // packet checksum
    return (QByteArray) msg;
  }

  QByteArray QCCHelper::toXBeeATCommand(QByteArray& msgToFormat) {
    QExtByteArray msg;
    uint length = 2 + msgToFormat.length();
    msg.append(0x7E); // 0x00: Startsymbol
    msg.appendEscaped((QByte) (length >> 8)); // 0x01: Length MSB
    msg.appendEscaped((QByte) (length >> 0)); // 0x02: Length LSB
    msg.appendEscapedChecksum(API_XBee_AT_Command); // 0x03: API-ID
    msg.appendEscapedChecksum('C'); // 0x04: Frame-ID
    for (int i = 0; i < msgToFormat.length(); i++) // 0x04: commandData 2..n bytes
      msg.appendEscapedChecksum(msgToFormat[i]);
    msg.appendChecksum(); // packet checksum
    return (QByteArray) msg;
  }

  QByteArray QCCHelper::toXBeeRemoteATCommand(QByteArray& msgToFormat, uint16 address16, uint64 address64) {
    QExtByteArray msg;
    uint length = 13 + msgToFormat.length();
    msg.append(0x7E); // 0x00: Startsymbol
    msg.appendEscaped((QByte) (length >> 8)); // 0x01: Length MSB
    msg.appendEscaped((QByte) (length >> 0)); // 0x02: Length LSB
    msg.appendEscapedChecksum(API_XBee_Remote_AT_Command_Request); // 0x03: API-ID
    msg.appendEscapedChecksum('C'); // 0x04: Frame-ID
    msg.appendEscapedChecksum((char) (address64 >> 7 * 8)); // 0x05: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 6 * 8)); // 0x06: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 5 * 8)); // 0x07: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 4 * 8)); // 0x08: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 3 * 8)); // 0x09: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 2 * 8)); // 0x0A: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 1 * 8)); // 0x0B: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address64 >> 0 * 8)); // 0x0C: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address16 >> 1 * 8)); // 0x0D: DestinationAddress MSB
    msg.appendEscapedChecksum((char) (address16 >> 0 * 8)); // 0x0E: DestinationAddress LSB
    msg.appendEscapedChecksum(0x02); // 0x0F: Options '0x02 - apply changes'
    for (int i = 0; i < msgToFormat.length(); i++) // 0x10: commandData 2..n bytes
      msg.appendEscapedChecksum(msgToFormat[i]);
    msg.appendChecksum(); // packet checksum
    return (QByteArray) msg;
  }

  QCCHelper::usbDeviceType_t QCCHelper::getUsbDeviceTypeByName(QString& usbDeviceName) {
    if (usbDeviceName.startsWith("USB-ISP-Adapter"))
      return USBDevice_ISP_ADAPTER;
    if (usbDeviceName.startsWith("USB-USART-Adapter"))
      return USBDevice_USART_ADAPTER;
    if (usbDeviceName.startsWith("USB-XBEE-Adapter"))
      return USBDevice_XBEE_ADAPTER;
    return USBDevice_None;
  }

  int QCCHelper::getApplicationModeByName(QString& usbDeviceName) {
    if (usbDeviceName.startsWith("USB-ISP-Adapter"))
      return APPLICATION_MODE_ISP_Adapter;
    if (usbDeviceName.startsWith("USB-USART-Adapter"))
      return APPLICATION_MODE_USART_Adapter;
    if (usbDeviceName.startsWith("USB-XBEE-Adapter"))
      return APPLICATION_MODE_XBEE_Adapter;
    return APPLICATION_MODE_None;
  }

  QString QCCHelper::toHexNumberString(uint64 value, uint numberDigits) {
    QString hex;
    for (int i = numberDigits; i > 0; i--) {
      hex.append(QString::number((value >> (i * 4 - 4)) & 0x0F, 16).toUpper());
    }
    return hex;
  }

  QString QCCHelper::getInitialisedStateString(typeInitialisedState initialisedState) {
    switch (initialisedState) {
      case STATE_INITIALISED:
        return "initialised";
      case STATE_NOT_INITIALISED:
        return "not initialised";
      case STATE_USBDEVICE_OPENED:
        return "USB-Device opened";
      case STATE_XBEE_WAIT_FOR_CHANNEL:
        return "wait for channel info";
      case STATE_XBEE_WAIT_FOR_HV:
        return "wait for HW-Version info";
      case STATE_XBEE_WAIT_FOR_PANID:
        return "wait for PANID info";
    }
    return QString("not defined");
  }

  QHash<QCCHelper::timerEvent_t, QString> QCCHelper::eventDescriptionMap;

  void QCCHelper::fillEventDescriptionMap() {
    if (eventDescriptionMap.isEmpty()) {
      eventDescriptionMap[EVENT_TIMEOUT_GENERAL] = "General timeout (not handled)!";
      eventDescriptionMap[EVENT_TIMEOUT_INITIALISE] = "Timeout initialising the QCC";
      eventDescriptionMap[EVENT_TIMEOUT_NODEDISCOVER] = "Timeout discovering nodes (XBee)";
      eventDescriptionMap[EVENT_TIMEOUT_XBEE_COMMAND] = "Timeout before getting response of XBee command";
      eventDescriptionMap[EVENT_TIMEOUT_XBEE_REMOTE_COMMAND] = "Timeout before getting response of XBee remote command";
      eventDescriptionMap[EVENT_TIMEOUT_XBEE_SEND_MESSAGE_RAW] = "Timeout before getting response of raw message";
      eventDescriptionMap[EVENT_TIMEOUT_XBEE_SEND_MESSAGE_CABLE] = "Timeout before getting response of cable message";
      eventDescriptionMap[EVENT_TIMEOUT_XBEE_SEND_MESSAGE_XBEE] = "Timeout before getting response of XBee message";
      eventDescriptionMap[EVENT_TIMEOUT_XBEE_SEND_MESSAGE_BL] = "Timeout before getting response of bootloader message";
      eventDescriptionMap[EVENT_TIMEOUT_XBEE_SEND_MESSAGE_ISP] = "Timeout before getting response of ISP message";
    } // else alreasy filled!
  }

  struct QCCHelper::XBeeRemoteNode_t* QCCHelper::getXBeeRemoteNode(QByteArray& msg,
      QList<XBeeRemoteNode_t*>& xbeeRemoteNodeList) {
    // ----------------------
    // 0x00 - StartDelimiter
    // 0x01 - Length_HighByte
    // 0x02 - Length_LowByte
    // 0x03 - API-Identifier
    foreach(struct XBeeRemoteNode_t* xbeeRemoteNode, xbeeRemoteNodeList)
      {
        switch ((uchar)msg[3]) {
          case API_XBeeS1_Receive_Packet_16Bit: {
            // Eine Nachricht wurde über ein USB-XBee-Adapter::XBeeSerie1 empfangen
            // 0x04 - SourceAddress16_1
            // 0x05 - SourceAddress16_0
            uint16 xbee_source_address16 = ((msg[4] & 0xFF) << 8) + msg[5];
            if (xbeeRemoteNode->address16 == xbee_source_address16)
              return xbeeRemoteNode;
            break;
          }
          case API_XBeeS2_ZigBee_Receive_Packet: {
            // Eine Nachricht wurde über ein USB-XBee-Adapter::XBeeSerie2 empfangen
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
            uint64 xbee_source_address64 = 0;
            xbee_source_address64 += ((uint64) (msg[0x05] & 0xFF) << 7 * 8);
            xbee_source_address64 += ((uint64) (msg[0x06] & 0xFF) << 6 * 8);
            xbee_source_address64 += ((uint64) (msg[0x07] & 0xFF) << 5 * 8);
            xbee_source_address64 += ((uint64) (msg[0x08] & 0xFF) << 4 * 8);
            xbee_source_address64 += ((uint64) (msg[0x09] & 0xFF) << 3 * 8);
            xbee_source_address64 += ((uint64) (msg[0x0A] & 0xFF) << 2 * 8);
            xbee_source_address64 += ((uint64) (msg[0x0B] & 0xFF) << 1 * 8);
            xbee_source_address64 += ((uint64) (msg[0x0C] & 0xFF) << 0 * 8);
            if (xbeeRemoteNode->address64 == xbee_source_address64)
              return xbeeRemoteNode;
            break;
          }
        }
      }
    // nothing found
    return NULL;
  }

  void QCCHelper::printXbeeRemoteNodeInfo(QString usbDeviceName, XBeeRemoteNode_t* xbeeNode) {
    QString line;
    line.append("[" + usbDeviceName + "]");
    line.append("[" + QCCHelper::toHexNumberString(xbeeNode->address16, 4) + ":");
    line.append(QCCHelper::toHexNumberString(xbeeNode->address64, 16));
    line.append(":" + xbeeNode->Identifier + "]");
    line.append("[" + xbeeNode->dns_name + "]");
    QLogDebug(line);
  }

} // namespace lpzrobots
