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
 *   Revision 1.3  2010-11-18 16:58:18  wrabe
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

namespace lpzrobots {
  
  int QCCHelper::getDefaultBaudrateByName(QString actDeviceName) {
    if (actDeviceName.startsWith("USB-XBEE-Adapter", Qt::CaseInsensitive))
      return 57600;
    if (actDeviceName.startsWith("USB-USART-Adapter", Qt::CaseInsensitive))
      return 460800;
    if (actDeviceName.startsWith("USB-ISP-Adapter", Qt::CaseInsensitive))
      return 921600;
    return 0;
  }

  QByteArray QCCHelper::toIspMessage(QByteArray msgToFormat) {
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
  QByteArray QCCHelper::toUsartMessage(QByteArray msgToFormat) {
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

  QByteArray QCCHelper::toXBeeS1Message(QByteArray msgToFormat, uint16 address16) {
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

  QByteArray QCCHelper::toXBeeS2Message(QByteArray msgToFormat, uint16 address16, uint64 address64) {
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

  QByteArray QCCHelper::toXBeeATCommand(QByteArray msgToFormat) {
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

  QByteArray QCCHelper::toXBeeRemoteATCommand(QByteArray msgToFormat, uint16 address16, uint64 address64) {
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

  QCCHelper::usbDeviceType_t QCCHelper::getUsbDeviceTypeByName(QString usbDeviceName) {
    if (usbDeviceName.startsWith("USB-ISP-Adapter"))
      return USBDevice_ISP_ADAPTER;
    if (usbDeviceName.startsWith("USB-USART-Adapter"))
      return USBDevice_USART_ADAPTER;
    if (usbDeviceName.startsWith("USB-XBEE-Adapter"))
      return USBDevice_XBEE_ADAPTER;
    return USBDevice_None;
  }

  int QCCHelper::getApplicationModeByName(QString usbDeviceName) {
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
  }


} // namespace lpzrobots
