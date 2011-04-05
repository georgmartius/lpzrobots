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
 *   Revision 1.6  2011-04-05 12:16:04  guettler
 *   - new tabWidget
 *   - all found DNS devices are shown in tabWidget with a QDNSDeviceWidget each
 *   - QDNSDeviceWidget shows DNS device name, USB adapter name and type,
 *     response time and incoming/outgoing status (if messages are currently sent
 *     or received)
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

#ifndef __QCCHELPER_H_
#define __QCCHELPER_H_

#include "types.h"
#include <QString>
#include <QByteArray>
#include "QExtByteArray.h"


namespace lpzrobots {

  class QCommunicationChannel;

  struct _communicationMessage;

  /**
   * static methods to reduce the code in class QCommunicationChannel
   */
  class QCCHelper {
    public:
      //      QCCHelper();
      //      virtual ~QCCHelper();

      enum typeInitialisedState {
        STATE_NOT_INITIALISED,
        STATE_USBDEVICE_OPENED,
        STATE_XBEE_WAIT_FOR_HV,
        STATE_XBEE_WAIT_FOR_CHANNEL,
        STATE_XBEE_WAIT_FOR_PANID,
        STATE_INITIALISED
      };

      enum usbDeviceType_t {
        USBDevice_None,
        USBDevice_ISP_ADAPTER,
        USBDevice_USART_ADAPTER,
        USBDevice_XBEE_ADAPTER
      };

      enum timerEvent_t {
         EVENT_TIMEOUT_GENERAL,
         EVENT_TIMEOUT_INITIALISE,
         EVENT_TIMEOUT_NODEDISCOVER,
         EVENT_TIMEOUT_XBEE_COMMAND,
         EVENT_TIMEOUT_XBEE_REMOTE_COMMAND,
         EVENT_TIMEOUT_XBEE_SEND_MESSAGE_RAW,
         EVENT_TIMEOUT_XBEE_SEND_MESSAGE_CABLE,
         EVENT_TIMEOUT_XBEE_SEND_MESSAGE_XBEE,
         EVENT_TIMEOUT_XBEE_SEND_MESSAGE_BL, ///< not used yet
         EVENT_TIMEOUT_XBEE_SEND_MESSAGE_ISP ///< not used yet
       };

      struct XBeeLocalNode_t {
          XBeeLocalNode_t(){
            type = 0;
            hardwareVersion = 0;
            rf_channel = 0x10000;  // Parameter Range: 0x0B - 0x1A (XBee) , 0x0C - 0x17 (XBee-PRO)
            pan_identifier = 0x10000;         // Parameter Range: 0 - 0xFFFF
          };
          uint16  hardwareVersion;
          uint    type;
          uint    rf_channel;
          uint    pan_identifier;
      };

      struct DNSDevice_t {
        public:
          QString dns_name;
          QCommunicationChannel* channel;
      };

      struct XBeeRemoteNode_t : public DNSDevice_t {
        public:
          XBeeRemoteNode_t(){
            address16 = 0xFFFE;
            address64 = 0x000000000000FFFF;
          };
          QString Identifier;
          uint16  address16;
          uint64  address64;
      };

      static usbDeviceType_t getUsbDeviceTypeByName(QString& usbDeviceName);
      static int getApplicationModeByName(QString& usbDeviceName);
      static int getDefaultBaudrateByName(QString& actDeviceName);
      static QByteArray toIspMessage(QByteArray& msgToFormat);
      static QByteArray toXBeeS1Message(QByteArray& msgToFormat, uint16 address16);
      static QByteArray toXBeeS2Message(QByteArray& msgToFormat, uint16 address16, uint64 address64);
      static QByteArray toUsartMessage(QByteArray& msgToFormat);
      static QByteArray toXBeeATCommand(QByteArray& msgToFormat);
      static QByteArray toXBeeRemoteATCommand(QByteArray& msgToFormat, uint16 address16, uint64 address64);

      /**
       * Extracts from the package the addresses and lookups which xbee remote node belongs to it.
       */
      static struct XBeeRemoteNode_t* getXBeeRemoteNode(QByteArray& msg, QList<XBeeRemoteNode_t*>& xbeeRemoteNodeList);


      static void printXbeeRemoteNodeInfo(QString usbDeviceName, XBeeRemoteNode_t* xbeeNode);
      static QString toHexNumberString(uint64 value, uint numberDigits);
      static QString getInitialisedStateString(typeInitialisedState initialisedState);

      static void fillEventDescriptionMap();
      static QHash<timerEvent_t, QString> eventDescriptionMap;

      enum ENUM_MSGGOUPS {
        MsgGroup_ECB_ROBOT_FIRMWARE = 0x01
      };

      enum ENUM_MSGCODES {
        MsgCode_ECB_DNSName_Read = 0x30
      };

      enum ENUM_XBEETYPE {
        XBeeType_UNKNOWN = 0, XBeeType_SERIE_1 = 1, XBeeType_SERIE_2 = 2
      };

      enum ENUM_API_IDENTIFIER {
        API_Cable_TransmitReceive = 0x20,
        //---------------------------------------------------------------
        // XBee-RF-Modul Serie 1 - API-Identifier
        API_XBee_Modem_Status = 0x8A,
        API_XBee_AT_Command = 0x08,
        API_XBee_AT_Command_Queue_Parameter_Value = 0x09,
        API_XBee_AT_Command_Response = 0x88,
        API_XBee_Remote_AT_Command_Request = 0x17,
        API_XBee_Remote_AT_Command_Response = 0x97,
        //---------------------------------------------------------------
        API_XBeeS1_Transmit_Request_64Bit = 0x00,
        API_XBeeS1_Transmit_Request_16Bit = 0x01,
        API_XBeeS1_Transmit_Status = 0x89,
        API_XBeeS1_Receive_Packet_64Bit = 0x80,
        API_XBeeS1_Receive_Packet_16Bit = 0x81,
        //---------------------------------------------------------------
        // XBee-RF-Modul Serie 2 - API-Identifier
        API_XBeeS2_ZigBee_Transmit_Request = 0x10,
        API_XBeeS2_Explicit_Addressing_ZigBee_Command_Frame = 0x11,
        API_XBeeS2_ZigBee_Transmit_Status = 0x8B,
        API_XBeeS2_ZigBee_Receive_Packet = 0x90,
        API_XBeeS2_ZigBee_Explicit_Rx_Indicator = 0x91,
        API_XBeeS2_ZigBee_IO_Data_Sample_Rx_Indicator = 0x92,
        API_XBeeS2_Sensor_Read_Indicator = 0x94,
        API_XBeeS2_Node_Identification_Indicator = 0x95
      //---------------------------------------------------------------
      };

      enum XBEE_PARAMS_t {
        XBEE_NODE_DISCOVER_TIME = 0 //2500
      };

      static const unsigned int RESPONSE_TIME_DEFAULT = 2000;

    private:
      static uchar checksum;
  };

}

#endif /* __QCCHELPER_H_ */
