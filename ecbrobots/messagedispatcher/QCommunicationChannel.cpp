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
 *
 *                                                                         *
 ***************************************************************************/

#include "QCommunicationChannel.h"

namespace lpzrobots {
  
  QCommunicationChannel::QCommunicationChannel() {

    responseTimer.setInterval(2000);
    connect(responseTimer, SIGNAL(timeout(uint)), this, SLOT(sl_ResponseTimerExpired(uint)));
  }
  
  QCommunicationChannel::~QCommunicationChannel() {
    // TODO Auto-generated destructor stub
  }

  void QCommunicationChannel::sl_ResponseTimerExpired(uint eventId){
    responseTimer.stop();

    switch(eventId)
    {
      default:
        break;
    }
  }

  void QCommunicationChannel::send_XBeeATND() {
    usbDeviceManager.writeData(QCCHelper::toXBeeATCommand((QByteArray) {(QByte) 'N', (QByte) 'D'}));
    responseTimer.start(transmitTimerLastAction_XBeeCommand);
  }
  void QCommunicationChannel::send_XBeeCommand(QByteArray command) {
    usbDeviceManager.writeData(QCCHelper::toXBeeATCommand(command));
    responseTimer.start(transmitTimerLastAction_XBeeCommand);
  }
  void QCommunicationChannel::send_XBeeRemoteCommand(QByteArray command) {
    usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand(command));
    responseTimer.start(transmitTimerLastAction_XBeeRemoteCommand);
  }
  void QCommunicationChannel::send_ECB_Reset()
  {
    switch (QCCHelper::getApplicationModeByName(usbDeviceManager.getDeviceName()))
    {
      case APPLICATION_MODE_USART_Adapter:
      {
        QByteArray msg;
        msg.append(MsgGroup_Identifier_ECBRobotFirmware);
        msg.append(Msg_ECB_AtMega128_ResetCableMode);
        usbDeviceManager.writeData(QCCHelper::toUsartMessage(msg));
        break;
      }
      case APPLICATION_MODE_XBEE_Adapter:
      {
        switch (USBDeviceXBeeType)
        {
          case XBeeType_Serie1:
          case XBeeType_Serie2:
          {
            if (ECB_XBeeAddress16 == 0xFFFE && ECB_XBeeAddress64 == 0x000000000000FFFF)
            {
              emit sig_stdOut("Bitte erst einen Knoten waehlen!");
              return;
            }

            usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand((QByteArray) {(QByte) 'D', (QByte) '0', 0x05}, ECB_XBeeAddress16, ECB_XBeeAddress64));
            sleep(100);
            usbDeviceManager.writeData(QCCHelper::toXBeeRemoteATCommand((QByteArray) {(QByte) 'D', (QByte) '0', 0x00}, ECB_XBeeAddress16, ECB_XBeeAddress64));
            break;
          }
        }//end switch
      }
    }
  }
  void QCommunicationChannel::dispatch_XbeeCommandResponse(QByteArray receiveBuffer) {
    QWord msgLength = (receiveBuffer[1] & 0xFF) << 8 | (receiveBuffer[2] & 0xFF);

    QString Command;

    Command.append((char) receiveBuffer[5]);
    Command.append((char) receiveBuffer[6]);

    if (Command.compare("HV") == 0)
    {
      quint16 HardwareVersionNumber = 0;

      HardwareVersionNumber += ((quint16) receiveBuffer[8] & 0xFF) << 1 * 8;
      HardwareVersionNumber += ((quint16) receiveBuffer[9] & 0xFF) << 0 * 8;

      switch (HardwareVersionNumber)
      {
        case 0x180B:
        case 0x1842:
          USBDeviceXBeeType = XBeeType_Serie1;
          //TODO: panelSetting->setUSBDeviceXBeeType("XBeeSerie1");
          break;
        case 0x1942:
          USBDeviceXBeeType = XBeeType_Serie2;
          //TODO: panelSetting->setUSBDeviceXBeeType("XBeeSerie2");
          break;
      }
      return;
    }

    if (Command.compare("ND") == 0) // NodeIdentifier-Response
    {
      if ((int) receiveBuffer[7] != 0) // Status OK?
      {
        emit sig_stdOut("Error occured while identifing nodes.");

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
      switch (USBDeviceXBeeType)
      {
        case XBeeType_Serie1:
        {
          for (int i = 0; i < msgLength - 17; i++)
            nodeId.append((char) receiveBuffer[19 + i]);
          nodeId.append("'");

          //TODO: panelSetting->setXBeeRemoteNodeIdentifier(nodeId);
          break;
        }
        case XBeeType_Serie2:
        {
          for (int i = 18; i < msgLength - 6; i++)
            nodeId.append((char) receiveBuffer[i]);
          nodeId.append("'");

          //TODO: panelSetting->setXBeeRemoteNodeIdentifier(nodeId);
          break;
        }
      }//end switch

    }
  }

}
