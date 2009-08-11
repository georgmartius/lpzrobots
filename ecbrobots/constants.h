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
 *   Revision 1.1  2009-08-11 15:49:05  guettler
 *   Current development state:
 *   - Support of communication protocols for XBee Series 1, XBee Series 2 and cable mode
 *   - merged code base from ecb_robots and Wolgang Rabes communication handling;
 *     ECBCommunicator (almost) entirely rewritten: Use of Mediator (MediatorCollegues: ECB),
 *     Callbackble (BackCaller: SerialPortThread)
 *   - New CThread for easy dealing with threads (is using pthreads)
 *   - New TimerThreads for timed event handling
 *   - SerialPortThread now replaces the cserialthread
 *   - GlobalData, ECBCommunicator is now configurable
 *   - ECBAgent rewritten: new PlotOptionEngine support, adapted to new WiredController structure
 *   - ECBRobot is now Inspectables (uses new infoLines functionality)
 *   - ECB now supports dnsNames and new communication protocol via Mediator
 *   - Better describing command definitions
 *   - SphericalRobotECB: adapted to new ECB structure, to be tested
 *                                                *
 *                                                                         *
 ***************************************************************************/

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

  typedef unsigned char           uint8;
  typedef unsigned short          uint16;
  typedef unsigned int            uint32;
  typedef unsigned long long int uint64;

  /**
   * Atmega128: 128 words per page -> 256 bytes.
   */
  static const int ECB_AtMega128_PageSize_Byte  = 256;
  /**
   * Number of pages of programm-space left for the user-programm.
   * (The AtMega128 contians 512 pages of flash. 480 pages the user
   * can use, 32 are resered for the bootloader.)
   */
  static const int ECB_AtMega128_NumberOfPages  = 480;


  static const int Application_MessageGroupCode = 0x01;

  // MessageCodes
  //---------------------------------------------------------------------------------
  static const int MsgNoEvent                                                 = 0x00;
  //---------------------------------------------------------------------------------

  /** Transmit Modes */
  //---------------------------------------------------------------------------------
  enum transmitMode
  {
      Undefined, ///< unknown type (also initial state)
      Cable,  ///< over cable
      XBee,   ///< XBee series 1
      XBeeS2  ///< Xbee series 2
  };


  /** ECB_BootloaderApiCodes + XBee_ApiCodes */
  //---------------------------------------------------------------------------------
  /** Cable - API-Identifier */
  static const int API_Cable_TransmitReceive                                  = 0x20;
  //---------------------------------------------------------------------------------
  /** XBee-RF-Modul Serie 1 - API-Identifier */
  static const int API_XBee_Modem_Status                                      = 0x8A;
  static const int API_XBee_AT_Command                                        = 0x08;
  static const int API_XBee_AT_Command_Queue_Parameter_Value                  = 0x09;
  static const int API_XBee_AT_Command_Response                               = 0x88;
  static const int API_XBee_Transmit_Request_64Bit                            = 0x00;
  static const int API_XBee_Transmit_Request_16Bit                            = 0x01;
  static const int API_XBee_Transmit_Status                                   = 0x89;
  static const int API_XBee_Receive_Packet_64Bit                              = 0x80;
  static const int API_XBee_Receive_Packet_16Bit                              = 0x81;
  //---------------------------------------------------------------------------------
  /** XBee-RF-Modul Serie 2 - API-Identifier */
  static const int API_XBeeS2_Modem_Status                                    = 0x8A;
  static const int API_XBeeS2_AT_Command                                      = 0x08;
  static const int API_XBeeS2_AT_Command_Queue_Parameter_Value                = 0x09;
  static const int API_XBeeS2_AT_Command_Response                             = 0x88;
  static const int API_XBeeS2_Remote_Command_Request                          = 0x17;
  static const int API_XBeeS2_Remote_Command_Response                         = 0x97;
  static const int API_XBeeS2_ZigBee_Transmit_Request                         = 0x10;
  static const int API_XBeeS2_Explicit_Addressing_ZigBee_Command_Frame        = 0x11;
  static const int API_XBeeS2_ZigBee_Transmit_Status                          = 0x8B;
  static const int API_XBeeS2_ZigBee_Receive_Packet                           = 0x90;
  static const int API_XBeeS2_ZigBee_Explicit_Rx_Indicator                    = 0x91;
  static const int API_XBeeS2_ZigBee_IO_Data_Sample_Rx_Indicator              = 0x92;
  static const int API_XBeeS2_Sensor_Read_Indicator                           = 0x94;
  static const int API_XBeeS2_Node_Identification_Indicator                   = 0x95;
  //---------------------------------------------------------------------------------


#endif /* CONSTANTS_H_ */
