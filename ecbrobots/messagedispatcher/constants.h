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
 *                                                *
 *                                                                         *
 ***************************************************************************/

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define debug true


#define APPLICATION_MODE_None 0
#define APPLICATION_MODE_ISP_Adapter 1
#define APPLICATION_MODE_USART_Adapter 2
#define APPLICATION_MODE_XBEE_Adapter 3

#define Api_ISP_TransmitBootloader 0x20
#define Api_ISP_TransmitFirmware   0x20

//---------------------------------------------------------------
// Cable - API-Identifier
#define API_Cable_TransmitReceive                            0x20
//---------------------------------------------------------------
// XBee-RF-Modul Serie 1 - API-Identifier
#define API_XBee_Modem_Status                                0x8A
#define API_XBee_AT_Command                                  0x08
#define API_XBee_AT_Command_Queue_Parameter_Value            0x09
#define API_XBee_AT_Command_Response                         0x88
#define API_XBee_Remote_AT_Command_Request                   0x17
#define API_XBee_Remote_AT_Command_Response                  0x97
//---------------------------------------------------------------
#define API_XBeeS1_Transmit_Request_64Bit                    0x00
#define API_XBeeS1_Transmit_Request_16Bit                    0x01
#define API_XBeeS1_Transmit_Status                           0x89
#define API_XBeeS1_Receive_Packet_64Bit                      0x80
#define API_XBeeS1_Receive_Packet_16Bit                      0x81
//---------------------------------------------------------------
// XBee-RF-Modul Serie 2 - API-Identifier
#define API_XBeeS2_ZigBee_Transmit_Request                   0x10
#define API_XBeeS2_Explicit_Addressing_ZigBee_Command_Frame  0x11
#define API_XBeeS2_ZigBee_Transmit_Status                    0x8B
#define API_XBeeS2_ZigBee_Receive_Packet                     0x90
#define API_XBeeS2_ZigBee_Explicit_Rx_Indicator              0x91
#define API_XBeeS2_ZigBee_IO_Data_Sample_Rx_Indicator        0x92
#define API_XBeeS2_Sensor_Read_Indicator                     0x94
#define API_XBeeS2_Node_Identification_Indicator             0x95
//---------------------------------------------------------------

  //---------------------------------------------------------------------------------

  // MessageCodes
  //---------------------------------------------------------------------------------
  static const int MsgNoEvent                                                 = 0x00;
  //---------------------------------------------------------------------------------
  static const int Msg_Programmer_Bootloader_Start_Receive                    = 0x01;
  static const int Msg_Programmer_Bootloader_End_Receive                      = 0x02;
  static const int Msg_Programmer_Bootloader_FlashPageRead_Send               = 0x03;
  static const int Msg_Programmer_Bootloader_FlashPageReadResponse_Receive    = 0x04;
  static const int Msg_Programmer_Bootloader_FlashPageWrite_Send              = 0x05;
  static const int Msg_Programmer_Bootloader_FlashPageWriteResponse_Receive   = 0x06;
  static const int Msg_Programmer_Bootloader_BootloaderEnd_Send               = 0x07;
  //---------------------------------------------------------------------------------
  static const int Msg_Programmer_SignOfLive_Send                             = 0x20;
  static const int Msg_Programmer_SignOfLiveResponse_Receive                  = 0x21;
  //---------------------------------------------------------------------------------
  static const int Msg_Programmer_BeginTransaction_Send                       = 0x22;
  static const int Msg_Programmer_BeginTransactionResponse_Receive            = 0x23;
  static const int Msg_Programmer_EndTransaction_Send                         = 0x24;
  static const int Msg_Programmer_EndTransactionResponse_Receive              = 0x25;
  //---------------------------------------------------------------------------------
  static const int Msg_Programmer_TargetSignatureBytesRead_Send               = 0x26;
  static const int Msg_Programmer_TargetSignatureBytesReadResponse_Receive    = 0x27;
  //---------------------------------------------------------------------------------
  static const int Msg_Programmer_TargetCalibrationBytesRead_Send             = 0x28;
  static const int Msg_Programmer_TargetCalibrationBytesReadResponse_Receive  = 0x29;
  //---------------------------------------------------------------------------------
  static const int Msg_Programmer_TargetFuseBitsRead_Send                     = 0x2A;
  static const int Msg_Programmer_TargetFuseBitsReadResponse_Receive          = 0x2B;
  static const int Msg_Programmer_TargetFuseBitsWrite_Send                    = 0x2C;
  static const int Msg_Programmer_TargetFuseBitsWriteResponse_Receive         = 0x2D;
  //---------------------------------------------------------------------------------
  static const int Msg_Programmer_TargetFlashPageRead_Send                    = 0x2E;
  static const int Msg_Programmer_TargetFlashPageReadResponse_Receive         = 0x2F;
  static const int Msg_Programmer_TargetFlashPageWrite_Send                   = 0x30;
  static const int Msg_Programmer_TargetFlashPageWriteResponse_Receive        = 0x31;
  //---------------------------------------------------------------------------------
  static const int Msg_Programmer_TargetChipErase_Send                        = 0x32;
  static const int Msg_Programmer_TargetChipEraseResponse_Receive             = 0x33;
  //---------------------------------------------------------------------------------
  static const int Msg_Programmer_Target_OSCCAL_Increment_Send                = 0x40;
  static const int Msg_Programmer_Target_OSCCAL_IncrementResponse_Receive     = 0x41;
  static const int Msg_Programmer_Target_OSCCAL_Decrement_Send                = 0x42;
  static const int Msg_Programmer_Target_OSCCAL_DecrementResponse_Receive     = 0x43;
  //---------------------------------------------------------------------------------
  // MessageCodes
  //---------------------------------------------------------------------------------
  static const int Msg_ECB_Bootloader_Bootloader_Start_Received               = 0x01;
  static const int Msg_ECB_Bootloader_Bootloader_End_Received                 = 0x02;
  static const int Msg_ECB_Bootloader_BootloaderEnd_Send                      = 0x03;
  //---------------------------------------------------------------------------------
  static const int Msg_ECB_Bootloader_FlashPageRead_Send                      = 0x10;
  static const int Msg_ECB_Bootloader_FlashPageRead_ResponseReceived          = 0x11;
  static const int Msg_ECB_Bootloader_FlashPageWrite_Send                     = 0x12;
  static const int Msg_ECB_Bootloader_FlashPageWrite_ResponseReceived         = 0x13;
  //---------------------------------------------------------------------------------
  static const int Msg_ECB_Bootloader_NodeIdentifierRead_Send                 = 0x16;
  static const int Msg_ECB_Bootloader_NodeIdentifierReadResponse_Received     = 0x17;
  static const int Msg_ECB_Bootloader_NodeIdentifierWrite_Send                = 0x18;
  static const int Msg_ECB_Bootloader_NodeIdentifierWriteResponse_Received    = 0x19;
  //---------------------------------------------------------------------------------
  static const unsigned char Msg_ECB_AtMega128_ResetCableMode                       = 0xFF;
  //---------------------------------------------------------------------------------


  // MessageResponseStates
  //--------------------------------------------------
  static const int State_ok                    = 0x00;
  static const int State_PageNumberError       = 0x01;
  static const int State_PageWriteError        = 0x02;


  // MessageBuffer_BytePositions
  //--------------------------------------------------
  static const int MsgBuffer_StartDelimiter    = 0x00;
  static const int MsgBuffer_LengthHigh        = 0x01;
  static const int MsgBuffer_LengthLow         = 0x02;
  static const int MsgBuffer_MsgCode           = 0x03;
  static const int MsgBuffer_MsgParam1         = 0x04;
  static const int MsgBuffer_MsgParam2         = 0x05;
  static const int MsgBuffer_MsgParam3         = 0x06;
  static const int MsgBuffer_MsgParam4         = 0x07;
  static const int MsgBuffer_MsgPageStart      = 0x08;
  //--------------------------------------------------


/** USBDeviceXBeeType (XBee-Serie) */
//---------------------------------------------------------------------------------
#define XBeeType_unknown  0
#define XBeeType_Serie1   1
#define XBeeType_Serie2   2

  /** ECB_Bootloader_ResponseStates */
  //---------------------------------------------------------------------------------
  static const int OperationSucceeded                                         = 0x00;
  static const int PageWrite_SegmentReceive_Confirmed_from_ECB                = 0x01;


  static const int WriteError                                                 = 0x10;
  static const int PageNumberError                                            = 0x11;
  static const int TransmissionSegmentError                                   = 0x12;
  static const int ECBME_NotAvailable                                         = 0x13;
  static const int ECBME_Not_Responding                                       = 0x14;
  static const int ECBME_UnknownFunktionParam                                 = 0x15;
  static const int EXT_SRAM_busy                                              = 0x16;
  static const int Unknown_Board_Number                                       = 0x17;
  //---------------------------------------------------------------------------------
  static const int Msg_XBEE_Command                                           = 0xFE;
  static const int Msg_Reset                                                  = 0xFF;
  //---------------------------------------------------------------------------------

  // Programmer flashSize parameter
  //---------------------------------------------------------------------------------
  static const int ProgrammerFlashPageSize      = 128;
  static const int ProgrammerNumberFlashPages   = 112;


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


#define transmitTimerLastAction_none              0x00
#define transmitTimerLastAction_initialize        0x10
#define transmitTimerLastAction_XBeeCommand       0x20
#define transmitTimerLastAction_XBeeRemoteCommand 0x21
#define transmitTimerLastAction_SendMessageRaw    0x30
#define transmitTimerLastAction_SendMessageCable  0x31
#define transmitTimerLastAction_SendMessageXBee   0x32
#define transmitTimerLastAction_SendMessageBL     0x33
#define transmitTimerLastAction_SendMessageISP    0x34




#endif /* CONSTANTS_H_ */
