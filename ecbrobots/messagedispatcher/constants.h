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
 *   Revision 1.4  2010-11-23 11:08:06  guettler
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
// MessageCodes
//---------------------------------------------------------------
#define MsgCode_ECB_CommandResetCableMode          0xFF
#define MsgGroup_Identifier_ECBRobotFirmware       0x01
#define MsgGroup_Identifier_ISP_ADAPTER_BOOTLOADER 0x00
#define MsgGroup_Identifier_ISP_ADAPTER_FIRMWARE   0x01
//---------------------------------------------------------------


  // MessageResponseStates
  //--------------------------------------------------
#define State_ok                    = 0x00;
#define State_PageNumberError       = 0x01;
#define State_PageWriteError        = 0x02;


  // MessageBuffer_BytePositions
  //--------------------------------------------------
#define MsgBuffer_StartDelimiter    = 0x00;
#define MsgBuffer_LengthHigh        = 0x01;
#define MsgBuffer_LengthLow         = 0x02;
#define MsgBuffer_MsgCode           = 0x03;
#define MsgBuffer_MsgParam1         = 0x04;
#define MsgBuffer_MsgParam2         = 0x05;
#define MsgBuffer_MsgParam3         = 0x06;
#define MsgBuffer_MsgParam4         = 0x07;
#define MsgBuffer_MsgPageStart      = 0x08;
  //--------------------------------------------------


/** USBDeviceXBeeType (XBee-Serie) */
//---------------------------------------------------------------------------------
#define XBeeType_unknown  0
#define XBeeType_Serie1   1
#define XBeeType_Serie2   2

  /** ECB_Bootloader_ResponseStates */
  //---------------------------------------------------------------------------------
#define OperationSucceeded                                         = 0x00;
#define PageWrite_SegmentReceive_Confirmed_from_ECB                = 0x01;


#define WriteError                                                 = 0x10;
#define PageNumberError                                            = 0x11;
#define TransmissionSegmentError                                   = 0x12;
#define ECBME_NotAvailable                                         = 0x13;
#define ECBME_Not_Responding                                       = 0x14;
#define ECBME_UnknownFunktionParam                                 = 0x15;
#define EXT_SRAM_busy                                              = 0x16;
#define Unknown_Board_Number                                       = 0x17;
  //---------------------------------------------------------------------------------
#define Msg_XBEE_Command                                           = 0xFE;
#define Msg_Reset                                                  = 0xFF;
  //---------------------------------------------------------------------------------

// MessageCodes
//---------------------------------------------------------------------------------
#define MsgCode_ResponsePacket                                     0x01
//---------------------------------------------------------------------------------
// MsgCodes - USB-ISP-Adapter::Bootloader
//---------------------------------------------------------------------------------
#define MsgCode_IspProgrammer_Bootloader_Start                     0x10
#define MsgCode_IspProgrammer_Bootloader_End                       0x11
#define MsgCode_IspProgrammer_Bootloader_FlashPageRead             0x12
#define MsgCode_IspProgrammer_Bootloader_FlashPageWrite            0x13
#define MsgCode_IspProgrammer_Bootloader_Activate                  0x14 // keine Verwendung!!
#define MsgCode_IspProgrammer_Bootloader_Deactivate                0x15
//---------------------------------------------------------------------------------
// MsgCodes - USB-ISP-Adapter::Firmware
//---------------------------------------------------------------------------------
#define MsgCode_IspProgrammer_Firmware_SoftwareVersionRead         0x20
//---------------------------------------------------------------------------------
#define MsgCode_IspProgrammer_TargetDevice_BeginTransaction        0x30
#define MsgCode_IspProgrammer_TargetDevice_EndTransaction          0x31
#define MsgCode_IspProgrammer_TargetDevice_SignatureBytesRead      0x32
#define MsgCode_IspProgrammer_TargetDevice_CalibrationBytesRead    0x33
#define MsgCode_IspProgrammer_TargetDevice_FuseBitsRead            0x34
#define MsgCode_IspProgrammer_TargetDevice_FuseBitsWrite           0x35
#define MsgCode_IspProgrammer_TargetDevice_FlashPageRead           0x36
#define MsgCode_IspProgrammer_TargetDevice_FlashPageWrite          0x37
#define MsgCode_IspProgrammer_TargetDevice_EEPROM_Read             0x38
#define MsgCode_IspProgrammer_TargetDevice_EEPROM_Write            0x39
#define MsgCode_IspProgrammer_TargetDevice_ChipErase               0x3A
//---------------------------------------------------------------------------------
// MsgCodes - ECB::Bootloader
//---------------------------------------------------------------------------------
#define MsgCode_ECB_SIGNAL_BootloaderStart                         0x10
#define MsgCode_ECB_Signal_BootloaderEnd                           0x11
#define MsgCode_ECB_Command_FLASH_PageRead                         0x12
#define MsgCode_ECB_Command_FLASH_PageWrite                        0x13
#define MsgCode_ECB_Command_EEPROM_BlockRead                       0x14
#define MsgCode_ECB_Command_EEPROM_BlockWrite                      0x15
#define MsgCode_ECB_Command_Bootloader_Deactivate                  0x16
//---------------------------------------------------------------------------------
#define MsgCode_ECB_AtMega128_ResetCableMode                       0xFF
//---------------------------------------------------------------------------------
#define MsgCode_ECB_Command_get_DNS_Name                           0x30


#endif /* CONSTANTS_H_ */
