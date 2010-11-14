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
 *   Revision 1.3  2010-11-14 12:14:06  wrabe
 *   - set debug to false
 *   - fix: show avrDeviceName after reading signature-bytes
 *
 *   Revision 1.2  2010/11/09 17:56:55  wrabe
 *   - change of the communication protocoll between lupae and usb-isp-adapter
 *   - therefore recoding the dedicated methods
 *   - reduction of the overloded send_Message methods to one method only
 *   - insertion of QExtActions to join all events of menu-buttons as well of quickstart-buttons
 *   - adding two new functions to read out and write into the eeprom-space of the atmega128 at an ecb
 *   - change of the fontSize in the hexViewer, change of the total-width of the window
 *                                                *
 *                                                                         *
 ***************************************************************************/

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define debug false

#define APPLICATION_MODE_None 0
#define APPLICATION_MODE_ISP_Adapter 1
#define APPLICATION_MODE_USART_Adapter 2
#define APPLICATION_MODE_XBEE_Adapter 3

/** ECB_BootloaderApiCodes + XBee_ApiCodes + ISP-Adapter_ApiCodes*/
//---------------------------------------------------------------
// Cable - API-Identifier
#define API_Cable_TransmitReceive                            0x20
//---------------------------------------------------------------
// XBee-RF-Modul Serie 1 - API-Identifier
#define API_XBee_Modem_Status                                0x8A
#define API_XBee_AT_Command                                  0x08
#define API_XBee_AT_Command_Queue_Parameter_Value            0x09
#define API_XBee_AT_Command_Response                         0x88
#define API_XBee_Transmit_Request_64Bit                      0x00
#define API_XBee_Transmit_Request_16Bit                      0x01
#define API_XBee_Transmit_Status                             0x89
#define API_XBee_Receive_Packet_64Bit                        0x80
#define API_XBee_Receive_Packet_16Bit                        0x81
//---------------------------------------------------------------
// XBee-RF-Modul Serie 2 - API-Identifier
#define API_XBeeS2_Modem_Status                              0x8A
#define API_XBeeS2_AT_Command                                0x08
#define API_XBeeS2_AT_Command_Queue_Parameter_Value          0x09
#define API_XBeeS2_AT_Command_Response                       0x88
#define API_XBeeS2_Remote_Command_Request                    0x17
#define API_XBeeS2_Remote_Command_Response                   0x97
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
#define Api_ISP_TransmitBootloader 0x20
#define Api_ISP_TransmitFirmware   0x20

#define MsgGroup_IspBootloader 0
#define MsgGroup_IspFirmware   1


// MessageCodes
//---------------------------------------------------------------------------------
#define MsgNoEvent                                                 0x00
//---------------------------------------------------------------------------------
// Message-Commands
#define MsgCode_NoEvent                                            0x00
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

// OperationResponseStates
//---------------------------------------------
#define state_OperationSucceeded           0x00
#define state_Confirm_PageSegmentReceive   0x01
#define state_FlashPageWriteError          0x02
#define state_FlashPageNumberError         0x03
#define state_TransmissionSegmentError     0x04
#define state_EEPromAddressNumberError     0x05
#define state_EEPromBlockWriteError        0x06
//---------------------------------------------


/** USBDeviceXBeeType (XBee-Serie) */
//---------------------------------------------------------------------------------
#define XBeeType_unknown  0
#define XBeeType_Serie1   1
#define XBeeType_Serie2   2


// Programmer flashSize parameter
//---------------------------------------------------------------------------------
#define ProgrammerFlashPageSize      128
#define ProgrammerNumberFlashPages   112

/**
 * Atmega128: 128 words per page -> 256 bytes.
 */
#define ECB_AtMega128_PageSize_Byte 256
/**
 * Number of pages of programm-space left for the user-programm.
 * (The AtMega128 contians 512 pages of flash. 480 pages the user
 * can use, 32 are resered for the bootloader.)
 */
#define ECB_AtMEGA128_NumberOfPages 480

#define ECB_ATMEGA128_EEPROM_BlockSize 64
#define ECB_ATMEGA128_EEPROM_TOTALSIZE 4096

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
