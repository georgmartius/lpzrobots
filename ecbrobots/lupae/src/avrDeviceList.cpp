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
 *   Revision 1.2  2010-11-09 17:56:55  wrabe
 *   - change of the communication protocoll between lupae and usb-isp-adapter
 *   - therefore recoding the dedicated methods
 *   - reduction of the overloded send_Message methods to one method only
 *   - insertion of QExtActions to join all events of menu-buttons as well of quickstart-buttons
 *   - adding two new functions to read out and write into the eeprom-space of the atmega128 at an ecb
 *   - change of the fontSize in the hexViewer, change of the total-width of the window
 *                                           *
 *                                                                         *
 ***************************************************************************/

#include "avrDeviceList.h"

namespace lpzrobots {
  QAVR_DeviceList::QAVR_DeviceList(QString filename) {
    avrDevice = 0;
    avrDeviceList_Head = 0;
    avrDeviceList_Tail = 0;

    QFile *file = new QFile(filename);
    QXmlInputSource *source = new QXmlInputSource(file);

    QXmlSimpleReader xmlReader;
    xmlReader.setContentHandler(this);
    xmlReader.setErrorHandler(this);

    hasFileLoad = xmlReader.parse(source);
  }

  QAVR_DeviceList::~QAVR_DeviceList() {
  }

  bool QAVR_DeviceList::isLoaded() {
    return hasFileLoad;
  }
  AVRDEVICE* QAVR_DeviceList::getDevice(uint Signature) {
    AVRDEVICE *tmp = avrDeviceList_Head;

    while (tmp != 0)
    {
      if (tmp->Signature == Signature)
        return tmp;
      tmp = tmp->next;
    }
    return 0;
  }

  //----------------------------------------------------------------------------------------------------
  // QXmlDefaultHandler
  //----------------------------------------------------------------------------------------------------
  bool QAVR_DeviceList::startDocument() {
    return true;
  }
  bool QAVR_DeviceList::endElement(const QString&, const QString&, const QString &name) {
    if (name == "AVR_Device")
    {
      if (avrDevice != 0)
      {
        if (avrDeviceList_Head == 0)
        {
          avrDeviceList_Head = avrDevice;
          avrDeviceList_Tail = avrDevice;
        } else
        {
          avrDeviceList_Tail->next = avrDevice;
          avrDeviceList_Tail = avrDevice;
        }
      }
    }
    return true;
  }

  bool QAVR_DeviceList::startElement(const QString&, const QString&, const QString &name, const QXmlAttributes &attrs) {
    QString s;
    bool ok;

    try
    {
      if (name == "AVR_Devices")
      {
      }
      if (name == "AVR_Device")
      {
        avrDevice = new AVRDEVICE;

        if (avrDevice == 0)
          return false;

        avrDevice->next = 0;
        avrDevice->Name = 0;
        avrDevice->NumberFuseBytes = 0;
        avrDevice->Signature = 0;
        avrDevice->PageSizeBytes = 0;
        avrDevice->NumberPages = 0;
        avrDevice->FuseBytes[0] = new AVRFUSEBYTE;
        avrDevice->FuseBytes[1] = new AVRFUSEBYTE;
        avrDevice->FuseBytes[2] = new AVRFUSEBYTE;

        for (int i = 0; i < 3; i++)
        {
          avrDevice->FuseBytes[i]->EnabledBitMask = 0;
          avrDevice->FuseBytes[i]->DefaultValue = 0;
          avrDevice->FuseBytes[i]->Value = 0;

          for (int j = 0; j < 8; j++)
          {
            avrDevice->FuseBytes[i]->Names[j] = 0;
            avrDevice->FuseBytes[i]->Description[j] = 0;
          }
        }
        currentFuseByte = 0;

        s = attrs.value("DeviceName");
        avrDevice->Name = new QString(s.toAscii());
        s = attrs.value("Signature");
        avrDevice->Signature = s.toUInt(&ok, 16);
        s = attrs.value("PageSize");
        avrDevice->PageSizeBytes = s.toUShort();
        s = attrs.value("NumberPages");
        avrDevice->NumberPages = s.toUShort();
        s = attrs.value("NumberFuseBytes");
        avrDevice->NumberFuseBytes = (QByte) s.toUShort() & 0x03;
      }
      if (name == "FuseBitsLow")
      {
        currentFuseByte = 0;
        s = attrs.value("defaultValue");
        avrDevice->FuseBytes[currentFuseByte]->DefaultValue = (QByte) s.toUInt(&ok, 16) & 0xFF;
        s = attrs.value("enabledBitMask");
        avrDevice->FuseBytes[currentFuseByte]->EnabledBitMask = (QByte) s.toUInt(&ok, 16) & 0xFF;
      }
      if (name == "FuseBitsHigh")
      {
        currentFuseByte = 1;
        s = attrs.value("defaultValue");
        avrDevice->FuseBytes[currentFuseByte]->DefaultValue = (QByte) s.toUInt(&ok, 16) & 0xFF;
        s = attrs.value("enabledBitMask");
        avrDevice->FuseBytes[currentFuseByte]->EnabledBitMask = (QByte) s.toUInt(&ok, 16) & 0xFF;
      }
      if (name == "FuseBitsExt")
      {
        currentFuseByte = 2;
        s = attrs.value("defaultValue");
        avrDevice->FuseBytes[currentFuseByte]->DefaultValue = (QByte) s.toUInt(&ok, 16) & 0xFF;
        s = attrs.value("enabledBitMask");
        avrDevice->FuseBytes[currentFuseByte]->EnabledBitMask = (QByte) s.toUInt(&ok, 16) & 0xFF;
      }
      if (name == "FuseBit")
      {
        s = attrs.value("number");
        int field = s.toInt() & 0x07;

        avrDevice->FuseBytes[currentFuseByte]->Names[field] = new QString(attrs.value("name"));
        avrDevice->FuseBytes[currentFuseByte]->Description[field] = new QString(attrs.value("description"));
      }
      return true;
    } catch (...)
    {
      return false;
    }
  }
}// namespace lpzrobots {

