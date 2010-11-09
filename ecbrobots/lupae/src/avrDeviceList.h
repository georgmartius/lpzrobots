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
 *                                             *
 *                                                                         *
 ***************************************************************************/

#ifndef QAVR_DEVICELIST_H_
#define QAVR_DEVICELIST_H_

#include "qxml.h"
#include "qhash.h"
#include "types.h"

namespace lpzrobots {
  /**
   * This structure holds the names an the (default)values of the
   * fusebits a device have.
   * A fusbyte contains 8 fusebits.
   */
  struct AVRFUSEBYTE {
    QString *Names[8]; /**< An Array that holds the names of the fusebits of the three fuseBytes. */
    QString *Description[8]; /**< An Array that holds a short description the fuse bits stands for. */
    QByte Value; /**< Here will stored the editable values of the current controller. */
    QByte DefaultValue; /**< Holds the default (shipped) fuse byte value. */
    QByte EnabledBitMask; /**< This holds a values that specify which fuse bits are available/editable. */
  };

  /**
   * This structure holds the paramter of a device (avr-mico-controller)
   * The paramters will read out of a xml-file.
   * By pointing to a further structure-object a single linked list will create holding
   * all the devices.
   */
  struct AVRDEVICE {
    QString *Name; /**< The name of the target i.e. 'AtMega128'. */
    uint Signature; /**< The signature of that divice (controller). */
    ushort PageSizeBytes; /**< This specifies number of bytes a flash-page holds. */
    ushort NumberPages; /**< This holds the number of pages the device contains. */
    ushort NumberFuseBytes; /**< Holds the number of fuse bytes that the device assists. */
    AVRFUSEBYTE *FuseBytes[3]; /**< This holds a pointer to a structure of AVRFUSEBYTES. */
    AVRDEVICE *next; /**< A pointer to the next element to establish a linked list. */
  };

  /**
   * design pattern: model view controller: here the model
   *
   * This holds the parameter of available controllers. To read out the parameter
   * out of a xml-file the class inherits the QXmlDefaultHandler.
   *
   * stored parameter:
   *  - size of a page of a controllers programm space
   *  - count of all pages of programm space a controller owns.
   *  - the names and values of the fusebits
   *
   *  The specific blocks of device-parameters are stored in a single linked list.
   */
  class QAVR_DeviceList : QXmlDefaultHandler {

  public:
    /**
     * Constructs a handler for use with subclasses of QXmlReader. Afterthere
     * the constuctor will read the content of the parameter-file 'AVR_Device_List.xml'.
     * @param filename specifies the full path and name of the parameter-file.
     */
    QAVR_DeviceList(QString filename);
    /**
     * Destroys the handler.
     */
    virtual ~QAVR_DeviceList();

    /**
     * QXmlDefaultHandler:
     * The reader calls this function when it starts parsing the document. The reader calls this
     * function just once, after the call to setDocumentLocator(), and before any other functions
     * in this class or in the QXmlDTDHandler class are called.
     * @return If this function returns false the reader stops parsing and reports an error.
     */
    bool startDocument();
    /**
     * QXmlDefaultHandler:
     * The reader calls this function when it has parsed an end element tag with the qualified name qName,
     * the local name localName and the namespace URI namespaceURI.
     * @param namespaceURI specifies the namespace.
     * @param localName holds the local name.
     * @param qName holds the qualified name.
     * @return If this function returns false the reader stops parsing and reports an error.
     */
    bool endElement(const QString & namespaceURI, const QString & localName, const QString & qName);
    /**
     * QXmlDefaultHandler:
     * The reader calls this function when it has parsed a start element tag.
     * There is a corresponding endElement() call when the corresponding end element tag is read. The
     * startElement() and endElement() calls are always nested correctly. Empty element tags
     * cause a startElement() call to be immediately followed by an endElement() call.
     *
     * The attribute list provided only contains attributes with explicit values. The attribute list
     * contains attributes used for namespace declaration (i.e. attributes starting with xmlns) only
     * if the namespace-prefix property of the reader is true.
     *
     * @param namespaceURI   is the namespace URI, or an empty string if the element has no namespace
     *                       URI or if no namespace processing is done.
     * @param localName      localName is the local name (without prefix), or an
     *                       empty string if no namespace processing is done.
     * @param qName          qName is the qualified name (with prefix).
     * @param atts           atts are the attributes attached to the element. If there are no attributes,
     *                       atts is an empty attributes object.
     * @return If this function returns false the reader stops parsing and reports an error.
     */
    bool startElement(const QString & namespaceURI, const QString & localName, const QString & qName, const QXmlAttributes & atts);

    /**
     * Returns the loadstate.
     * @return true id the parameter-file was read and parsed correctly, otherwise returns false.
     */
    bool isLoaded();
    /**
     * Returns a device-block that corresponds o the given signature of the target device.
     * @param Signature a value that correspondes with a target device.
     * @return a object (structure) of a device if the given signature-bytes corresponds to a stored one, otherwise
     *         return a null-poiner.
     */
    AVRDEVICE* getDevice(uint Signature);

  private:
    bool hasFileLoad; ///< Holds the loadstate.
    QByte currentFuseByte; ///< Holds temporary the fusebyte while reading the parameters.
    AVRDEVICE *avrDevice; ///< Holds temporary the parameters while reading.
    AVRDEVICE *avrDeviceList_Head; ///< Holds the head of the linked list.
    AVRDEVICE *avrDeviceList_Tail; ///< holds the tail of the linked list, allows to fast appand the linked list.


  };
}//namespace lpzrobots
#endif /* QAVR_DEVICELIST_H_ */

