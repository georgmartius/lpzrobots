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
 *                                            *
 *                                                                         *
 ***************************************************************************/

#ifndef PGMVIEWER_H_
#define PGMVIEWER_H_
#include <QtGui>
#include <QWidget>
#include "types.h"

namespace lpzrobots {

  class QPanelHexViewer : public QWidget {
  Q_OBJECT
  public:

    /**
     * Creates a new HexViewer-Widget. The hexViewer displays binary data in a
     * human-readable manner. All the bytes are presented as an hexadecimal
     * number as well as and ascii-character.
     */
    QPanelHexViewer(QWidget *parent = 0);

    /**
     * Returns the binary data.
     * @return a QByteArray containing the binary data.
     */
    QByteArray getBinary();
    /**
     * A page is a pice of the entire content of the buffer. The buffer will
     * graduate into pices of the size of second param and returns the desired
     * pice defined by first param. If the page does not exists the page will
     * be empty. An empty page will contain all the values 0xFF.
     * @param iPageNumber specifies the desired part to be returned.
     * @param iPageSize_in_Bytes defines the size of the pices.
     * @return a QByteArray containing the desired pice of the binary data.
     */
    QByteArray getPage(ushort iPageNumber, ushort iPageSize_in_Bytes);
    /**
     * A page is a pice of the entire content of the buffer. The buffer will
     * graduate into pices of the size of second param. If the start of the desired
     * pice defined by first param is located in the entire buffer the page exists.
     * @return true if the specified page exists.
     */
    bool hasPage(ushort iPageNumber, ushort iPageSize_in_Bytes);
    /**
     * A page is a pice of the entire content of the buffer. The buffer will
     * graduate into pices of the size of second param. If the desired page exists
     * and there content consits only of the value 0xFF then the page is empty.
     * @return true if the specified page is empty.
     */
    bool isPageEmpty(ushort iPageNumber, ushort iPageSize);

  public slots:
    /**
     * Sets the binary data to be displayed.
     * @param binary a QByteArray of the binary.
     */
    void setBinary(QByteArray binary);
    /**
     * Updates the member verticalScrollPosition containing the vertical scroll position.
     * After there repaints the widgets content immediately.
     * @param scrollPos the new scroll position.
     */
    void setScrollPosition(int scrollPos);

  protected:
    /**
     * This event handler was reimplemented to receive paint events which are passed in the event parameter.
     * This widget displays the containing data in lines of hexadecimal values. Each line consists of 16
     * values and is devided in tho parts. On the left part the binaries are displayed in groups of 16 values
     * as hexadecimal numbers and the second part displays the ascii-values. According to the
     * verticalScrollposition and the available size of the window a part of the binary data will painted out.
     * @param event holds the paint event parameter.
     */
    void paintEvent(QPaintEvent *event);

  private:
    QScrollBar *scrollBar; ///< Holds the control.
    QByteArray pgmBuffer; ///< Holds the data to be displayed.
    int verticalScrollPosition; ///< Holds the vertical scroll position.

  };


}//namespace lpzrobots
#endif /* PGMVIEWER_H_ */
