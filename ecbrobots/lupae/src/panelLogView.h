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
 *                                              *
 *                                                                         *
 ***************************************************************************/

#ifndef QPANELLOGVIEW_H_
#define QPANELLOGVIEW_H_

#include <QtGui>
#include <QWidget>
#include "types.h"

namespace lpzrobots {

  class QPanelLogView : public QWidget {
  Q_OBJECT

  public:
    /**
     * Creates a new widget that contains some comboBoxes and the describing labes to empower the
     * user to change the name of the used seriel port, to change the speed-setting of the serial
     * port named baudrate and to change the speed of the target device. (If a device is clocked with
     * a low frequency i.e. by a rc-oscillator the user can couse the programmer to set up wait periods
     * so the target system can react.)
     * Further it creates an QTextEdit to display internal states to the user in a user-readable manner.
     */
    QPanelLogView();
    /**
     * Destroys the Widget.
     */
    virtual ~QPanelLogView();

    /**
     * Removes the entire text from the textEdit_LogView.
     */
    void clearLogViewText();
    /**
     * Appends the current text of the textEdit_LogView with the given textpart.
     * (to write a new line to the logview)
     * @param text The text to be appended to the current text of the TextEdit-control
     */
    void appendLogViewText(QString text);

  private:
    QTextEdit *textEdit_LogView; ///< Holds the control.
    QLabel *label_TargetDevice; /**< Holds the control displaying the name of the
     current selected target device. */
  };

}//namespace lpzrobots
#endif /* QPANELLOGVIEW_H_ */
