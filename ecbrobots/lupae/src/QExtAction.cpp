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
 *   Revision 1.1  2010-11-09 17:56:55  wrabe
 *   - change of the communication protocoll between lupae and usb-isp-adapter
 *   - therefore recoding the dedicated methods
 *   - reduction of the overloded send_Message methods to one method only
 *   - insertion of QExtActions to join all events of menu-buttons as well of quickstart-buttons
 *   - adding two new functions to read out and write into the eeprom-space of the atmega128 at an ecb
 *   - change of the fontSize in the hexViewer, change of the total-width of the window
 *                                                *
 *                                                                         *
 ***************************************************************************/
#include "QExtAction.h"

namespace lpzrobots {

  QExtAction::QExtAction(QObject* parent) :
    QAction(parent), actionId(0) {
  }

  QExtAction::QExtAction(int actionId, const QString &text, QObject* parent) :
    QAction(text, parent), actionId(actionId) {
    connect(this, SIGNAL(triggered()), this, SLOT(sl_triggered()));
  }

  QExtAction::QExtAction(int actionId, const QIcon &icon, const QString &text, QObject* parent) :
    QAction(icon, text, parent), actionId(actionId) {
    connect(this, SIGNAL(triggered()), this, SLOT(sl_triggered()));
  }

  QExtAction::~QExtAction() {
  }

  void QExtAction::setActionId(int actionId) {
    this->actionId = actionId;
  }

  void QExtAction::sl_triggered() {
    emit triggered(actionId);
  }

} // namespace lpzrobots
