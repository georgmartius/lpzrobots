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
 *   Revision 1.1  2010-11-03 13:05:27  wrabe
 *   -new version 2.0 uses ftdi driver library (libftdi and libusb)
 *   -special string identifiers (device descriptor) of usb devices (FT232RL), hard coded!
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "panelLogView.h"

QPanelLogView::QPanelLogView()
{
	setPalette(QPalette(QColor(220, 230, 220)));
	setAutoFillBackground(true);

	QGridLayout *grid = new QGridLayout();
	this->setLayout(grid);


	// Das Textfeld
	textEdit_LogView = new QTextEdit();
	QPalette palette = textEdit_LogView->palette();
	palette.setColor(QPalette::Active, QPalette::Base, QColor(220, 230, 220));
	palette.setColor(QPalette::Inactive, QPalette::Base, QColor(200, 210, 200));
	textEdit_LogView->setAutoFillBackground(true);
	textEdit_LogView->setPalette(palette);
	textEdit_LogView->setTextInteractionFlags(Qt::TextSelectableByMouse);
	grid->addWidget(textEdit_LogView, 0, 0);

}

QPanelLogView::~QPanelLogView() {}

//------------------------------------------------------------------------------------------------------
void QPanelLogView::clearLogViewText(){
	textEdit_LogView->clear();}
void QPanelLogView::appendLogViewText(QString text){
	textEdit_LogView->append(text);}
//------------------------------------------------------------------------------------------------------









