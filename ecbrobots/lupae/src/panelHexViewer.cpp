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

#include <QPainter>
#include <QTextStream>
#include "panelHexViewer.h"

namespace lpzrobots {

  QPanelHexViewer::QPanelHexViewer(QWidget *parent) :
    QWidget(parent) {
    scrollBar = new QScrollBar(Qt::Vertical);
    connect(scrollBar, SIGNAL(valueChanged(int)), this, SLOT(setScrollPosition(int)));

    QWidget *w = new QWidget();
    QHBoxLayout *l = new QHBoxLayout();
    l->addWidget(w);
    l->addWidget(scrollBar);
    l->setMargin(0);
    setLayout(l);

    setPalette(QPalette(QColor(220, 220, 240)));
    setAutoFillBackground(true);
    verticalScrollPosition = 0;
    setMinimumWidth(580);
    setMinimumHeight(300);
  }
  void QPanelHexViewer::setBinary(QByteArray binary) {
    pgmBuffer = binary;
    scrollBar->setMaximum(pgmBuffer.length() / 16 - 1);
    update();
  }
  QByteArray QPanelHexViewer::getBinary() {
    return pgmBuffer;
  }

  QByteArray QPanelHexViewer::getPage(ushort iPageNumber, ushort iPageSize_in_Bytes) {
    QByteArray bPage;

    bPage.resize(iPageSize_in_Bytes);
    bPage.fill(0xFF);

    int startIndex = iPageNumber * iPageSize_in_Bytes;

    try
    {
      // Load the Page
      for (int i = 0; i < iPageSize_in_Bytes && startIndex + i < pgmBuffer.length(); i++)
      {
        bPage[i] = pgmBuffer[startIndex + i];
      }
    } catch (...)
    {
    }
    return bPage;
  }

  bool QPanelHexViewer::hasPage(ushort iPageNumber, ushort iPageSize_in_Bytes) {
    int startIndex = (int) iPageNumber * (int) iPageSize_in_Bytes;

    try
    {
      if (startIndex < pgmBuffer.length())
        return true;
    } catch (...)
    {
    }
    return false;
  }
  bool QPanelHexViewer::isPageEmpty(ushort iPageNumber, ushort iPageSize) {
    uint startIndex = iPageNumber * iPageSize;
    try
    {
      // A page is a pice of the entire content of the buffer. The buffer will be
      // graduate into pices of the size of second param. If all the bytes of the
      // pice specified by the first param compares 0xFF the page is empty,
      // true will be returned.
      for (uint i = 0; i < iPageSize; i++)
        if ((QByte) (pgmBuffer.at(startIndex + i)) != 0xFF)
        {
          return false;
        }
    } catch (...)
    {
    }
    return true;
  }

  void QPanelHexViewer::setScrollPosition(int scrollPos) {
    verticalScrollPosition = scrollPos;
    update();
  }
  void QPanelHexViewer::paintEvent(QPaintEvent *) {
    QPainter painter(this);
    painter.setFont(QFont("Courier", 10));

    QString sSpace = "";
    QString sPgm = "";
    QString sAscii = "";

    int lineSpacing = 20;

    if (!pgmBuffer.isEmpty())
    {
      int lines = (height() - 20) / lineSpacing;

      for (int j = 0; j < lines; j++)
      {
        sSpace = "";
        sPgm = "";
        sAscii = "";
        for (int i = 0; i < 16; i++)
        {
          int index = i + j * 16 + verticalScrollPosition * 16;
          if (index >= pgmBuffer.length())
            break;
          if (i == 0)
          {
            sSpace = QString::number(index, 16).toUpper();
            sPgm = "";
            sAscii = "";
          }
          sPgm += QString::number((QByte) pgmBuffer[index] >> 4, 16).toUpper();
          sPgm += QString::number((QByte) pgmBuffer[index] & 0x0F, 16).toUpper();
          if (i == 7)
            sPgm += "  ";
          else
            sPgm += " ";
          if (31 < (QByte) pgmBuffer[index] && (QByte) pgmBuffer[index] < 127)
            sAscii += pgmBuffer[index];
          else
            sAscii += ".";
        }
        painter.drawText(QRect(10, 10 + j * lineSpacing, 50, lineSpacing), Qt::AlignRight, sSpace);
        painter.drawText(QRect(80, 10 + j * lineSpacing, 390, lineSpacing), Qt::AlignLeft, sPgm);
        painter.drawText(QRect(480, 10 + j * lineSpacing, 200, lineSpacing), Qt::AlignLeft, sAscii);
      }
    }
  }

}//namespace lpzrobots

