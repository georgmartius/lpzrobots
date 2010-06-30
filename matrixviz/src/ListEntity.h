/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    mam06fyl@studserv.uni-leipzig.de (robot14)                           *
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
 *   DESCRIPTION                                                           *
 *                                                                         *
 *   Visualization tool for matrices...                                    *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2010-06-30 11:37:51  robot14
 *   changed input for position of a stop from QLineEdit to QDoubleSpinBox
 *
 *                                                                         *
 ***************************************************************************/

#include <QColor>
#include <QtGui>

#ifndef __LISTENTITY_H_
#define __LISTENTITY_H_

class ListEntity : public QWidget{

  Q_OBJECT

  public:
    ListEntity(int i, QColor color, double pos, QWidget *parent = 0);
    ~ListEntity();

    int i;
    QColor color;
    double pos;
    QPushButton* button;
    QDoubleSpinBox* posEdit;

  public slots:
    void addClicked();
    void remClicked();
    void changeColor();
    void changePos(double pos);

  signals:
    void addClicked(int i);
    void remClicked(int i);
    void changeColor(int i, QColor color);
    void changePos(int i, double pos);

  private:
    void fillButton();
};

#endif /* __LISTENTITY_H_ */
