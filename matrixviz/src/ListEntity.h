/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Antonia Siegert (original author)                                  *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
    void updateFromData();

  signals:
    void addClicked(int i);
    void remClicked(int i);
    void changeColor(int i, QColor color);
    void changePos(int i, double pos);

  private:
    void fillButton();
};

#endif /* __LISTENTITY_H_ */
