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

#include "ListEntity.h"
#include <QColorDialog>

/*
 * Editable Entity in the QListWidget of the optionWidget of ColorPalette
 */

ListEntity::ListEntity(int i, QColor color, double pos, QWidget *parent)
  : QWidget(parent){
  this->i = i;
  this->color = color;
  this->pos = pos;
  button = new QPushButton();
  fillButton();
  connect(button, SIGNAL(clicked()), this, SLOT(changeColor()));
  button->setMaximumSize(20, 20);
  button->setMinimumSize(20, 20);

  posEdit = new QDoubleSpinBox();
  posEdit->setRange ( -99., 99.);
  posEdit->setValue(pos);
  posEdit->setDecimals(2);
  posEdit->setSingleStep(0.1);
  posEdit->setMinimumSize ( 40, 15 );
  connect(posEdit, SIGNAL(valueChanged(double)), this, SLOT(changePos(double))); //if l losses focus or enter pressed

  QPushButton* add = new QPushButton("+");
  connect(add, SIGNAL(clicked()), this, SLOT(addClicked()));
  add->setMaximumSize(20, 20);
  add->setMinimumSize(20, 20);

  QPushButton* rem = new QPushButton("-");
  connect(rem, SIGNAL(clicked()), this, SLOT(remClicked()));
  rem->setMaximumSize(20, 20);
  rem->setMinimumSize(20, 20);

  QHBoxLayout* hlayout = new QHBoxLayout();
  //hlayout->setMargin(10);
  hlayout->addWidget(button);
  hlayout->addWidget(posEdit);
  hlayout->addWidget(add);
  hlayout->addWidget(rem);
  hlayout->setContentsMargins(0,0,0,0);
  setLayout(hlayout);
}

ListEntity::~ListEntity() {
}

void ListEntity::fillButton(){
  QPixmap pix = QPixmap(20,20);
  pix.fill(color);
  button->setIcon(QIcon((const QPixmap) pix));
}

void ListEntity::addClicked() { //SLOT
  emit addClicked(i);
}

void ListEntity::remClicked() { //SLOT
  emit remClicked(i);
}

void ListEntity::changeColor() { //SLOT //colorchooser
  QColor newColor= QColorDialog::getColor ((const QColor) color, this);
  this->color = newColor;
  fillButton();
  update();
  emit changeColor(i, newColor);
}

void ListEntity::changePos(double pos) { //SLOT
  emit changePos(i, pos);
}

void ListEntity::updateFromData() { //SLOT
  fillButton();
  posEdit->setValue(pos);    
}


