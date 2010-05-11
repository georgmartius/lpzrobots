/*
 * ListEntity.cpp
 *
 *  Created on: 13.01.2010
 *      Author: oni
 */

#include "ListEntity.h"
#include <QColorDialog>
//#include <QIcon>

ListEntity::ListEntity(int i, QColor color, double pos, QWidget *parent)
  : QWidget(parent){
  this->i = i;
  this->color = color;
  this->pos = pos;
  button = new QPushButton();
//  QPixmap pix = QPixmap(20,20);
//  pix.fill(color);
//  button->setIcon(QIcon((const QPixmap) pix));
  fillButton();
  //button->setPixmap(const QPixmap &);
  connect(button, SIGNAL(clicked()), this, SLOT(changeColor()));
  button->setMaximumSize(20, 20);
  button->setMinimumSize(20, 20);

  lineEdit = new QLineEdit(QString::number(pos, 'f'));
  lineEdit->setInputMask("#09.00000");// O_o
  lineEdit->setMinimumSize ( 80, 15 );
  connect(lineEdit, SIGNAL(textChanged(const QString &)), this, SLOT(changePos(const QString &))); //if l losses focus or enter pressed

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
  hlayout->addWidget(lineEdit);
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
void ListEntity::changePos(const QString &text) { //SLOT
  pos = text.toDouble();
  emit changePos(i, pos);
}
