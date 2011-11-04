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

#include "ScaleFunction.h"

/*
 * define functions for linearisation of values
 */

ScaleFunction::ScaleFunction(QWidget *parent): QWidget(parent), func(0), base(2), n(2), mul(1){

  baseEdit = new QLineEdit(QString::number(base, 'f'));
  baseEdit->setInputMask("09.00000");// O_o
  baseEdit->setMinimumSize ( 50, 15 );
  connect(baseEdit, SIGNAL(textChanged(const QString &)), this, SLOT(changeBase(const QString &)));
  bLabel = new QLabel("b:");

  mulEdit = new QLineEdit(QString::number(mul, 'f'));
  mulEdit->setInputMask("#09.00000");// O_o
  mulEdit->setMinimumSize(50, 15);
  connect(mulEdit, SIGNAL(textChanged(const QString &)), this, SLOT(changeMultiplier(const QString &)));
  mLabel = new QLabel("m:");

  nEdit = new QLineEdit(QString::number(n, 'f'));
  nEdit->setInputMask("#09.00000");// O_o
  nEdit->setMinimumSize(50, 15);
  connect(nEdit, SIGNAL(textChanged(const QString &)), this, SLOT(changeN(const QString &)));
  nLabel = new QLabel("n:");

  funcBox = new QComboBox(this);
  funcBox->addItem("x");
  funcBox->addItem("m log_b x");
  funcBox->addItem("m x^n");
  //add new function here
  connect(funcBox, SIGNAL(activated(int)), this, SLOT(changeFunction(int)));

  hlayout = new QHBoxLayout();

  hlayout->addWidget(mLabel);
  hlayout->addWidget(mulEdit);
  hlayout->addWidget(bLabel);
  hlayout->addWidget(baseEdit);
  hlayout->addWidget(nLabel);
  hlayout->addWidget(nEdit);

  mLabel->hide();
  mulEdit->hide();
  bLabel->hide();
  baseEdit->hide();
  nLabel->hide();
  nEdit->hide();

  vlayout = new QVBoxLayout();
  //hlayout->setMargin(10);
  vlayout->addWidget(funcBox);
  vlayout->addLayout(hlayout);
  vlayout->setContentsMargins(0,0,0,0);
  setLayout(vlayout);
}

ScaleFunction::~ScaleFunction(){

}

double ScaleFunction::getValue(double val){
  switch(func){
    // y = x
    case 0:
      return val;
      break;
    // y = m * log_b x
    case 1:
      if(val > 0.000001){
        //positive
        return (mul * (log(val)/log(base)));
      }else{
        if(-0.000001 <= val && val <= 0.000001){
          return 0.;
        }else{
          //negative
          return -1. * mul * (log(-1. * val)/log(base));
        }
      }
      break;
    // y = m * x^n
    case 2:
      if (-0.000001 <= val && val <= 0.000001) {
        return 0.;
      } else {
        if( val > 0.000001 ){
          return (mul * pow(val, n));
        }else{
          return (-1. * mul * pow(-1. * val, n));
        }
      }
      break;
      //catch new function here
  }
  return 0.;
}

void ScaleFunction::changeFunction(int i){
  func = i;
  mLabel->hide();
  mulEdit->hide();
  bLabel->hide();
  baseEdit->hide();
  nLabel->hide();
  nEdit->hide();
  switch(func){
    case 0:
      break;
    case 1:
      mLabel->show();
      mulEdit->show();
      bLabel->show();
      baseEdit->show();
      break;
    case 2:
      mLabel->show();
      mulEdit->show();
      nLabel->show();
      nEdit->show();
      break;
  }
}

void ScaleFunction::changeBase(const QString &val){
  base = val.toDouble();

}

void ScaleFunction::changeMultiplier(const QString &val){
  mul = val.toDouble();
}

void ScaleFunction::changeN(const QString &val){
  n = val.toDouble();
}
