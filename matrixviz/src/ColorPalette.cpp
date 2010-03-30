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
 *                                                                         *
 ***************************************************************************/

#include "ColorPalette.h"
//#include "ColorPaletteDialog.h"
#include "ListEntity.h"
#include <math.h>
#include <iostream>
#include <QtGui>
//#include <QtXml>
#include <QtXml/QDomDocument> //<qt4/QtXml/QDomDocument>
#include <QtXml/QDomElement> //<qt4/QtXml/QDomElement>
#include <QFile>
#include <QFileDialog>

using namespace std;

ColorPalette::ColorPalette(QWidget *parent)
: QWidget(parent){
  if( debug) cout << "in CP Konstrunktor" << endl;
  resize(30,150);
  setMaximumWidth(30);
  setMouseTracking(true); // enables tooltips while mousemoving over widget
  if( debug) cout << "struct matrixviz::STOP num1, num2, num3;" << endl;
  struct STOP *num1 = new STOP, *num2 = new STOP, *num3 = new STOP;
  QColor red(Qt::red), white(Qt::white), blue(Qt::blue);
  if( debug) cout << "num1.color = red;" << endl;
  num1->color = red;
  num1->pos = -1.;
  num2->color = white;
  num2->pos = 0.;
  num3->color = blue;
  num3->pos = 1.;
  if( debug) cout << "push_back" << endl;
  this->stops = new QVector<STOP*>();
  this->stops->push_back(num1);
  this->stops->push_back(num2);
  this->stops->push_back(num3);
  if( debug) cout << "initMaxMin" << endl;
  initMaxMin();
  currentPath = "";
  stopList = new QListWidget();
}

ColorPalette::~ColorPalette(){}

void ColorPalette::initMaxMin(){
  min = 900000000;
  max = -900000000;
  for(int i = 0; i < stops->size(); i++){
    if( stops->at(i)->pos < min) min = stops->at(i)->pos;
    if( stops->at(i)->pos > max) max = stops->at(i)->pos;
  }
}

QColor ColorPalette::pickColor(double val){
  // if the value is outer range return the color of the range
  if ( stops->first()->pos > val) return stops->first()->color;
  if ( stops->last()->pos < val) return stops->last()->color;
  STOP *pos1 = new STOP, *pos2 = new STOP;
  pos1->pos = -1000.;//min
  pos2->pos = 1000.;//max
  // find the nearest stops surrounding
  for ( int i = 0; i < stops->size(); i++){
    if( stops->at(i)->pos < val && stops->at(i)->pos > pos1->pos){
      pos1 = stops->at(i);
    }else if( stops->at(i)->pos > val && stops->at(i)->pos < pos2->pos){
      pos2 = stops->at(i);
    }else if ( stops->at(i)->pos == val ) return stops->at(i)->color;
  }
  int r1 = pos1->color.red(), r2 = pos2->color.red(),
    g1 = pos1->color.green(), g2 = pos2->color.green(),
    b1 = pos1->color.blue(), b2 = pos2->color.blue(),
    r,g,b;
  double p1 = pos1->pos, p2 = pos2->pos;
  // linear system
  r = floor(((r2-r1)/(p2-p1)) * (val - p1) + r1);
  g = floor(((g2-g1)/(p2-p1)) * (val - p1) + g1);
  b = floor(((b2-b1)/(p2-p1)) * (val - p1) + b1);
  return QColor(r,g,b);
}

void ColorPalette::paintEvent(QPaintEvent *){
  QPainter painter ( this );
  double step = (max - min) / height();
  for ( int i = 0; i < height(); i++){
    painter.setPen(pickColor(min + i * step));
    painter.drawLine(0, height() - i, width(), height() - i);
  }
  painter.end();
}

void ColorPalette::addStop(int num, QRgb color, double pos){
  STOP *newStop = new STOP;
  newStop->color = color;
  newStop->pos = pos;
  this->stops->insert(num, newStop);
}

void ColorPalette::deleteStop(int num){ //obsolete
  this->stops->remove(num);
}

void ColorPalette::resizeEvent(QResizeEvent *event){
  resize(event->size().width(),event->size().height());
  update();
  event->accept();
}

void ColorPalette::mouseDoubleClickEvent(QMouseEvent *){
//  ColorPaletteDialog *dialog = new ColorPaletteDialog(stops);
  QWidget *config = makeConfigBox();
  config->show();
//  connect(dialog, SIGNAL(refresh()), this, SLOT(refresh())); //oder so vll
  repaint();
}

void ColorPalette::mouseMoveEvent ( QMouseEvent *event ){
  //event->x/y();
  //setToolTip ( const QString &text );
  double step = (max - min) / height();
  double val = min + event->y() * step;
  setToolTip((const QString) QString::number(val));
}

//void ColorPalette::refresh(){ //SLOT
//  if (debug) cout << "in SLOT ColorPalette::refresh()" << endl;
//  update();
//}

//****** merged


QWidget* ColorPalette::makeConfigBox(){
  if (debug) cout << "in makeView():\nstops.size(): " << stops->size() << endl;
  // befüllen
  //QWidget* test = new QWidget();
  updateList();

  QWidget *configBox = new QWidget();
  QVBoxLayout *mainWindow = new QVBoxLayout();
  QHBoxLayout *loadSaveButtonLayout = new QHBoxLayout();

  QPushButton *load = new QPushButton( "load", configBox );
  QPushButton *save = new QPushButton( "save", configBox );
  connect(load, SIGNAL(pressed()), this, SLOT(loadStopList()));
  connect(save, SIGNAL(pressed()), this, SLOT(saveStopList()));
  loadSaveButtonLayout->addWidget(load);
  loadSaveButtonLayout->addWidget(save);

  mainWindow->addLayout(loadSaveButtonLayout);
  mainWindow->addWidget(stopList);
  //mainWindow->addWidget(test);
  //mainWindow->addWidget(cp);
  configBox->setLayout(mainWindow);
  configBox->resize(400,300);
//  buttons->addWidget(add);
//  buttons->addWidget(remove);
//  mainWindow->addLayout(buttons);
  //connect to close
  connect( this, SIGNAL(sendQuit()), configBox, SLOT(close()));
  return configBox;
}

void ColorPalette::updateList(){
  if ( debug) cout << "in updateList" << endl;
  stopList->clear();
  if ( debug) cout << "stoplist clear" << endl;
  for(int i = 0; i < stops->size(); i++){
    if ( debug) cout << "i: " << i << endl;
      QListWidgetItem *item = new QListWidgetItem("", stopList);
      item->setSizeHint(QSize(350,40)); //itemwidget wont show up without that (qtbug)
      stopList->addItem(item);

      ListEntity *w = new ListEntity(i, stops->at(i)->color, stops->at(i)->pos);

      connect(w, SIGNAL(addClicked(int)), this, SLOT(addStop(int)));
      connect(w, SIGNAL(remClicked(int)), this, SLOT(removeStop(int)));
      connect(w, SIGNAL(changeColor(int, QColor)), this, SLOT(changeStopColor(int, QColor)));
      connect(w, SIGNAL(changePos(int, double)), this, SLOT(changeStopPos(int, double)));
      stopList->setItemWidget(item, w);
    }
}

void ColorPalette::addStop(int i){ //SLOT
  //int QListWidget::currentRow () const
  if (debug) cout << "in addStop() i: " << i << endl;
  STOP *newEntry = new STOP;
  newEntry->pos = stops->at(i)->pos + 0.001;
  newEntry->color = Qt::white;
  stops->insert(i+1, newEntry);
  updateList();
//  emit refresh();
  update();
}

void ColorPalette::changeStopColor(int i, QColor color){ //SLOT
  //int QListWidget::currentRow () const
  stops->at(i)->color = color;
//  emit refresh();
  update();
}

void ColorPalette::changeStopPos(int i, double pos){ //SLOTdelete stopList->takeItem(i);
  if (debug) cout << "in ColorPaletteDialog::changeStopPos()..." << endl;
  stops->at(i)->pos = pos;
  //maybe reorder
  bool changed = false;
  for(int j = i; j > 0; j--){
    if(stops->at(i)->pos < stops->at(i-1)->pos) {
      STOP *temp = stops->at(i);
      stops->remove(i);
      stops->insert(i-1, temp);
      changed = true;
    }
  }
  for(int j = i; j <= stops->last()->pos; j++){
    if (stops->at(i)->pos > stops->at(i + 1)->pos) {
      STOP *temp = stops->at(i);
      stops->remove(i);
      stops->insert(i + 1, temp);
      changed = true;
    }
  }
  if (changed) updateList();

//  emit refresh();
  update();

}
void ColorPalette::removeStop(int i){ //SLOT
  if( stops->size() > 2){
    stops->remove(i);
//    emit refresh();
    update();
    updateList();
  }else{
    QMessageBox msgBox;
    msgBox.setText("The minimum amount of stops is 2!");
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.exec();
  }
}

void ColorPalette::loadStopList(){
  loadStopListFromFile(QFileDialog::getOpenFileName(this, tr("Load File"), "/home", tr("XML files (*.xml)")));
  update();
}

void ColorPalette::loadStopListFromFile(QString filename) {
  if (debug)
    cout << "in ColorPalette::loadStopList" << endl;
  if (debug)
     cout << "path: " << filename.toStdString() << endl;
  QDomDocument doc("StopList");
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly)) {
    if (file.fileName() != "")
      QMessageBox::warning(this, "", "Can't open!");
    return;
  }
  currentPath = file.fileName();
  if (debug) cout << "curentPath: " << currentPath.toStdString() << endl;
  QString error;
  if (!doc.setContent(&file, &error)) {
    file.close();
    QMessageBox::warning(this, "Error reading file!", error);
    return;
  }
  file.close();
  if (debug) cout << "file close" << endl;
  QDomElement root = doc.documentElement();
  if (root.tagName() != "Stops") {
    QMessageBox::warning(this, "", "Can't find Stops");
    return;
  }
  stops->clear();
  if (debug) cout << "stops clear" << endl;
  QDomNode n = root.firstChild();
  while (!n.isNull()) {
    QDomElement e = n.toElement();
    if (!e.isNull()) {
      if (e.tagName().startsWith("Stop")) {
        STOP *s = new STOP;

        s->pos = e.attribute("Position", "").toDouble();
        s->color = QColor(e.attribute("R", "").toDouble(), e.attribute("G", "").toDouble(),
            e.attribute("B", "").toDouble());

        stops->push_back(s);
        if (debug) cout << "stops push_back" << endl;
      }
    }
    n = n.nextSibling();
  }
  if (debug) cout << "updatelist" << endl;
  updateList();
  if (debug) cout << "loading complete" << endl;
}


void ColorPalette::saveStopList(){
  QDomDocument doc( "StopList" );

  QDomElement root = doc.createElement( "Stops" );
  doc.appendChild( root );

  for (int i = 0; i < stops->size(); i++){
    QDomElement stop = doc.createElement( QString("Stop").append(QString::number(i+1)));
    stop.setAttribute( "Position", stops->at(i)->pos );
    stop.setAttribute( "R", stops->at(i)->color.red() );
    stop.setAttribute( "G", stops->at(i)->color.green() );
    stop.setAttribute( "B", stops->at(i)->color.blue() );
    root.appendChild(stop);
  }

  QFile file(QFileDialog::getSaveFileName(this, tr("Save File"), currentPath, tr("XML files (*.xml)")));

  currentPath = file.fileName();
  if( !file.open( QIODevice::WriteOnly ) && (file.fileName() == "")) QMessageBox::warning(this, currentPath, "Can't save!");
  else{
    QTextStream ts(&file);
    ts << doc.toString();

    file.close();
  }
}

QString ColorPalette::getPath(){
  return currentPath;
}

void ColorPalette::closeEvent(QCloseEvent * event){
  if (debug) cout << "CP::closeEvent" << endl;
  emit sendQuit();
  event->accept();
}
