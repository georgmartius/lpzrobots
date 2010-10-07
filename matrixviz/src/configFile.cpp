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
 *   Revision 1.4  2010-10-07 16:50:34  martius
 *   fixed bug in config loading if channel is in logfile but not in current file
 *   some debug output
 *   color gradients with dot instead of komma
 *
 *   Revision 1.3  2010/06/30 11:36:59  robot14
 *   removed typo
 *
 *   Revision 1.2  2010/05/11 16:51:47  robot14
 *   first working version
 *
 *   Revision 1.1  2010/03/30 13:19:09  robot14
 *   first version
 *
 *
 *                                                                         *
 ***************************************************************************/

#include "configFile.h"
#include <iostream>

using namespace std;


configFile::configFile(): saved(false){
}

configFile::~configFile(){

}

void configFile::load(MatrixVisualizer* mv){

  matrixVis = mv;

  QDomDocument doc( "MatrixVizConfigFile" );
  QFile file("matrixVizConf.xml");
  if( !file.open( QIODevice::ReadOnly ) ){
    cout << "no ConfigFile for MatrixVisualizer found!" << endl;
    return;
  }
  if( !doc.setContent( &file ) )
  {
    file.close();
    cout << "Error reading configfile" << endl;
    return;
  }
  file.close();
  QDomElement root = doc.documentElement();
  if( root.tagName() != "MatrixVizConfiguration" ){
    cout << "Configfile corrupted" << endl;
    return;
  }

  QDomNode n = root.firstChild();
  while( !n.isNull() )
  {
    QDomElement e = n.toElement();
    if( !e.isNull() )
    {
      if( e.tagName() == "MatrixVizWindow"){
        matrixVis->move(e.attribute("X", "").toInt(), e.attribute("Y", "").toInt());
      }
      if( e.tagName() == "VisualisationWindow" )
      {
        VisualiserSubWidget* window = 0;
        if( e.attribute( "mode", "") == "matrix"){
          MatrixPlotChannel* c = mv->getMatrixPlotChannel(e.attribute("source", ""));
          if(c){
            window = new VisualiserSubWidget(c, e.attribute("X", "").toInt(), e.attribute("Y", "").toInt(), 
                                             e.attribute("width", "").toInt(),
                                             e.attribute("height", "").toInt(), 
                                             e.attribute("colorPaletteFile", ""));
          }
        }else{
          VectorPlotChannel* c = mv->getVectorPlotChannel(e.attribute("source", ""));
          if(c){
            window = new VisualiserSubWidget(c, e.attribute("X", "").toInt(), e.attribute("Y", "").toInt(), 
                                             e.attribute("width", "").toInt(),
                                             e.attribute("height", "").toInt(), 
                                             e.attribute("colorPaletteFile", ""));
          }            
        }
        if(window){
          window->switchVisMode(e.attribute("visMode", "").toInt());
          newOpenedWindow(window);
          matrixVis->connectWindowForUpdate(window);
          window->show();
        }
      }
    }

    n = n.nextSibling();
  }
    //auslesen und VisSubWid erstellen


}

void configFile::save(){
  if (debug)
    cout << "configFile::save()" << endl;
  QDomDocument doc("MatrixVizConfigFile");

  QDomElement root = doc.createElement("MatrixVizConfiguration");
  doc.appendChild(root);

  QDomElement win = doc.createElement("MatrixVizWindow");
  win.setAttribute("X", matrixVis->pos().x());
  win.setAttribute("Y", matrixVis->pos().y());
  root.appendChild(win);

  for (QList<VisualiserSubWidget*>::iterator it = openWindows.begin(); it != openWindows.end(); it++) {
    QDomElement win = doc.createElement("VisualisationWindow");
    win.setAttribute("source", (*it)->getChannelName());
    win.setAttribute("mode", (*it)->getMode());
    win.setAttribute("visMode", (*it)->getVisMode());
    win.setAttribute("X", (*it)->pos().x());
    win.setAttribute("Y", (*it)->pos().y());
    win.setAttribute("width", (*it)->getSize().width());
    win.setAttribute("height", (*it)->getSize().height());
    win.setAttribute("colorPaletteFile", (*it)->getColorPaletteFilepath());
    root.appendChild(win);
  }

  QFile file("./matrixVizConf.xml");

  if (!file.open(QIODevice::WriteOnly))
    cout << "Can't save configuration!" << endl;
  else {
    QTextStream ts(&file);
    ts << doc.toString();

    file.close();
  }
}
//register every opened window
void configFile::newOpenedWindow(VisualiserSubWidget* window){
  if(debug) cout << "in configFile::newOpenedWindow" << endl;
  //connecting for quit and save
  connect(this, SIGNAL(sendQuit()), window, SLOT(close()));
  connect(window, SIGNAL(windowClosed(VisualiserSubWidget*)), this, SLOT(windowClosed(VisualiserSubWidget*)));
  openWindows.append(window);
}
//if a window closed externally then remove it from list
void configFile::windowClosed(VisualiserSubWidget* window){
  openWindows.removeAt(openWindows.indexOf(window));
}

void configFile::doQuit(){
  if (debug) cout << "emit sendquit" << endl;
  if(!saved){
    saved = true;
    save();
    emit sendQuit();
  }
}
