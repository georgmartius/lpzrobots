/***************************************************************************
 *   Copyright (C) 2005 by Dominic Schneider   *
 *   dominic@isyspc8   *
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
 ***************************************************************************/


#ifndef GUILOGGER_H
#define GUILOGGER_H

#include <qmainwindow.h>
#include <qpopupmenu.h>
#include <qstringlist.h>
#include <qlabel.h>
#include <qdialog.h>
#include <qptrlist.h>
#include <qlayout.h>
#include <qstringlist.h>
//#include <qtimer.h>
#include <qptrqueue.h>
#include <qmutex.h>
#include <qmap.h>
#include <qvaluelist.h>
#include <qscrollview.h>
#include <qvbox.h>
#include <qlistbox.h>
#include <qlineedit.h>
#include <qpushbutton.h>
#include <qslider.h>
#include <qlistview.h>
#include <qtextedit.h>

#include <string>
#include <list>
#include "taggedcheckbox.h"
#include "gnuplot.h"
//#include <queue.h>
#include "inifile.h"
#include "commlineparser.h"

class ChannelRow;
class QTimer;

/** \brief Base class for layout and all the visualisation stuff
  * \author Dominic Schneider
  */
class guilogger: public QMainWindow
{
    Q_OBJECT

public:
    guilogger(const CommLineParser& configobj);
    ~guilogger();
    void setChannels(QStringList &clist);
    void setChannels(const char &clist);
    void addChannel(const QString &name, const std::string &title="", const std::string &style="lines");
    void putData(const QString &name, double data);

private slots:
    void taggedCheckBoxToggled(const Tag& tag, int gpwindow, bool on);
    void receiveRawData(char *);
    void update();
    void GNUPlotUpdate();
    void save();
    void load();
    void dataSliderValueChanged(int value);
    void dataSliderReleased();
    void horizonSliderValueChanged(int value);
    void horizonSliderReleased();

private:
    void updateSliderPlot();
    int  analyzeFile();

private:
    typedef QMap<QString, QValueList<int> > ChannelToWindowMap;  // Zuordnung von Channels auf PlotWindows
    
    QPtrList<ChannelRow> ChannelRowPtrList; // für Grafikelemente
    QPtrQueue<QString> inputbuffer;
    QBoxLayout* channellayout;
    QBoxLayout* commlayout;
    QBoxLayout* layout;
    QScrollView* sv;
    QWidget* channelWidget;
    QWidget* commWidget; 
    
    QTextEdit   *parameterlistbox;
    QLineEdit   *paramvaluelineedit;
    QPushButton *sendbutton;
    QSlider     *dataslider;
    QSlider     *horizonslider;
    QLabel      *dataslidervalue;
    QLabel      *horizonslidervalue;
    
    QPopupMenu  *filemenu;
    
    Gnuplot<QString> *gp;
    std::list<bool*> *buttonArray;
    bool *gpWindowVisibility;
    ChannelToWindowMap KnownChannels;  // Channels from the ConfigFile
    QValueList<QString> ChannelList;
    
    int plotwindows;
    int framecounter;  //deprecated
    int datacounter;
    int datadelayrate;  // how much data traffic is neccessary to replot
    int buffersize;
    QString mode;
    QString filename;
    
    QTimer *timer;
    QTimer *plottimer;
    QTimer *filegraphtimer;
    
    QMutex queuemutex;

    IniFile cfgFile;
};


#endif
