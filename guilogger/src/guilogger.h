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

#include <q3mainwindow.h>
#include <q3popupmenu.h>
#include <qstringlist.h>
#include <qlabel.h>
#include <qdialog.h>
#include <q3ptrlist.h>
#include <qlinkedlist.h>
#include <qlayout.h>
#include <qstringlist.h>
#include <qscrollarea.h>
//#include <qtimer.h>
#include <q3ptrqueue.h>
#include <qmutex.h>
#include <qmap.h>
#include <q3valuelist.h>
#include <q3scrollview.h>
#include <q3vbox.h>
#include <q3boxlayout.h>
#include <q3listbox.h>
#include <qlineedit.h>
#include <qpushbutton.h>
#include <qslider.h>
#include <q3listview.h>
#include <q3textedit.h>

#include <list>
#include "taggedcheckbox.h"
#include "gnuplot.h"
//#include <queue.h>
#include "inifile.h"
#include "commlineparser.h"

class ChannelRow;
class ChannelSelectRow;
class QTimer;

/** \brief Base class for layout and all the visualisation stuff
  * \author Dominic Schneider
  */
class guilogger: public Q3MainWindow
{
    Q_OBJECT

public:
    guilogger(const CommLineParser& configobj);
    ~guilogger();
    void setChannels(QStringList &clist);
    void setChannels(const char &clist);
    void addChannel(const QString &name, const QString &title="", const QString &style="lines");
    void putData(const QString &name, double data);

private slots:
    void taggedCheckBoxToggled(const Tag& tag, int gpwindow, bool on);
    void taggedComboBoxChanged(const Tag& tag, int gpwindow, const QString& entry);
    void receiveRawData(QString);
    void update();
    void GNUPlotUpdate();
    void save();
    void load();
    void dataSliderValueChanged(int value);
    void dataSliderReleased();
    void horizonSliderValueChanged(int value);
    void horizonSliderReleased();
    void sendButtonPressed();

signals:
    void quit();

private:
    void updateSliderPlot();
    int  analyzeFile();

private:
    typedef QMap<QString, Q3ValueList<int> > ChannelToWindowMap;  // Zuordnung von Channels auf PlotWindows
    
    Q3PtrList<ChannelRow> ChannelRowPtrList; // für Grafikelemente
    QLinkedList<QString> inputbuffer;
    Q3BoxLayout* layout;
    QWidget* channelandslider;
    Q3BoxLayout* channelandsliderlayout;
    Q3BoxLayout* channellayout;
    Q3BoxLayout* commlayout;
    Q3BoxLayout* sendlayout;
    //    Q3ScrollView* sv;
    QScrollArea* sv;
    QWidget* channelWidget;
    QWidget* commWidget; 
    
    Q3TextEdit   *parameterlistbox;
    QLineEdit   *paramvaluelineedit;
    QPushButton *sendbutton;
    QSlider     *dataslider;
    QSlider     *horizonslider;
    QLabel      *dataslidervalue;
    QLabel      *horizonslidervalue;
    
    Q3PopupMenu  *filemenu;
    
    Gnuplot<QString> *gp;
    std::list<bool*> *buttonArray;
    bool *gpWindowVisibility;
    ChannelToWindowMap KnownChannels;  // Channels from the ConfigFile
    ChannelSelectRow* ref1channels;    // Channels to use for Reference (x axis)
    QString* ref1channelsnames; // this is only needed for initialisation time
    Q3ValueList<QString> ChannelList;
    
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
