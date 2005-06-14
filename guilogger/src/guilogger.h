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
#include <qstringlist.h>
#include <qlabel.h>
#include <qdialog.h>
#include <qptrlist.h>
#include <qlayout.h>
#include <qstringlist.h>
#include <qtimer.h>
#include <qptrqueue.h>

#include <string>
#include <list>
#include "taggedcheckbox.h"
#include "gnuplot.h"
//#include <queue.h>

class ChannelRow;

/** \brief Base class for layout and all the visualisation stuff
  * \author Dominic Schneider
  */
class guilogger: public QDialog
{
    Q_OBJECT

public:
    guilogger();
    ~guilogger();
    void setChannels(QStringList &clist);
    void setChannels(const char &clist);
    void addChannel(const QString &name, const std::string &title="", const std::string &style="lines");
    void putData(const QString &name, double data);

private slots:
    void taggedCheckBoxToggled(const Tag& tag, int gpwindow, bool on);
    void receiveGNUPlotChannels(QStringList &names);  // deprecated
    void receiveGNUPlotData(QStringList &data);       // deprecated
    void receiveRawData(char *);
    void update();
    void GNUPlotUpdate();
    
private:
    typedef std::list<QString> listtype;

    QPtrList<ChannelRow> channellist; // für Grafikelemente
    QPtrQueue<QString> inputbuffer;
    QBoxLayout* layout;

    Gnuplot<QString> *gp;
    listtype *nameslists;
    std::list<bool*> *buttonArray;
    bool *gpWindowVisibility;

    int plotwindows;
    int updaterate_pwwindow;
    int framecounter;

    QTimer *timer;
    QTimer *plottimer;
};


#endif
