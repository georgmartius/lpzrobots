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
#include "guilogger.h"

#include <utility>
#include "channelrow.h"
#include <qthread.h>
#include <qapplication.h>


guilogger::guilogger()
//    : QMainWindow( 0, "guilogger", WDestructiveClose )
    : QDialog( 0, "guilogger")
{
    plotwindows = 3;
    updaterate_pwwindow = 20;
    framecounter=0;

    layout = new QBoxLayout(this, QBoxLayout::TopToBottom);

    ChannelRow* row = new ChannelRow("Channels", plotwindows, this);
    layout->addWidget(row);
    channellist.append(row);
    resize( 450, 600 );

    gp = new Gnuplot<QString>[plotwindows];
    nameslists = new listtype[plotwindows];

    gpWindowVisibility = new bool[plotwindows];
    for(int i=0; i<plotwindows; i++) gpWindowVisibility[i]=true;  // am Anfang alle Fenster sichtbar

    timer = new QTimer( this );
    connect( timer, SIGNAL(timeout()), SLOT(update()) );
    timer->start( 10, FALSE );

    plottimer = new QTimer( this);
    connect(plottimer, SIGNAL(timeout()), SLOT(GNUPlotUpdate()));
    plottimer->start(100, FALSE);
}


guilogger::~guilogger()
{   delete []gp;
    delete []nameslists;
}


void guilogger::taggedCheckBoxToggled(const Tag& tag, int gpwindow, bool on)
{
//    printf("%s toggled, %i ", tag.latin1(), gpwindow);
//    if(on) printf("on\n");else printf("off\n");

    if(tag == "Channels")              // komplettes Fenster abschalten
    {   //if(on) gpWindowVisibility[gpwindow]=true;
        //else gpWindowVisibility[gpwindow]=false;
        // channellist    //enthält Grafikelemente
    }

    if( on) gp[gpwindow].show(tag);  // einzelnen Kanal abschalten
    else gp[gpwindow].hide(tag);

//    update();
}


void guilogger::addChannel(const QString &name, const std::string &title, const std::string &style)
{
    printf("Trying to add Channel: %s\n", name.latin1());
    int status=0;
    listtype::iterator i;

    for(int i=0; i<plotwindows; i++) nameslists[i] = gp[i].getNames();  // Namenslisten aus dem GNUPlot holen

    for(int k=0; k<plotwindows; k++)
    {   for (i=nameslists[k].begin(); i != nameslists[k].end(); i++)
            if(*i == name) break;     // 1. Element mit Eintrag name finden...
        if(i == nameslists[k].end()) 
        {   gp[k].addChannel(name, title, style);   //Channel noch nicht vorhanden, dann eintragen
            status++;
            gp[k].hide(name);
        }
    }

    if(status==plotwindows)
    {   ChannelRow* newrow = new ChannelRow(name, plotwindows, this);  // neues Grafikelement für Channel erzeugen
        layout->addWidget(newrow);
        newrow->show();
        channellist.append(newrow);
        printf("   Channel added\n");
    }
    else printf("   Channel already inserted.\n");

}

void guilogger::receiveRawData(char *data)
{   QMutex mutex;
    mutex.lock();
    inputbuffer.enqueue(new QString(data));
    mutex.unlock();

//    free(data);
}


void guilogger::putData(const QString &name, double data)
{
    for(int i=0; i<plotwindows; i++) gp[i].putData(name, data);  // in jedes Fenster Daten putten
}


void guilogger::update()
{   QString *data;
    QStringList parsedString;
    QStringList::iterator i;
    QMutex mutex;

    while(1)
    {  mutex.lock();
       data = inputbuffer.dequeue();
       mutex.unlock();
       if (data==0) break;

       parsedString = QStringList::split(' ', *data);

       if(*(parsedString.begin()) == "#C")   //Channels einfügen
       {   parsedString.remove(parsedString.begin());  // remove #C preambel
           for(i=parsedString.begin(); i != parsedString.end(); i++) addChannel(*i);

           //framecounter++;
           for(int i=0; i<plotwindows; i++) if(gpWindowVisibility[i]) gp[i].plot();
       }
       else   // Daten einlesen
       {
           listtype::iterator inames = nameslists[0].begin();
           i = parsedString.begin();

           while((i != parsedString.end()) && (inames != nameslists[0].end()))
           {   putData(*inames, (*i).toFloat());
               i++;
               inames++;
           }
           
//           framecounter++;
//           if((framecounter % 20) == 0) for(int i=0; i<plotwindows; i++) if(gpWindowVisibility[i]) gp[i].plot();
       }

       printf("Parse: %s\n", data->latin1());
    }

//    framecounter++;
//    if((framecounter % 10) == 0)
//      for(int i=0; i<plotwindows; i++) if(gpWindowVisibility[i]) gp[i].plot();

}

void guilogger::GNUPlotUpdate()
{    //if((framecounter % 20) == 0) for(int i=0; i<plotwindows; i++) if(gpWindowVisibility[i]) gp[i].plot();
    framecounter++; 
    int i = framecounter % plotwindows;
    gp[i].plot();
    //     if(i < plotwindows ) gp[i].plot();
}

// deprecated
void guilogger::receiveGNUPlotChannels(QStringList &names)
{
    QStringList::iterator i = names.begin();

    while(i != names.end())
    {   addChannel(*i);
        i++;
    }

    // damit die dynamisch erzeugten Zeilenelemente angezeigt werden
    QThread::postEvent(this, new QShowEvent()); // Hack !!?? 
}

// deprecated
void guilogger::receiveGNUPlotData(QStringList &data)
{
    QStringList::iterator idata = data.begin();
    listtype::iterator inames = nameslists[0].begin();

    while((idata != data.end()) && (inames != nameslists[0].end()))
    {   putData(*inames, (*idata).toFloat());
        idata++;
        inames++;
    }

    update();
}

