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
#include <qmenubar.h>
#include <qtimer.h>
#include <qregexp.h>


guilogger::guilogger(int datadelayrate, int plotwindows) : QMainWindow( 0, "guilogger")
{
    this->plotwindows   = plotwindows;    // per default parameter  3
    this->datadelayrate = datadelayrate;  // per default parameter 10
    framecounter = 0;
    datacounter  = 0;

    setCentralWidget(new QWidget(this, "Central_Widget"));
    layout        = new QHBoxLayout(centralWidget());

    channelWidget = new QWidget(centralWidget());
    channelWidget->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred,2,0, FALSE));
    commWidget = new QWidget(centralWidget()); 
    commWidget->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred,1,0, FALSE));

    
    layout-> addWidget(channelWidget);
    layout-> addWidget(commWidget);

    channellayout = new QVBoxLayout(channelWidget);
    commlayout    = new QVBoxLayout(commWidget);

    parameterlistbox    = new QListBox(commWidget);
    paramvaluelineedit = new QLineEdit(commWidget);
    sendbutton         = new QPushButton("Send",commWidget);

    commlayout->addWidget(parameterlistbox);
    commlayout->addWidget(paramvaluelineedit);
    commlayout->addWidget(sendbutton);

//    sv = new QScrollView(channelWidget);
//    channellayout->addWidget(sv);
    //    channellayout = new QBoxLayout(sv->viewport(), QBoxLayout::TopToBottom);
    
//    channellayout = new QVBoxLayout(centralWidget());
//    commlayout    = new QVBoxLayout(centralWidget());

//    layout->addLayout(channellayout);
//    layout->addLayout(commlayout);

    
    
    
    
    
    
    
    filemenu = new QPopupMenu(this);
    menuBar()->insertItem("&File", filemenu);
    filemenu->insertItem("&Save", this, SLOT(save()));
    filemenu->insertItem("&Load", this, SLOT(load()));
//    filemenu->insertItem("&Exit", this, SLOT(quit()));

    ChannelRow* row = new ChannelRow("Channels", plotwindows, channelWidget);
    channellayout->addWidget(row);
    
//    ChannelRow* row = new ChannelRow("Channel", plotwindows, sv->viewport());
//    sv->enableClipper(TRUE);
//    sv->addChild(row);
        
    ChannelRowPtrList.append(row);
    resize( 450, 600 );

    gp = new Gnuplot<QString>[plotwindows];

    for(int k=0; k<plotwindows; k++)
    {
    }
    
    gpWindowVisibility = new bool[plotwindows];
    for(int i=0; i<plotwindows; i++) gpWindowVisibility[i]=true;  // am Anfang alle Fenster sichtbar

    timer = new QTimer( this );
    connect( timer, SIGNAL(timeout()), SLOT(update()) );
    timer->start( 10, FALSE );

    plottimer = new QTimer( this);
    connect(plottimer, SIGNAL(timeout()), SLOT(GNUPlotUpdate()));
    plottimer->start(100, FALSE);

    load();
}


guilogger::~guilogger()
{   delete []gp;
//    delete []nameslists;
}


/// what happens if one of the checkboxes is toggled
void guilogger::taggedCheckBoxToggled(const Tag& tag, int gpwindow, bool on)
{
//    printf("%s toggled, %i ", tag.latin1(), gpwindow);
//    if(on) printf("on\n");else printf("off\n");

    if(tag == "Channels")              // komplettes Fenster abschalten
    {   //if(on) gpWindowVisibility[gpwindow]=true;
        //else gpWindowVisibility[gpwindow]=false;
    }

    if( on) gp[gpwindow].show(tag);  // einzelnen Kanal abschalten
    else gp[gpwindow].hide(tag);

    for(int i=0; i<plotwindows; i++) gp[i].plot();
}


/// saves the channel configuration to file
void guilogger::save()
{   ChannelRow *cr;
    FILE *outstream;
    QString secname, nr;
    
    cfgFile.setFilename("guilogger.cfg");
    
    outstream = fopen("guilogger.config", "w+");
    if(outstream == NULL) {printf("Cannot open file."); return;}

    for(int i=0; i<plotwindows; i++)
    {   cr = ChannelRowPtrList.first();
        fprintf(outstream,"Window %i\n", i);
        
        nr = QString::number(i, 10);
        secname = "Window";
//        secname += nr;
        
//        nr += '\n';
        
        IniSection *sec = cfgFile.addSection(secname);
        sec->addValue("Number", nr);

        while(cr != 0)
        {   if(cr->isChecked(i)) 
            {   fprintf(outstream,"%s\n", (cr->getChannelName()).latin1());
                sec->addValue("Channel", cr->getChannelName());
            }
            cr = ChannelRowPtrList.next();
        }
//        fprintf(outstream, "\n");
    }
    
    fclose(outstream);
    cfgFile.Save();
    cfgFile.Clear();
}


/// loads the channel configuration from file
void guilogger::load()
{   ChannelRow *cr;
//    FILE *outstream;
//    char *data;
    int pwin;
    QString qv;
    QRegExp re;
//    re.setWildcard(TRUE);
/*
    outstream = fopen("guilogger.config", "r");
    if(outstream == NULL) {printf("Cannot open config file.\n");return;}

    data = (char*) malloc(255);  // i hope its enough

    KnownChannels.clear();
    pwin = -1;
    while(fscanf(outstream, "%s", data) == 1)
    {
        if(data[0]=='W' && data[1]=='i' && data[2]=='n') 
        {   fscanf(outstream, "%i", &pwin);  // aktuelles Plotwindow einlesen
            continue;       // String bedarf keiner weiteren Abarbeitung
        }
        qs = data;
        re.setPattern(qs);
        
        if((pwin > -1) && (pwin < plotwindows) )
        {   
            KnownChannels[qs].append(pwin);  // add this channel to the knowen Channels

            cr = ChannelRowPtrList.first();  // Liste mit Channels die gesendet wurden
            while(cr != 0)
            {   //printf("Vgl. %s - %s\n", qs.latin1(), (cr->getChannelName()).latin1());
                if(qs==cr->getChannelName() || re.exactMatch(cr->getChannelName()))
                {   gp[pwin].show(qs);
                    cr->setChecked(pwin, TRUE);
                }
                cr = ChannelRowPtrList.next();
            }
        }
    }

    free(data);
    fclose(outstream);
*/
    IniSection* section;
    IniVar* var;
    
    KnownChannels.clear();
    pwin = -1;

    cfgFile.setFilename("guilogger.cfg");
    cfgFile.Load();

    for(section = cfgFile.sections.first(); section != 0; section = cfgFile.sections.next())
    {   if(section->getName() == "Window")
           for(var = section->vars.first(); var!=0; var = section->vars.next())
           {   qv = var->getValue();
               
               if(var->getName() == "Number") 
               {   pwin = qv.toInt();
//                   printf("WinNo = %i\n", pwin);
               }
               else if(var->getName() == "Channel")
               {        re.setPattern(qv);
//                        printf("  %s", qv.latin1());
                        KnownChannels[qv].append(pwin);

                        cr = ChannelRowPtrList.first();  // Liste mit Channels die gesendet wurden
                        while(cr != 0)
                        {   //printf("Vgl. %s - %s\n", qs.latin1(), (cr->getChannelName()).latin1());
                            if(qv==(cr->getChannelName()+"\n") || re.exactMatch(cr->getChannelName()))
                            {   gp[pwin].show(qv);
                                cr->setChecked(pwin, TRUE);
                            }
                            cr = ChannelRowPtrList.next();
                        }
               }
           }
        
    }
    
    printf("Config file loaded.\n");
}


/// adds the channel to GNUPlot and refuses adding the channel if it already exists
void guilogger::addChannel(const QString &name, const std::string &title, const std::string &style)
{
    if(ChannelList.find(name) == ChannelList.end()) // Channel noch nicht vorhanden...
    {   
        ChannelList.push_back(name);

        for(int k=0; k<plotwindows; k++)
        {   gp[k].addChannel(name, title, style);   // ...also in jedes GNUPlotfesnter feuern
            gp[k].hide(name);                       //    und per default nicht sichtbar machen
        }

        ChannelRow* newrow = new ChannelRow(name, plotwindows, channelWidget);           // neues Grafikelement für Channel erzeugen
//        ChannelRow* newrow = new ChannelRow(name, plotwindows, sv->viewport());           // neues Grafikelement für Channel erzeugen
        connect(newrow, SIGNAL(sendtaggedCheckBoxToggled(const Tag&, int, bool)),
                  this,   SLOT(taggedCheckBoxToggled( const Tag&, int, bool)));
        channellayout->addWidget(newrow);
//        framecounter++;
//        sv->addChild( newrow, 0, 30*framecounter);

        newrow->show();
        ChannelRowPtrList.append(newrow);

        QRegExp re;
        re.setWildcard(TRUE);

        ChannelToWindowMap::iterator it = KnownChannels.begin();  // durch die Map der bekannten Channels (aus config File) iterieren
        if(it != KnownChannels.end()) re.setPattern(it.key()); 

//        printf("Send %s.\n", name.latin1());  // DEBUG
        while(it != KnownChannels.end())  // guggen ob neuer Channel auf einen der Ausdrücke aus dem config file matcht
        {   //printf("  %s ", it.key().latin1());  // DEBUG

            if(name == it.key() || re.exactMatch(name))
            {    //printf("Match\n");   // DEBUG
                QValueList<int>::iterator lit = (*it).begin();  // wenn Ausdruck matcht, Plotwindows rausbekommen
                while(lit != (*it).end())
                {   int b = *lit;
                    newrow->setChecked(b, TRUE);
                    gp[b].show(name);
                    lit++;
                }
            }
//            else printf("\n");   // DEBUG

            it++;
            re.setPattern(it.key());
        }
    }


/*
    for(int i=0; i<plotwindows; i++) nameslists[i] = gp[i].getNames();  // Namenslisten aus dem GNUPlot holen

    for(int k=0; k<plotwindows; k++)        // in jedem Plotwindow schauen ob Channel schon vorhanden ist
    {   for (i=nameslists[k].begin(); i != nameslists[k].end(); i++)
            if(*i == name) break;         // 1. Element mit Eintrag "name" finden...
        if(i == nameslists[k].end()) 
        {   gp[k].addChannel(name, title, style);   //Channel noch nicht vorhanden, dann eintragen
            status++;
            gp[k].hide(name);
        }
    }

    QRegExp re;
    re.setWildcard(TRUE);
    
    if(status==plotwindows)
    {   ChannelRow* newrow = new ChannelRow(name, plotwindows, this);  // neues Grafikelement für Channel erzeugen
    channellayout->addWidget(newrow);
        newrow->show();
        ChannelRowPtrList.append(newrow);
        #ifdef DEBUG
           printf("   Channel added\n");
        #endif
        
        ChannelToWindowMap::iterator it = KnownChannels.begin();  // durch die Map iterieren
        if(it != KnownChannels.end()) re.setPattern(it.key()); 

//        printf("Send %s.\n", name.latin1());
        while(it != KnownChannels.end())  // guggen ob neuer Channel auf einen der Ausdrücke aus dem config file matcht
        {   printf("  %s ", it.key().latin1());

            if(name == it.key() || re.exactMatch(name))
            {    //printf("Match\n");
                 QValueList<int>::iterator lit = (*it).begin();  // wenn Ausdruck matcht, Plotwindows rausbekommen
                 while(lit != (*it).end())
                 {   int b = *lit;
                     newrow->setChecked(b, TRUE);
                     gp[b].show(name);
                     lit++;
                 }
            }
            else printf("\n");
            
            it++;
            re.setPattern(it.key());
        }
*/
        /*
        ChannelToWindowMap::iterator it = KnownChannels.find(name);    // ist eingefügter Channel in der Liste ???
        if(it != KnownChannels.end())     // aktuell eingefügter Channel ist in Liste vorhanden !!!
        {   //printf("%s.\n", it.data().latin1());
            QValueList<int>::iterator lit = (*it).begin();
            while(lit != (*it).end())
            {   int b = *lit;
                newrow->setChecked(b, TRUE);
                gp[b].show(name);
                lit++;
            }
        }
        */

}


/// enqueue the raw and unparsed data
void guilogger::receiveRawData(char *data)
{   
    queuemutex.lock();
       inputbuffer.enqueue(new QString(data));
    queuemutex.unlock();
}


/// put the data in every window available
void guilogger::putData(const QString &name, double data)
{
    for(int i=0; i<plotwindows; i++) gp[i].putData(name, data);
}


// emties the buffer queue and parses the data then putting it to GNUPlot
// updates the GNUPlot data queues with fresh data
void guilogger::update()
{   QString *data;
    QStringList parsedString;
    QStringList::iterator i;

    while(1)
    {  queuemutex.lock();
          data = inputbuffer.dequeue();
       queuemutex.unlock();
       if (data==0) break;
       
       parsedString = QStringList::split(' ', *data);  //parse data string with Space as separator

       if(*(parsedString.begin()) == "#C")   //Channels einlesen
       {
           parsedString.remove(parsedString.begin());  // remove #C preambel
           for(i=parsedString.begin(); i != parsedString.end(); i++) addChannel(*i);  //transmit channels to GNUPlot

           for(int i=0; i<plotwindows; i++) if(gpWindowVisibility[i]) gp[i].plot();  // show channels imediatly
       }
       else if( (*(parsedString.begin()))[0] != '#')  // Daten einlesen
       {
           QValueList<QString>::iterator channelname = ChannelList.begin();
           i = parsedString.begin();

           while((i != parsedString.end()) && (channelname != ChannelList.end()))
           {   putData(*channelname, (*i).toFloat());  // send data and correlated channel name to GNUPlot
               i++;
               channelname++;
           }
           datacounter++;
       }

       #ifdef DEBUG
          printf("Parse: %s\n", data->latin1());
       #endif
    }

}


// updates every n milliseconds one of the GNUPlot windows
void guilogger::GNUPlotUpdate()
{   //framecounter++; 
    //int i = framecounter % plotwindows;
    if(datacounter > datadelayrate)
    {   for(int i=0; i<plotwindows; i++) gp[i].plot();
        datacounter=0;
    }
}
