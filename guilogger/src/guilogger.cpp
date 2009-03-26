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
#include "channelselectrow.h"
#include <qthread.h>
#include <qapplication.h>
#include <qmenubar.h>
#include <qtimer.h>
#include <qregexp.h>
#include <qscrollarea.h>
//Added by qt3to4:
#include <Q3HBoxLayout>
#include <Q3ValueList>
#include <QLabel>
#include <Q3PopupMenu>
#include <Q3VBoxLayout>

#include "quickmp.h"


guilogger::guilogger(const CommLineParser& configobj, const QRect& screenSize) 
  : Q3MainWindow( 0, "guilogger"), screenSize(screenSize) {
    plotwindows   = 5;    // per default parameter  3
    datadelayrate = 10;  // per default 10
    framecounter  = 0;
    datacounter   = 0;
    buffersize    = 250;
    int linecount = 0;
    const int startplottimer = 2000;
    const int maxplottimer = 5000;
    
    mode     = configobj.getMode();
    filename = configobj.getFile();

    load();  // load Config File
    
    gp = new Gnuplot<QString>[plotwindows];  // GNUPlots erzeugen    
    
    QString calcPositions = cfgFile.getValueDef("Misc","CalcPositions","no");
    QString windowLayout = cfgFile.getValueDef("Misc","WindowLayout","tlh");
    int windowsPerRowColumn = cfgFile.getValueDef("Misc","WindowsPerRowColumn","3").toInt();
    if(calcPositions.contains("yes")){
      // arrange Gnuplot windows
      int xstart = windowLayout.contains("l") ? 0 : screenSize.width();
      int xinc   = windowLayout.contains("l") ? 1 : -1;
      int ystart = windowLayout.contains("t") ? 0 : screenSize.height();
      int yinc   = windowLayout.contains("t") ? 1 : -1;
      bool hor   = windowLayout.contains("h");
      int xpos   = xstart;
      int ypos   = ystart;
      for(int k=0; k<plotwindows; k++) {
	QSize s = (windowsize.contains(k) ? windowsize[k] : QSize(400,300)) + QSize(10,50);      
	if(hor){
	  xpos = xstart+(k%windowsPerRowColumn)*xinc*s.width();
	  ypos = ystart+(k/windowsPerRowColumn)*yinc*s.height();
	}else{
	  xpos = xstart+(k/windowsPerRowColumn)*xinc*s.width();
	  ypos = ystart+(k%windowsPerRowColumn)*yinc*s.height();
	}
	windowposition.insert(k,QSize(xinc > 0 ? xpos : xpos-s.width(),yinc > 0 ? ypos : ypos-s.height()));
      }      
    }

// serial version    
//     for(int k=0; k<plotwindows; k++) {
//       QSize s = windowsize.contains(k) ? windowsize[k] : QSize(400,300);
//       if(windowposition.contains(k)){
// 	QSize pos = windowposition[k];
// 	gp[k].init(s.width(), s.height(), pos.width(), pos.height());	
//       }else{	
// 	gp[k].init(s.width(), s.height());
//       }
//     }

// parallel version    
    QMP_SHARE(windowsize);
    QMP_SHARE(windowposition);
    QMP_SHARE(gp);
    QMP_PARALLEL_FOR(k,0,plotwindows){
      QMP_USE_SHARED(windowsize, QMap<int, QSize>);
      QMP_USE_SHARED(windowposition, QMap<int, QSize>);
      QMP_USE_SHARED(gp, Gnuplot<QString>*);
      QSize s = windowsize.contains(k) ? windowsize[k] : QSize(400,300);
      if(windowposition.contains(k)){
	QSize pos = windowposition[k];
	gp[k].init(s.width(), s.height(), pos.width(), pos.height());	
      }else{	
	gp[k].init(s.width(), s.height());
      }
    }
    QMP_END_PARALLEL_FOR;



    IniSection GNUplotsection;    
    if(cfgFile.getSection(GNUplotsection,"GNUPlot",false)){      
      IniVar* var;
      QString qv;
      for(var = GNUplotsection.vars.first(); var!=0; var = GNUplotsection.vars.next()) {   
	qv = var->getValue();
	if(var->getName() == "Command") 
	  for(int k=0; k<plotwindows; k++) gp[k].command(qv.latin1());
      } 
    }

    if(mode == "file" && !filename.isEmpty()) linecount = analyzeFile();

    setCentralWidget(new QWidget(this, "Central_Widget"));
    layout           = new Q3VBoxLayout(centralWidget());
    channelandslider = new QWidget(centralWidget());
    channelandslider->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred,0,10, FALSE));
    
    layout->addWidget(channelandslider);    
    channelandsliderlayout = new Q3HBoxLayout(channelandslider);

    sv = new QScrollArea(centralWidget());
    channelandsliderlayout->addWidget(sv);

    //    sv = new Q3ScrollView(centralWidget());
    // sv->setResizePolicy(Q3ScrollView::AutoOneFit);
    
    //    channelWidget = new QWidget(sv->viewport());
    channelWidget = new QWidget(channelandslider);
    //    sv->addChild(channelWidget);
    sv->setWidget(channelWidget);
    sv->setWidgetResizable(true);
    // sv->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred,2,0, FALSE));


    channelWidget->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred,0,0, FALSE));

    commWidget = new QWidget(centralWidget()); 
    commWidget ->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum,1,1, FALSE));

    layout->addWidget(commWidget);

    QWidget *horizonsliderwidget = new QWidget(channelandslider);
    channelandsliderlayout->addWidget(horizonsliderwidget);

    QWidget *datasliderwidget;
    if(mode== "file"){
      datasliderwidget    = new QWidget(channelandslider);
      channelandsliderlayout->addWidget(datasliderwidget);
    }

    channellayout = new Q3VBoxLayout(channelWidget);
    commlayout    = new Q3HBoxLayout(commWidget);


    Q3BoxLayout *horizonsliderlayout = new Q3VBoxLayout(horizonsliderwidget);
    horizonslider = new QSlider(Qt::Vertical, horizonsliderwidget);
    horizonslider->setInvertedAppearance(true);
    horizonslidervalue = new QLabel(horizonsliderwidget);

    if(mode == "file") 
    {   horizonslider->setMaxValue(linecount);
        horizonslider->setValue(buffersize);
        horizonsliderlayout->addWidget(new QLabel("Win", horizonsliderwidget));
        horizonslidervalue->setText(QString::number(buffersize, 10));
    }
    else 
    {   horizonslider->setMinValue(10);
        horizonslider->setMaxValue(maxplottimer);   // MaxTimer für Plottimer
        horizonslider->setValue(startplottimer);       // actual Value for Plottimer
        horizonsliderlayout->addWidget(new QLabel("Timer", horizonsliderwidget));
        horizonslidervalue->setText(QString::number(startplottimer, 10));
    }
    horizonsliderlayout->addWidget(horizonslider);
    horizonsliderlayout->addWidget(horizonslidervalue);
    connect(horizonslider, SIGNAL(valueChanged(int )), this, SLOT(horizonSliderValueChanged(int )));
    connect(horizonslider, SIGNAL(sliderReleased())  , this, SLOT(horizonSliderReleased()));
    
    if(mode=="file"){
      Q3BoxLayout *datasliderlayout = new Q3VBoxLayout(datasliderwidget);
      dataslider    = new QSlider(Qt::Vertical, datasliderwidget);
      dataslider->setInvertedAppearance(true);
      dataslidervalue = new QLabel(datasliderwidget);
      dataslidervalue->setText(QString::number(0, 10));
      dataslider->setMaxValue(0);
      dataslider->setMaxValue(linecount);
      datasliderlayout->addWidget(new QLabel("Data", datasliderwidget));
      datasliderlayout->addWidget(dataslider);
      datasliderlayout->addWidget(dataslidervalue);
      connect(dataslider, SIGNAL(valueChanged(int )), this, SLOT(dataSliderValueChanged(int )));
      connect(dataslider, SIGNAL(sliderReleased())  , this, SLOT(dataSliderReleased()));
    }

    parameterlistbox   = new Q3TextEdit(commWidget);
    parameterlistbox->setMaximumHeight(60);
    paramvaluelineedit = new QLineEdit(commWidget);
    sendbutton         = new QPushButton("Send Cmd",commWidget);
    connect(sendbutton, SIGNAL(released())  , this, SLOT(sendButtonPressed()));

    parameterlistbox->setTextFormat(Qt::LogText);
    
    sendlayout = new Q3VBoxLayout(commlayout);
    sendlayout->addWidget(paramvaluelineedit);
    sendlayout->addWidget(sendbutton);

    commlayout->addWidget(parameterlistbox);
    
    filemenu = new Q3PopupMenu(this);
    menuBar()->insertItem("&Menu", filemenu);
    filemenu->insertItem("&Save Config", this, SLOT(save()));
    filemenu->insertItem("&Edit Config", this, SLOT(editconfig()));
    filemenu->insertItem("&Exit", this, SLOT(doQuit()));

    ChannelRow* row = new ChannelRow("Disable", plotwindows, channelWidget);    
    channellayout->addWidget(row);
    connect(row, SIGNAL(sendtaggedCheckBoxToggled(const Tag&, int, bool)),
	    this,   SLOT(taggedCheckBoxToggled( const Tag&, int, bool)));
    ChannelRowPtrList.append(row);
    QFrame* sep = new QFrame(commWidget);
    sep->setFrameStyle(QFrame::HLine | QFrame::Raised);
    channellayout->addWidget(sep);
    
    // Reference
    ref1channels = new ChannelSelectRow("Ref", plotwindows, channelWidget);    
    ref1channels->addItem("-");
    channellayout->addWidget(ref1channels);
    connect(ref1channels, SIGNAL(sendtaggedComboBoxChanged(const Tag&, int, const QString&)),
	    this,   SLOT(taggedComboBoxChanged( const Tag&, int, const QString&)));    

    resize( 450, 600 );
    
    gpWindowVisibility = new bool[plotwindows];
    for(int i=0; i<plotwindows; i++) gpWindowVisibility[i]=true;  // am Anfang alle Fenster sichtbar

    timer = new QTimer( this );
    connect( timer, SIGNAL(timeout()), SLOT(update()) );
    timer->start( 10, FALSE );

    plottimer = new QTimer( this);
    connect(plottimer, SIGNAL(timeout()), SLOT(GNUPlotUpdate()));
    plottimer->start(startplottimer, FALSE);

    filegraphtimer = NULL;
}


guilogger::~guilogger()
{   delete []gp;
}

void guilogger::horizonSliderReleased()
{    updateSliderPlot();
     if(mode != "file") plottimer->changeInterval(buffersize);  // change Plotintervall
}

void guilogger::dataSliderValueChanged(int value)
{    dataslidervalue->setText(QString::number(value, 10));
}

void guilogger::horizonSliderValueChanged(int value)
{    horizonslidervalue->setText(QString::number(value, 10));
     buffersize = value;
}

void guilogger::sendButtonPressed(){
  // send command to gnuplot
  for(int i=0; i<plotwindows; i++) 
    gp[i].command(paramvaluelineedit->text().latin1());   
}

void guilogger::dataSliderReleased()
{    updateSliderPlot();
}


void guilogger::updateSliderPlot()
{  
    if(mode!="file") return;
    bool status;
    int value = dataslider->value();
    QString cmd;
    
    ChannelRow *cr;
    int channel;
    parameterlistbox->clear();
    parameterlistbox->append("set style data lines");
    parameterlistbox->append("set zeroaxis");

    for(int i=0; i<plotwindows; i++){   
      if(gpWindowVisibility[i]){
	int refindex = ref1channels->getSelectedIndex(i);
	if(refindex > 0){
	  cmd = "plot \"" + filename + "\" ";
        }else{
	  cmd = "plot [" + QString::number(value, 10) + ":" + QString::number(value + buffersize)+"] \"" + filename + "\" ";
        }
	cr = ChannelRowPtrList.first();
	cr = ChannelRowPtrList.next(); //because first is disable
	
	channel=0;  
        status = FALSE;

        while(cr!= 0)
        {   channel++;
            if(cr->isChecked(i)) {
	      if(status==TRUE)
		cmd += ", \"\"";
	      if(refindex>0){
		cmd += " u " + QString::number(refindex,10) + ":";
	      }else{
		cmd += " u ";
	      }
	      cmd += QString::number(channel, 10)+ " t \""+ cr->getChannelName() + "\" ";
	      status = TRUE;	     
            }
            cr = ChannelRowPtrList.next();
        }

//         while(cr != 0)
//         {   channel++;
//             if(cr->isChecked(i)) cmd += ", \"\" u " + QString::number(channel, 10) + " t \""+ cr->getChannelName() + "\" ";
//             cr = ChannelRowPtrList.next();
//         }

//        gp[i].command("set style data lines");
//        gp[i].command("set zeroaxis");

        if(status) 
        {   gp[i].command(cmd.latin1());
            parameterlistbox->append(cmd);
        }
      }
    }
}


/// analyzes the file, send channels and return number of lines with data
int guilogger::analyzeFile()
{   char *s=NULL;
    int buffersize=0; 
    char c;
    int size=1, i=1;
    int linecount=0;

    FILE *instream;
//         printf("Counting Lines...   ");

    instream = fopen(filename, "r");
    if(instream == NULL) 
    {   printf("Cannot open input file.\n"); 
        return 0;
    }
    bool channelline=false;
    while(!channelline){
      size=1;
      c=0;	    
      while(c!= 10 && c != 13) 
	{
	  i = fread(&c, 1, 1, instream);
	  if(i==1) { 
	    size++; 
	    if(size>=buffersize){
	      buffersize=buffersize*2+1;
	      s=(char*)realloc(s, buffersize);
	    }
	    s[size-2] = c;
	  } else{
	    channelline=true;
	    break;
	  }
	}
      if (s[0] == '#' && s[1] == 'C') channelline=true;       
    }
    s[size-1]='\0';
    printf(s);

    receiveRawData(QString(s));

    do
    {   i = fread(&c, 1, 1, instream);
        if(i!=1) break;
        if(c == 10 || c == 13) linecount++;  // count only lines with data
    } while(i==1);

    fclose(instream);

//    linecount = (linecount-250 > 0)?linecount:0;  // um in einem Datensatz < Buffersize nicht scrollen zu können

//         printf("%i\n", linecount);
    if(s != NULL) free(s);
    return linecount;
}

/// what happens if one of the checkboxes is toggled
void guilogger::taggedCheckBoxToggled(const Tag& tag, int gpwindow, bool on)
{
    if(tag == "Disable"){ 
      if(on) gpWindowVisibility[gpwindow]=false;
      else gpWindowVisibility[gpwindow]=true;
    }
//     else if(tag == "Send")
//     {   
//       // send command to gnuplot
//       printf("Huhu: %s\n",paramvaluelineedit->text().latin1());
//       for(int i=0; i<plotwindows; i++) 
// 	gp[i].command(paramvaluelineedit->text().latin1()); 
//     }

    if(mode == "file") {   
      updateSliderPlot();
    } else {
        if( on) {   
	  gp[gpwindow].show(tag);  // einzelnen Kanal abschalten            
        } else { 
	  gp[gpwindow].hide(tag);
        }        
        for(int i=0; i<plotwindows; i++) gp[i].plot();  // sofort aktualisieren
    }
}

/// what happens if one of the ComboBox is changed
void guilogger::taggedComboBoxChanged(const Tag& tag, int gpwindow, const QString& entry)
{
    if(tag == "Ref"){ 
      if(entry.isEmpty() || entry == "-")
	gp[gpwindow].setUseReference1(false);
      else
	gp[gpwindow].setReference1(entry);
    }
}

void guilogger::save(){
  save(false);
}

/** saves the channel configuration to file
    if blank is used then a basic file is written without the window stuff
 */
void guilogger::save(bool blank)
{   ChannelRow *cr;
    QString nr;
    IniSection *section;

    cfgFile.setFilename("guilogger.cfg");

    section = cfgFile.sections.first();  // delete all "window" sections, because they will be rewritten in the next "for loop".
    while(1)
    {   if(section == 0) break;
        if(section->getName() != "Window")
        {   section = cfgFile.sections.next();
            continue;
        }

        cfgFile.sections.remove();  // remove current item, iterator++ 
        section = cfgFile.sections.current();
    }

    // If we don't have a Misc and GNUPlot section then add it.    
    IniSection sec;
    if(!cfgFile.getSection(sec,"Misc",false)){
      cfgFile.setComment("# Please restart guilogger to apply changes you make in the file");
      section=cfgFile.addSection("Misc");
      section->addValue("PlotWindows","5");
      section->addValue("CalcPositions","no"," # If yes then the gnuplot windows will be placed according to the layout");
      section->addValue("WindowLayout","tlh"," # From where to start: t: top, b: bottom, l: left, r: right, h: horizontal, v: vertical");
      section->addValue("WindowsPerRowColumn","3");
    }
    if(!cfgFile.getSection(sec,"GNUPlot",false)){
      section=cfgFile.addSection("GNUPlot");
      section->addValue("Command","set style data lines");
      section->addValue("Command","set zeroaxis");
    }    

    if(!blank){
      for(int i=0; i<plotwindows; i++){   	  
	nr = QString::number(i, 10);
	
	IniSection *sec = cfgFile.addSection("Window");
	sec->addComment("# you can also set the size and the position of a window. The channels can also contain wildcards like x* or C[0]*");
	sec->addValue("Number", nr);
	QString ref = ref1channels->getSelected(i);
	if(!ref.isEmpty() && ref.compare("-")!=0){
	  sec->addValue("Reference1", ref);
	}	  	
	if(windowsize.contains(i)){
	  QSize s = windowsize[i];
	  sec->addValue("Size",QString("%1x%2").arg(s.width()).arg(s.height()));
	} else 
	  sec->addValue("Size", "400x300");
	
	if(windowposition.contains(i)){
	  QSize pos = windowposition[i];
	  sec->addValue("Position",QString("%1 %2").arg(pos.width()).arg(pos.height())," # set to -1 to -1 ti make it automatically set by your windowmanager, see also calcPositions above");
	}else{
	  sec->addValue("Position", "-1 -1"," # set to any coordinate to place a window by hand, see also calcPositions above");	
	}  
	cr = ChannelRowPtrList.first();	  
	while(cr != 0)
	  { 	  
	    if(cr->isChecked(i))  sec->addValue("Channel", cr->getChannelName());
	    cr = ChannelRowPtrList.next();
	    }
      }      
    }      
    cfgFile.Save();
}


/// loads the channel configuration from file
void guilogger::load()
{   ChannelRow *cr;
    int pwin;
    QString qv;
    QRegExp re;
//    re.setWildcard(TRUE);

    IniSection* section;
    IniVar* var;

    ref1channelsnames=new QString[plotwindows];

    KnownChannels.clear();
    pwin = -1;

    cfgFile.setFilename("guilogger.cfg");
    if(!cfgFile.Load()){
      printf("Configuration file does not exist try to create it.\n");
      save(true); // this automatically creates the config (on disk and in memory)
    }
    

    for(section = cfgFile.sections.first(); section != 0; section = cfgFile.sections.next()) {   
      if(section->getName() == "Window"){
           pwin=0;
           for(var = section->vars.first(); var!=0; var = section->vars.next())
           {   qv = var->getValue();
	     
               if(var->getName() == "Number") pwin = qv.toInt();
	       else if(var->getName() == "Reference1"){
		 ref1channelsnames[pwin] = qv;
		 // todo: select appropriate in Ref1Channel ReferenceChannels[pwin]=qv;
	       } else if(var->getName() == "Channel")
               {        re.setPattern(qv);
                        KnownChannels[qv].append(pwin);

                        cr = ChannelRowPtrList.first();  // Liste mit Channels die gesendet wurden
                        while(cr != 0)
                        {   //printf("vgl. %s - %s\n", qs.latin1(), (cr->getChannelName()).latin1());
                            if(qv==(cr->getChannelName()) || re.exactMatch(cr->getChannelName()))
                            {   gp[pwin].show(qv);
                                cr->setChecked(pwin, TRUE);
                            }
                            cr = ChannelRowPtrList.next();
                        }
               } else if(var->getName() == "Size"){
		 int x,y;
		 if(sscanf(qv.latin1(),"%ix%i",&x,&y)==2)
		   windowsize.insert(pwin, QSize(x,y));
	       } else if(var->getName() == "Position"){
		 int w,h;
		 if(sscanf(qv.latin1(),"%i %i",&w,&h)==2)
		   windowposition.insert(pwin, QSize(w,h));
	       }	       
           }

      } else if(section->getName() == "Misc")
            for(var = section->vars.first(); var!=0; var = section->vars.next())
            {   qv = var->getValue();	      
	      if(var->getName() == "PlotWindows"){
		plotwindows = qv.toInt();
		delete[] ref1channelsnames;
		ref1channelsnames = new QString[plotwindows];
	      }
            }
    }

    printf("Config file loaded.\n");
}


void guilogger::editconfig(){
  system("$EDITOR ./guilogger.cfg");

}

/// adds the channel to GNUPlot and refuses adding the channel if it already exists
void guilogger::addChannel(const QString &name, const QString &title, const QString &style)
{
    if(ChannelList.find(name) == ChannelList.end()) // Channel noch nicht vorhanden...
    {   
        ChannelList.push_back(name);
	ref1channels->addItem(name);
	
        for(int k=0; k<plotwindows; k++)
        {   gp[k].addChannel(name, title, style);   // ...also in jedes GNUPlotfenster feuern
            gp[k].hide(name);                       //    und per default nicht sichtbar machen
        }

	// neues Grafikelement für Channel erzeugen
        ChannelRow* newrow = new ChannelRow(name, plotwindows, channelWidget); 
        connect(newrow, SIGNAL(sendtaggedCheckBoxToggled(const Tag&, int, bool)),
                  this,   SLOT(taggedCheckBoxToggled( const Tag&, int, bool)));
        channellayout->addWidget(newrow);
//        framecounter++;
//        sv->addChild( newrow, 0, 30*framecounter);

//        newrow->show();
        ChannelRowPtrList.append(newrow);

        QRegExp re;
        re.setPatternSyntax(QRegExp::Wildcard);

        ChannelToWindowMap::iterator it = KnownChannels.begin();  // durch die Map der bekannten Channels (aus config File) iterieren

//        printf("Send %s.\n", name.latin1());  // DEBUG
        while(it != KnownChannels.end())  // guggen ob neuer Channel auf einen der Ausdrücke aus dem config file matcht
        {   //printf("  %s ", it.key().latin1());  // DEBUG
            re.setPattern(it.key());
            if(name == it.key() || re.exactMatch(name))
            {   //printf("Match\n");   // DEBUG
                Q3ValueList<int>::iterator lit = (*it).begin();  // wenn Ausdruck matcht, Plotwindows rausbekommen
                while(lit != (*it).end())
                {   int b = *lit;
                    newrow->setChecked(b, TRUE);
                    gp[b].show(name);
                    lit++;
                }
            }

            it++;
        }
    }
}


/// enqueue the raw and unparsed data
void guilogger::receiveRawData(QString data)
{
    queuemutex.lock();
    //    fprintf(stderr,"Rec: %s", data.latin1());
    inputbuffer.push_back(data);
    queuemutex.unlock();
}


///  put the data in every window available
void guilogger::putData(const QString &name, double data)
{
    for(int i=0; i<plotwindows; i++) gp[i].putData(name, data);
}


void guilogger::doQuit(){
  emit quit();
}


/**  empties the buffer queue and parses the data then putting it to GNUPlot
  *  updates the GNUPlot data queues with fresh data
  */
void guilogger::update()
{   QString data;
    QStringList parsedString;
    QStringList::iterator i;

    while(!inputbuffer.empty()){ 
      queuemutex.lock();      
      data = inputbuffer.front();
      inputbuffer.pop_front();
      queuemutex.unlock();
      
      parsedString = QStringList::split(' ', data);  //parse data string with Space as separator
      QString& first = *(parsedString.begin());
      if(first == "#C")   //Channels einlesen
	{	
	  parsedString.erase(parsedString.begin());
	  //transmit channels to GNUPlot
	  for(i=parsedString.begin(); i != parsedString.end(); i++) addChannel((*i).stripWhiteSpace());	  
	  for(int i=0; i<plotwindows; i++) {
	    ref1channels->setSelected(i, ref1channelsnames[i]);
	    taggedComboBoxChanged("Ref", i, ref1channelsnames[i]);
	    GNUPlotUpdate(false);  // show channels immediatly
	  }
	}
      else if(first.length()>=2 &&  first[0] == '#' && first[1] == 'Q')   //Quit
	{
	  printf("Guilogger: Received Quit\n");
	  emit quit();
	}	
      else if( first[0] != '#')  // Daten einlesen
	{
	  Q3ValueList<QString>::iterator channelname = ChannelList.begin();
	  i = parsedString.begin();
	  
	  while((i != parsedString.end()) && (channelname != ChannelList.end()))
	    { // send data and correlated channel name to GNUPlot  
	      putData(*channelname, (*i).stripWhiteSpace().toFloat());  
	      i++;
	      channelname++;
	    }
	  datacounter++;
	}
    }
    
}


// updates every n milliseconds the GNUPlot windows
void guilogger::GNUPlotUpdate(){
  GNUPlotUpdate(true);
}

// updates every n milliseconds the GNUPlot windows
void guilogger::GNUPlotUpdate(bool waitfordata)
{   //framecounter++; 
    //int i = framecounter % plotwindows;
  if(!waitfordata || datacounter > datadelayrate){
    QMP_SHARE(gpWindowVisibility);
    QMP_SHARE(gp);
    QMP_PARALLEL_FOR(i,0,plotwindows){
      //    for(int i=0; i<plotwindows; i++) {
      QMP_USE_SHARED(gpWindowVisibility, bool*);
      if(gpWindowVisibility[i]){
	QMP_USE_SHARED(gp, Gnuplot<QString>*);
	gp[i].plot();
      }
    }
    QMP_END_PARALLEL_FOR;
    datacounter=0;
  }
}
