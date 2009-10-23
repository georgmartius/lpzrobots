/***************************************************************************
 *   Copyright (C) 2009 by Georg Martius and Dominic Schneider   *
 *   georg.martius@web.de   *
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

#include "stl_adds.h"

#include <utility>
#include <qthread.h>
#include <qapplication.h>
#include <qmenubar.h>
#include <qtimer.h>
#include <qregexp.h>
#include <qscrollarea.h>

#include "quickmp.h"


GuiLogger::GuiLogger(const CommLineParser& configobj, const QRect& screenSize) 
  : QMainWindow( 0), screenSize(screenSize), channelData(0) {
  setWindowTitle("GuiLogger");
  mode     = configobj.getMode();
  filename = configobj.getFile();

  connect(&channelData, SIGNAL(quit()), this, SLOT(doQuit()));


  load();  // load Config File

  lastPlotTime = 0;
  int linecount = 0;
  const int maxplottimer = 5000;
  filePlotHorizon = 500;    

  if(mode == "file" && !filename.isEmpty()) linecount = analyzeFile();

  setCentralWidget(new QWidget(this));
  layout           = new QVBoxLayout(centralWidget());
  channelandslider = new QWidget(centralWidget());
  QSizePolicy sp1(QSizePolicy::Preferred, QSizePolicy::Preferred);
  sp1.setHorizontalStretch(0);
  sp1.setVerticalStretch(10);
  channelandslider->setSizePolicy(sp1);
    


  layout->addWidget(channelandslider);    
  channelandsliderlayout = new QHBoxLayout(channelandslider);

  sv = new QScrollArea(centralWidget());
  channelandsliderlayout->addWidget(sv);
  //    sv = new Q3ScrollView(centralWidget());
  // sv->setResizePolicy(Q3ScrollView::AutoOneFit);    
  //    channelWidget = new QWidget(sv->viewport());
  //  channelWidget = new QWidget(channelandslider);
  
  // create Model for channel - plotwindow association
  tableModel = new PlotChannelsTableModel(&plotInfos, channelandslider);
  connect(&channelData, SIGNAL(update()), tableModel, SLOT(update()));
  connect(tableModel, SIGNAL(updateWindow(int)), this, SLOT(plotChannelsChanged(int)));
  // create View for channel - plotwindow association
  channelWidget = new QTableView(channelandslider);
  channelWidget->setModel(tableModel);
  channelWidget->resizeColumnsToContents();
  channelWidget->setSelectionBehavior(QAbstractItemView::SelectItems);
  channelWidget->setSelectionMode(QAbstractItemView::SingleSelection);

  // I tried to get a tree view working but it took too much time. see descarded folder
//   // create Model for channel - plotwindow association
//   treeModel = new PlotChannelsTreeModel(&plotInfos, channelandslider);
//   connect(&channelData, SIGNAL(update()), treeModel, SLOT(update()));
//   connect(treeModel, SIGNAL(updateWindow(int)), this, SLOT(plotChannelsChanged(int)));
//   // create View for channel - plotwindow association
//   channelTreeWidget = new QTreeView(channelandslider);
//   channelTreeWidget->setModel(treeModel);
//   //channelTreeWidget->resizeColumnsToContents();
//   channelTreeWidget->setSelectionBehavior(QAbstractItemView::SelectItems);
//   channelTreeWidget->setSelectionMode(QAbstractItemView::SingleSelection);
//   channelandsliderlayout->addWidget(channelTreeWidget);

  // // create editor for reference
//   QItemEditorFactory *editorFactory = new QItemEditorFactory;
//   QItemEditorCreatorBase *creator = new QStandardItemEditorCreator<QCombobox>();
//   editorFactory->registerEditor(QVariant::String, creator);  

  sv->setWidget(channelWidget);
  sv->setWidgetResizable(true);
 
  //  channelandsliderlayout->addWidget(channelWidget);

  // sv->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred,2,0, FALSE));
  channelWidget->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));

  commWidget = new QWidget(centralWidget()); 
  QSizePolicy sp2(QSizePolicy::Preferred, QSizePolicy::Minimum);
  sp2.setHorizontalStretch(1);
  sp2.setVerticalStretch(1);  
  commWidget ->setSizePolicy(sp2);

  layout->addWidget(commWidget);

  QWidget *horizonsliderwidget = new QWidget(channelandslider);
  channelandsliderlayout->addWidget(horizonsliderwidget);

  QWidget *datasliderwidget;
  if(mode== "file"){
    datasliderwidget    = new QWidget(channelandslider);
    channelandsliderlayout->addWidget(datasliderwidget);
  }

  channellayout = new QVBoxLayout(channelWidget);
  commlayout    = new QHBoxLayout(commWidget);


  QVBoxLayout *horizonsliderlayout = new QVBoxLayout(horizonsliderwidget);
  horizonslider = new QSlider(Qt::Vertical, horizonsliderwidget);
  horizonslidervalue = new QLabel(horizonsliderwidget);

  if(mode == "file") { 
    horizonslider->setInvertedAppearance(true);
    horizonslider->setMaximum(linecount);
    horizonslider->setValue(filePlotHorizon);
    horizonsliderlayout->addWidget(new QLabel("Win", horizonsliderwidget));
    horizonslidervalue->setText(QString::number(filePlotHorizon, 10));
  } else { 
    horizonslider->setMinimum(10);
    horizonslider->setMaximum(maxplottimer);   // MaxTimer für Plottimer
    horizonslider->setValue(startplottimer);       // actual Value for Plottimer
    horizonsliderlayout->addWidget(new QLabel("Interval", horizonsliderwidget));
    horizonslidervalue->setText(QString("%1\nms").arg(startplottimer));
  }
  horizonsliderlayout->addWidget(horizonslider);
  horizonsliderlayout->addWidget(horizonslidervalue);
  connect(horizonslider, SIGNAL(valueChanged(int )), this, SLOT(horizonSliderValueChanged(int )));
  connect(horizonslider, SIGNAL(sliderReleased())  , this, SLOT(horizonSliderReleased()));
    
  if(mode=="file"){
    QBoxLayout *datasliderlayout = new QVBoxLayout(datasliderwidget);
    dataslider = new QSlider(Qt::Vertical, datasliderwidget);
    dataslider->setInvertedAppearance(true);
    dataslidervalue = new QLabel(datasliderwidget);
    dataslider->setMinimum(0);
    dataslider->setMaximum(linecount);
    dataslidervalue->setText(QString::number(0, 10));
    datasliderlayout->addWidget(new QLabel("Data", datasliderwidget));
    datasliderlayout->addWidget(dataslider);
    datasliderlayout->addWidget(dataslidervalue);
    connect(dataslider, SIGNAL(valueChanged(int )), this, SLOT(dataSliderValueChanged(int )));
    connect(dataslider, SIGNAL(sliderReleased())  , this, SLOT(dataSliderReleased()));
  }

  parameterlistbox   = new QTextEdit(commWidget);
  parameterlistbox->setMaximumHeight(60);
  parameterlistbox->setLineWrapMode(QTextEdit::NoWrap);
  
  paramvaluelineedit = new QLineEdit(commWidget);
  sendbutton         = new QPushButton("Send Cmd",commWidget);
  connect(sendbutton, SIGNAL(released())  , this, SLOT(sendButtonPressed()));

  //  parameterlistbox->setTextFormat(Qt::LogText);
    
  sendlayout = new QVBoxLayout();
  commlayout->addLayout(sendlayout);
  sendlayout->addWidget(paramvaluelineedit);
  sendlayout->addWidget(sendbutton);

  commlayout->addWidget(parameterlistbox);
    
  filemenu = new QMenu("&Menu", this); 
  menuBar()->addMenu(filemenu);
  filemenu->addAction("&Save Config", this, SLOT(save()));
  filemenu->addAction("&Edit Config", this, SLOT(editconfig()));
  filemenu->addAction("&Load Config", this, SLOT(load()));
  filemenu->addAction("&Exit", this, SLOT(doQuit()));

  
  plottimer = new QTimer( this);
  if(mode == "file"){
    resize( 480, 600 );
    updateSliderPlot();
  }else{  
    resize( 800, 600 );
    connect(plottimer, SIGNAL(timeout()), SLOT(plotUpdate()));
    plottimer->setSingleShot(false);
    plottimer->start(startplottimer);
  }
  
}


GuiLogger::~GuiLogger() {   
  FOREACH(QVector<PlotInfo*>,plotInfos, p){
    delete *p;
  }
}

void GuiLogger::horizonSliderReleased() {
  updateSliderPlot();
  if(mode != "file") // in this case this slider shows the time
    plottimer->setInterval(filePlotHorizon);  // change Plotintervall
}

void GuiLogger::dataSliderValueChanged(int value) {
  dataslidervalue->setText(QString::number(value, 10));
}

void GuiLogger::horizonSliderValueChanged(int value) {
  if(mode=="file"){
    horizonslidervalue->setText(QString::number(value, 10));
  } else {
    horizonslidervalue->setText(QString("%1\nms").arg(value));
  }
  filePlotHorizon = value;
}



void GuiLogger::sendButtonPressed() {
  //   // send command to gnuplot
  for(int i=0; i<plotwindows; i++) 
    plotWindows[i].command(paramvaluelineedit->text());   
}

void GuiLogger::dataSliderReleased() {    
  updateSliderPlot();
}


void GuiLogger::updateSliderPlot() {  
  if(mode!="file") return;
  int start = dataslider->value();
    
  parameterlistbox->clear();
  parameterlistbox->append("set style data lines");
  parameterlistbox->append("set zeroaxis");

  
  for(int i=0; i<plotwindows; i++){   
    QString cmd = plotWindows[i].plotCmd(filename,start,start+filePlotHorizon);
    if(!cmd.isEmpty()){
      plotWindows[i].command(cmd);
      parameterlistbox->append(cmd);    
    }
  }
}


/// analyzes the file, send channels and return number of lines with data
int GuiLogger::analyzeFile() {
  char *s=NULL;
  int buffersize=0; 
  char c;
  int size=1, i=1;
  int linecount=0;

  FILE *instream;
  //         printf("Counting Lines...   ");

  instream = fopen(filename.toLatin1(), "r");
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
  printf("%s",s);

  channelData.receiveRawData(QString(s));

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


void GuiLogger::save(){
  save(false);
}

/** saves the channel configuration to file
    if blank is used then a basic file is written without the window stuff
*/
void GuiLogger::save(bool blank){
  QString nr;
  IniSection *section;

  cfgFile.setFilename("guilogger.cfg");

  section = cfgFile.sections.first();  // delete all "window" sections, because they will be rewritten in the next "for loop".
  while(section)     {   
    if(section->getName() == "Window") { 
      cfgFile.sections.remove();  // remove current item, iterator++ 
      section = cfgFile.sections.current();
    } else if(blank && (section->getName() == "General" || section->getName() == "GNUPlot" || 
		   section->getName() == "Misc")) { 
      cfgFile.sections.remove();  // remove current item, iterator++ 
      section = cfgFile.sections.current();
    } else {
      section = cfgFile.sections.next();
    }
  }

  // If we don't have a General section then add it.    
  IniSection sec;
  if(!cfgFile.getSection(sec,"General",false)){
    cfgFile.setComment("# Please restart guilogger to apply changes you make in the file");
    section=cfgFile.addSection("General");
    section->addValue("Version",VERSIONSTRING, " # do not change!");
    section->addValue("PlotWindows","5");
    section->addValue("CalcPositions","yes"," # If yes then the gnuplot windows will be placed according to the layout");
    section->addValue("WindowLayout","blh"," # From where to start: t: top, b: bottom, l: left, r: right, h: horizontal, v: vertical");
    section->addValue("WindowsPerRowColumn","3");
    
    section->addValue("UpdateInterval","2000"," # time between plotting updates in ms");
    section->addValue("MinData4Replot","1"," # number of input events before updating plots");
    section->addValue("BufferSize","250", " # Size of history");
  }
  // If we don't have a GNUPlot section then add it.    
  if(!cfgFile.getSection(sec,"GNUPlot",false)){
    section=cfgFile.addSection("GNUPlot");
    section->addValue("Command","set terminal x11");
    section->addValue("Command","set style data lines");
    section->addValue("Command","set zeroaxis");
  }    

  if(1){ // !blank
    for(int i=0; i<plotwindows; i++){   	  
      nr = QString::number(i, 10);
	
      IniSection *sec = cfgFile.addSection("Window");
      sec->addComment("# you can also set the size and the position of a window. The channels can also contain wildcards like x* or C[0]*");
      sec->addValue("Number", nr);
      if(!plotInfos[i]->getIsVisible()){
	sec->addValue("Disabled", "yes");
      }
      QString ref = channelData.getChannelName(plotInfos[i]->getReference1());
      if(!ref.isEmpty()){
        sec->addValue("Reference1", ref);
      }	  	
      ref = channelData.getChannelName(plotInfos[i]->getReference2());
      if(!ref.isEmpty()){
        sec->addValue("Reference2", ref);
      }	  	
      if(windowsize.contains(i)){
        QSize s = windowsize[i];
        sec->addValue("Size",QString("%1x%2").arg(s.width()).arg(s.height()));
      } else 
        sec->addValue("Size", "400x300");
	
      if(windowposition.contains(i)){
        QSize pos = windowposition[i];
        sec->addValue("Position",QString("%1 %2").arg(pos.width()).arg(pos.height())," # set to \"-1 -1\" to make it automatically set by your windowmanager. If calcPositions is used then this is ignored");
      }else{
        sec->addValue("Position", "-1 -1"," # set to any coordinate to place a window by hand, (-1 -1) means automatic. If calcPositions then this is ignored");	
      }  
      FOREACHC(QLinkedList<int>, plotInfos[i]->getVisibleChannels(), c){        
        sec->addValue("Channel", channelData.getChannelName(*c));
        // todo maybe add style as well
      }
    }      
  }      
  cfgFile.Save();
}


/// loads the channel configuration from file
void GuiLogger::load() {
  int pwin;
  QString qv;
  //    re.setWildcard(TRUE);

  IniSection* section;
  IniVar* var;

  pwin = -1;

  cfgFile.setFilename("guilogger.cfg");
  if(!cfgFile.Load()){
    printf("Guilogger: Configuration file does not exist try to create it.\n");
    save(true); // this automatically creates the config (on disk and in memory)
  }

  // load general settings
  if(cfgFile.getValueDef("General","Version","").trimmed() != VERSIONSTRING) {
    printf("Guilogger: Configuration file has incorrect version, I create a new one.\n");
    save(true);// this automatically creates the config (on disk and in memory)
  }
  plotwindows = cfgFile.getValueDef("General","PlotWindows","5").toInt();  
  startplottimer = cfgFile.getValueDef("General","UpdateInterval","2000").toInt();  
  datadelayrate = cfgFile.getValueDef("General","MinData4Replot","1").toInt();  
  
  if(plotInfos.size()<plotwindows){    
    // create plotinfos
    for(int i=plotInfos.size(); i<plotwindows; i++){
      PlotInfo* pi = new PlotInfo(channelData);
      connect(&channelData, SIGNAL(channelsChanged()), pi, SLOT(channelsChanged()));
      plotInfos.push_back(pi);    
    }
  }else{ // delete some plotwindows
    int old = plotInfos.size();
    for(int i = plotwindows; i < old; i++){
      plotInfos.pop_back();
    }
  }

  // load window settings
  for(section = cfgFile.sections.first(); section != 0; section = cfgFile.sections.next()) {   
    if(section->getName() == "Window"){
      pwin=0;
      for(var = section->vars.first(); var!=0; var = section->vars.next())
        {   qv = var->getValue();          
          if(var->getName() == "Number") {
            pwin = qv.toInt();
            if(plotInfos.size()<=pwin){
              fprintf(stderr, "we don't have so many windows: %i\n", pwin);
              goto finish;
            }
          } else if(var->getName() == "Disabled") {
            plotInfos[pwin]->setIsVisible(qv.trimmed() != "yes");
          } else if(var->getName() == "Reference1") {
            plotInfos[pwin]->setReference1(qv);
          } else if(var->getName() == "Reference2") {
            plotInfos[pwin]->setReference2(qv);
          } else if(var->getName() == "Channel") {       
	    plotInfos[pwin]->setChannelShow(qv,true);
	  } else if(var->getName() == "Size") {
            int x,y;
            if(sscanf(qv.latin1(),"%ix%i",&x,&y)==2)
              windowsize.insert(pwin, QSize(x,y));
          } else if(var->getName() == "Position") {
            int w,h;
            if(sscanf(qv.latin1(),"%i %i",&w,&h)==2)
              windowposition.insert(pwin, QSize(w,h));
          }	       
        }

    }
  }
 finish:

  // load and calculate positioning
  QString calcPositions = cfgFile.getValueDef("General","CalcPositions","yes");
  QString windowLayout = cfgFile.getValueDef("General","WindowLayout","blh");
  int windowsPerRowColumn = cfgFile.getValueDef("General","WindowsPerRowColumn","3").toInt();
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

  // close all plotwindows
  plotWindows.clear();
    
  // create plotting windows
  for(int i=0; i<plotwindows; i++){
    plotWindows.push_back(Gnuplot(plotInfos[i])); // TODO load visiblity from cfgfile
  }

  //open and position plotwindows
  //  serial version    
  for(int k=0; k<plotwindows; k++) {
    QSize s = windowsize.contains(k) ? windowsize[k] : QSize(400,300);
    if(windowposition.contains(k)){
      QSize pos = windowposition[k];
      plotWindows[k].init(s.width(), s.height(), pos.width(), pos.height());	
    }else{	
      plotWindows[k].init(s.width(), s.height());
    }
  }

  // send gnuplot commands
  IniSection GNUplotsection;    
  if(cfgFile.getSection(GNUplotsection,"GNUPlot",false)){      
    IniVar* var;
    QString qv;
    for(var = GNUplotsection.vars.first(); var!=0; var = GNUplotsection.vars.next()) {   
      qv = var->getValue();
      if(var->getName() == "Command") 
        for(int k=0; k<plotwindows; k++) 
          plotWindows[k].command(qv.latin1());
    } 
  }

  channelData.setBufferSize(cfgFile.getValueDef("General","BufferSize","250").toInt());  
  printf("Guilogger: Config file loaded.\n");
}


void GuiLogger::editconfig(){
  system("$EDITOR ./guilogger.cfg");

}


void GuiLogger::doQuit(){
  emit quit();
}

void GuiLogger::plotChannelsChanged(int window){
  if(mode=="file") 
    updateSliderPlot();
  else
    plotUpdate(false, window);
}

// updates every n milliseconds the GNUPlot windows
void GuiLogger::plotUpdate(){
  plotUpdate(true);
}

void GuiLogger::plotUpdate(bool waitfordata, int window)
{   
  if(mode=="file") return;
  //  fprintf(stderr,"udpaten, %i, %i\n", waitfordata , channelData.getTime());
  if(!waitfordata || channelData.getTime() - lastPlotTime > datadelayrate){
    // serial version    
    if(window==-1){
//       // serial version
//       for(int i=0; i<plotwindows; i++) {
//         plotWindows[i].plot();
//       }
    // Parallel Version
      PlotWindows* pw = &plotWindows;
      QMP_SHARE(pw);
      QMP_PARALLEL_FOR(i,0,plotwindows){
	QMP_USE_SHARED(pw, PlotWindows*);
  	(*pw)[i].plot();
      }      
      QMP_END_PARALLEL_FOR;

    } else {
      if(window >=0 && window<plotwindows)
        plotWindows[window].plot();
    }
    if(waitfordata) lastPlotTime=channelData.getTime();

  }
}
