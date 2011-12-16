
#include "gnuplot.h"
#include "stl_adds.h"
#include <stdio.h>
#include <locale.h> // need to set LC_NUMERIC to have a '.' in the numbers piped to gnuplot

Gnuplot::Gnuplot(const PlotInfo* plotInfo)
  : plotInfo(plotInfo), pipe(0) {
};

Gnuplot::~Gnuplot(){
  close();
};

void Gnuplot::init(const QString& gnuplotcmd, int w,int h, int x, int y){
  open(gnuplotcmd, w, h, x, y);
}

bool Gnuplot::open(const QString& gnuplotcmd, int w,int h, int x, int y){
  char cmd[512];
  setlocale(LC_NUMERIC,"C"); // set us type output
//  setlocale(LC_NUMERIC,"en_US"); // set us type output
#if defined(WIN32) || defined(_WIN32) || defined (__WIN32) || defined(__WIN32__) \
        || defined (_WIN64) || defined(__CYGWIN__) || defined(__MINGW32__)
  sprintf(cmd, "%s", gnuplotcmd.latin1());
#else
  if(x==-1 || y==-1)
    sprintf(cmd, "%s -geometry %ix%i -noraise >/dev/null 2>/dev/null", gnuplotcmd.latin1(), w, h);
  else
    sprintf(cmd, "%s -geometry %ix%i+%i+%i -noraise >/dev/null 2>/dev/null", gnuplotcmd.latin1(), w, h, x, y);
#endif
  pipe=popen(cmd,"w");

  return true;
  //return popen("gnuplot -geometry 400x300","w");
  /*char b[100];
  sprintf(b,"gnup%i",rand()%1000);
  return fopen(b,"w");*/
}

void Gnuplot::close(){
  if(pipe)
    pclose(pipe);
  pipe=0;
}


/** send arbitrary command to gnuplot.
    like "set zeroaxis" or other stuff */
void Gnuplot::command(const QString& cmd){
  fprintf(pipe,"%s\n",cmd.latin1());
  fflush(pipe);
};


/** print buffer content to file pointer, usually a pipe */
void plotDataSet(FILE* f, const ChannelVals& vals){
  FOREACHC(ChannelVals, vals, v){
    fprintf(f,"%f ", *v);
  }
  fprintf(f,"\n");
};


/** make gnuplot plot selected content of data buffers */
QString Gnuplot::plotCmd(const QString& file, int start, int end){
  const QLinkedList<int>& vc = plotInfo->getVisibleChannels();
  if(vc.size()==0) return QString();  
  QStringList buffer;  
  bool first=true;
  const ChannelData& cd = plotInfo->getChannelData();  
  QString range;
  if(start!=-1 && end!=-1){
    range=QString(" every ::%1::%2 ").arg(start).arg(end); 
  }
  
  FOREACHC(QLinkedList<int>, vc, i){    
    if(first){
      buffer << "plot '" << (file.isEmpty() ? "-" : file)  << "' ";    
      first=false;
    } else {
      buffer << ", '' ";
    }
    buffer << range;
    if(!file.isEmpty()){
      if(plotInfo->getUseReference1()){    
        buffer << QString(" u %1:%2 ").arg(plotInfo->getReference1()+1).arg(*i+1);
      }else{
        // TODO add reference2!
        buffer << QString(" u %1 ").arg(*i+1);
      }
    }
    buffer << "t '" << cd.getInfos()[*i].name << "'";        
    if(!plotInfo->getChannelInfos()[*i].style != PS_DEFAULT) 
      buffer << " w " << plotInfo->getChannelInfos()[*i].getStyleString();    
  }
  return buffer.join(QString());  
}



/** make gnuplot plot selected content of data buffers */
void Gnuplot::plot(){
  // calculate real values for start and end
  if(!plotInfo || !plotInfo->getIsVisible()) return;

  // todo: use reference2 and plot3d?

  const ChannelData& cd      = plotInfo->getChannelData();
  const QLinkedList<int>& vc = plotInfo->getVisibleChannels();
  // FILE* pipe = stderr; // test
  if(vc.size()==0) return;
  fprintf(pipe, "%s\n", plotCmd().latin1());    
  
  if(plotInfo->getUseReference1()){    
    QLinkedList<int> visibles(vc);
    visibles.push_front(plotInfo->getReference1());        
    const QVector<ChannelVals>& vals = cd.getHistory(visibles, 0); // full history    
    int len = visibles.size();
    for(int k=1; k< len; k++){
      FOREACHC(QVector<ChannelVals>, vals, v){
        fprintf(pipe,"%f %f\n", (*v)[0], (*v)[k]);
      }
      fprintf(pipe,"e\n");
    }
  } else {
    const QVector<ChannelVals>& vals = cd.getHistory(vc, 0); // full history    
    int len = vc.size();
    for(int k=0; k< len; k++){
      FOREACHC(QVector<ChannelVals>, vals, v){
        fprintf(pipe,"%f\n", (*v)[k]);
      }
      fprintf(pipe,"e\n");
    }
  }
  fflush(pipe);
};    
