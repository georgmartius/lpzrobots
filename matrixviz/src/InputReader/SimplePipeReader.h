
#ifndef __SIMPLE_PIPE_READER_H__
#define __SIMPLE_PIPE_READER_H__

#include "AbstractPipeReader.h"
#include <QTextStream>
// #include <iostream>

class SimplePipeReader : public AbstractPipeReader
{
  Q_OBJECT
    
public:
    
  SimplePipeReader();
  virtual ~SimplePipeReader();
   
  virtual void run();
  
//   virtual void waitForChannelData();
  virtual bool readyForData();
  
protected:
  
  virtual std::list<std::string> getChannelLine();

  virtual std::list<std::string> getDescriptionLine();
  
  virtual std::list<double> getDataLine();
  
  virtual void resetChannelLine() { currentChannelLine=""; }

signals:
  void newData();
  
private:
  
//   PipeIODevice* input_file;
  QTextStream* input_line;//(stdin);
	QString currentDataLine;
  QString currentChannelLine;
  QString currentDescriptionLine;
  
//   QMutex* mutex;
  
};

#endif
