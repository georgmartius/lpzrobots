
#ifndef __ABSTRACT_PIPE_READER_H__
#define __ABSTRACT_PIPE_READER_H__

#include <list>
#include <string>
#include <QFile>
#include <iostream>
#include <QThread>
#include <QMutexLocker>
#include <QMutex>
#include <QStringList>
#include <QObject>

class AbstractPipeReader : public QThread
{
//   Q_OBJECT

public:
//   AbstractPipeReader() = 0;
//   virtual ~AbstractPipeReader() {};

  virtual void run()=0;

  virtual std::list<std::string> getChannelLine() = 0;


  virtual std::list<std::string> getDescriptionLine() = 0;

  virtual std::list<double> getDataLine() = 0;

//   virtual void waitForChannelData() = 0;
  virtual bool readyForData() = 0;
  // the thread should go ahead with reading data. (allowing the gui to fire up)
  virtual void goReadData() = 0;
  // causes the reader to wait before reading more data before goReadData is called
  virtual void waitUntilGo() = 0;

// signals:
//   void newData() {};
//
protected:

private:


};

#endif
