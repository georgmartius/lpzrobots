
#ifndef __SIMPLE_PIPE_READER_H__
#define __SIMPLE_PIPE_READER_H__

#include "AbstractPipeReader.h"
#include <QTextStream>
// #include <iostream>

class SimplePipeReader : public AbstractPipeReader
{
  Q_OBJECT

public:

  SimplePipeReader(bool noVideo);
  virtual ~SimplePipeReader();

  virtual void run();

//   virtual void waitForChannelData();
  virtual bool readyForData();

  virtual void goReadData() { waitForGui = false;};
  virtual void waitUntilGo() { waitForGui = true;};
protected:

  virtual std::list<std::string> getChannelLine();

  virtual std::list<std::string> getDescriptionLine();

  virtual std::list<double> getDataLine();

  virtual void resetChannelLine() { currentChannelLine=""; }

signals:
  void newData();
  void captureFrame(long index, QString directory);
  void sourceName(QString name);

private:

//   PipeIODevice* input_file;
  QTextStream* input_line;//(stdin);
        QString currentDataLine;
  QString currentChannelLine;
  QString currentDescriptionLine;
  static const bool debug = false;
  bool waitForGui;
  bool noVideo;

//   QMutex* mutex;

};

#endif
