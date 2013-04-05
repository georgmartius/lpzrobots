#ifndef __PIPE_IODEVICE_H
#define __PIPE_IODEVICE_H

#include <QFile>

class PipeIODevice : public QFile
  {
    public:
      PipeIODevice() : QFile() {};
      virtual ~PipeIODevice() {};

      virtual bool canReadLine() const {
        return QFile::buffer.contains('\n') || QIODevice::canReadLine();
      }

  };

#endif
