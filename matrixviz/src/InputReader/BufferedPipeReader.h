/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Antonia Siegert (original author)                                  *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 *                                                                         *
 ***************************************************************************/

#ifndef _BUFFERED_PIPE_READER__H
#define _BUFFERED_PIPE_READER__H

#include <QFile>
#include <QString>
#include <QLinkedList>
#include <QMutex>
#include <QObject>
#include <iostream>
#include <QMutexLocker>

#include "AbstractPipeReader.h"


class BufferedPipeReader : public AbstractPipeReader
{
  
  Q_OBJECT
  
public:
  
  BufferedPipeReader(AbstractPipeReader* apr);
  virtual ~BufferedPipeReader();
//   virtual void waitForChannelData();
  virtual bool readyForData() {return true;};
  
  void run();
  
public slots:
  void newIncomingData();
  
signals:
  void newData();
  
protected:
  
  /**
  * return the channelLine, that is hold by the given AbstractPipeReader
  * (ex. SimplePipeReader)
  */
  virtual std::list<std::string> getChannelLine();
 
  /**
  * return the first data of the buffer
  */
  virtual std::list<double> getDataLine();
  
private:
  AbstractPipeReader* apr;
  typedef std::list<double> buffer_t;
  QLinkedList<buffer_t> input_buffer;
  QMutex mutex;
//   QLinkedList<std::string> *channelList;
};


#endif

