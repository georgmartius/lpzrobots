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
#ifndef __CHANNELDATA_H
#define __CHANNELDATA_H

#include <QObject>
#include <QString>
#include <QVector>
#include <QLinkedList>
#include <QList>
#include <QStringList>
#include <QHash>

// definition for Vector and Matrix elements
typedef struct _MultiChannel{
  int startindex; // start index in the data buffer
  int size;       //  number of elements in the multichannel entry
  bool ismatrix;  // true if matrix, otherwise vector
  int columns;    // only valid of matrices
} MultiChannel;

typedef QString ChannelName;
typedef QString ChannelDescr;

typedef enum ChannelType { AutoDetection, Single, VectorElement, MatrixElement };
typedef struct _ChannelInfo{
  ChannelName  name;
  ChannelDescr descr;
  ChannelType  type;
} ChannelInfo;

typedef QVector<double> ChannelVals;

typedef QLinkedList<int> IndexList; 

typedef QHash <ChannelName, MultiChannel> MultiChannelMap;

class ChannelData : public QObject{
  Q_OBJECT
public:  

  ChannelData(int buffersize);
  
  int getBuffersize();
  void setBufferSize(int newBuffersize);
    
  /// sets a new information about a channel (also works before initialization)
  void setChannelInfo(const ChannelInfo& info);

  /// sets the channels (and initialized the data buffer)
  void setChannels(const QStringList& channels);

  /// returns the list of channels
  const QVector<ChannelName >& getChannels() const;

  /// returns the channel index and (-1) if not found
  int getChannelIndex(const ChannelName& name) const;

  /// returns the name of the channel with the given index (empty string for out of bounds)
  const ChannelName& getChannelName(int index) const;

  /// inserts a new set of data into the ring buffer
  void setData(const QVector<double>& data); 

  /// returns the data of the given channel at the given index
  ChannelVals getData(const IndexList& channels, int i) const;

  /// returns the data of the given channel at the given index
  ChannelVals getData(const QList<ChannelName>& channels, int i) const;

  /** returns the data of the given channels starting from history entries in the past.
      if history=0 then the entire history is given
   */
  QVector<ChannelVals> getHistory(const IndexList& channels, int history = 0) const;
  /** returns the data of the given channels starting from history entries in the past.
      if history=0 then the entire history is given
   */
  QVector<ChannelVals> getHistory(const QList<ChannelName>& channels, int history = 0) const;

  const QVector<ChannelVals > getData() const { return data; }
  const QVector<ChannelInfo > getInfos() const { return channels; }
  int getNumChannels() const { return numchannels; }
  const MultiChannelMap  getMultichannels() const { return multichannels; }
  int getTime() const { return time; } 

public slots:
  void receiveRawData(QString line);

protected:
  

signals:
  void quit();
  void channelsChanged();
  void update();
  
private:
  /** first array is the ring buffer
      second array if channels */
  QVector<ChannelVals>  data;
  /// names of channels
  QVector<ChannelInfo> channels;
  /// number of channels
  int numchannels;
  /// map from channel-names to index
  QHash<ChannelName, int> channelindex;
  /// multichannel types like matrices and vectors are referenced here
  MultiChannelMap multichannels;
  
  int buffersize; ///< size of ringbuffer

  /// map to store preset information
  QHash<QString, ChannelInfo> preset;

  
  int time; ///< index for ringbuffer
  bool initialized;

  ChannelName emptyChannelName; // empty string
};


#endif
