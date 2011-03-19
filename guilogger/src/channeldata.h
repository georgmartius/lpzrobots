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

typedef QString ChannelObjectName;
typedef QString ChannelName;
typedef QString ChannelDescr;

enum ChannelType { AutoDetection, Single, Matrix, Vector, 
			   VectorElement, MatrixElement };
/// information of a channel
typedef struct _ChannelInfo{
  ChannelName  name;
  ChannelDescr descr;
  ChannelObjectName objectName;
  ChannelType  type;
  int row;    // only valid for vectors and matrices
  int column; // only valid for vectors and matrices
} ChannelInfo;

/** definition hierarchical representation for Vector and Matrix elements 
    but also for normal elements (to support hierarchical representation)
 */
typedef struct _MultiChannel{
  ChannelInfo info; 
  int startindex; // start index in the data buffer
  int rows;       // number rows in the multichannel entry
  int columns;    // number of columns  in the multichannel entry
  int size;       // number of channels that belong to this multichannel
} MultiChannel;


typedef QVector<double> ChannelVals;

typedef QLinkedList<int> IndexList; 

typedef QVector<MultiChannel> MultiChannels;

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

  /// returns the multi channel index and (-1) if not found
  int getMultiChannelIndex(const ChannelName& name) const;

  /** sets the desription of a channel 
      (can be used before initialization and also for multichannels) 
  */
  void setChannelDescription(const ChannelObjectName& objectName, const ChannelName& name, const ChannelDescr& description);

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

  const QVector<ChannelVals>& getData() const { return data; }
  const QVector<ChannelInfo>& getInfos() const { return channels; }
  int getNumChannels() const { return numchannels; }
  int getNumMultiChannels() const { return multichannels.size(); }
    const MultiChannels& getMultiChannels() const { return multichannels; }
  int getTime() const { return time; } 

public slots:
  void receiveRawData(QString line);

protected:
  /// extracts a multichannel from the channels starting from position i (i is advanced)
  MultiChannel extractMultiChannel(int* i);  
  /// returns the name without the index specifiers e.g. for A[0][3] it returns A
  QString getChannelNameRoot(const ChannelName& name) const ;
signals:
  void quit();
  void channelsChanged();
  void update();
  void rootNameUpdate(QString name);
  
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
  /** list of multichannels like matrices and vectors, but also normal elements
      are referenced here to support hierarchical representation
  */
  MultiChannels multichannels;
  /// map from (multi-)channel-name to index
  QHash<ChannelName, int> multichannelindex;
  
  int buffersize; ///< size of ringbuffer

  /// map to store preset information and information about virtual channels (e.g. names of matrices as a whole)
  QHash<ChannelName, ChannelInfo> preset;
  
  int time; ///< index for ringbuffer
  bool initialized;

  ChannelName emptyChannelName; // empty string
};


#endif
