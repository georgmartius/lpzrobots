/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Dominic Schneider (original author)                                  *
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
#ifndef __PLOTINFO_H
#define __PLOTINFO_H

#include <QObject>
#include <QString>
#include <QVector>
#include <QList>
#include <QLinkedList>
#include <QStringList>
#include <QHash>

#include "channeldata.h"

enum PlotStyle {PS_DEFAULT, PS_LINES, PS_POINTS};

// information for a single channel for one gnuplot window
class ChannelPlotInfo{
public:
  ChannelPlotInfo() {};
  ChannelPlotInfo(PlotStyle style)
    : style(style) {};
  PlotStyle style;
  const char* getStyleString() const {
    switch(style){
    case PS_LINES:   return "l";
    case PS_POINTS:  return "p";
    default: return "l";
    }
  }
};

class PlotInfo : public QObject{
  Q_OBJECT
public:
  PlotInfo(const ChannelData& cd);

  /// sets whether the channel is to be shown
  void setChannelShow(int index, bool on);
  /// sets whether the channel is to be shown (this works also before initialization)
  void setChannelShow(const QString& name, bool on);
  void setAllChannelShow(bool on);

  bool getChannelShow(int index) const;
  bool getChannelShow(const QString& name) const;

  /// if empty then no reference is used
  void setReference1(const ChannelName& ref1);
  /// if -1 then no reference is used
  void setReference1(int ref1);
  int  getReference1() const { return reference1; }
  const ChannelName& getReference1Name() const;
  // if empty then no reference is used
  void setReference2(const ChannelName& ref2);
  // if -1 then no reference is used
  void setReference2(int ref2);
  int  getReference2() const { return reference2; }
  const ChannelName& getReference2Name() const;
  bool getUseReference1() const { return reference1 != -1; }
  bool getUseReference2() const { return reference2 != -1; }

  void setStyle(const ChannelName& channel, PlotStyle style);
  void setStyle(int channel, PlotStyle style);

  void setIsVisible(bool enable);
  bool getIsVisible() const { return isVisisble; }

  const ChannelData& getChannelData() const { return channelData; }
  const QLinkedList<int> getVisibleChannels() const { return visiblechannels; }
  const QVector<ChannelPlotInfo >& getChannelInfos() const { return channels; }


public slots:
  // information that channels are changed (this initializes the structure)
  void channelsChanged();

private:
  /// reference to channel data
  const ChannelData& channelData;

  /** list of channel information */
  QVector<ChannelPlotInfo > channels;
  ///< indexes of visible channels (this makes the display quick and the selection slow)
  QLinkedList<int> visiblechannels; // TODO: convert to a map
  bool isVisisble;

  int reference1;
  int reference2;

  bool initialized;

  /// map to store preset information
  QHash<QString, bool> preset;
  QString presetReference1;
  QString presetReference2;
};


#endif
