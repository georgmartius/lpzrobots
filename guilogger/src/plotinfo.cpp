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

#include "plotinfo.h"
#include "stl_adds.h"

PlotInfo::PlotInfo(const ChannelData& cd)
  : channelData(cd), isVisisble(true), reference1(-1), reference2(-1),
    initialized(false) {

}

void PlotInfo::setReference1(const ChannelName& ref1){
  if(!initialized)
    presetReference1 = ref1;
  else {
    int index = channelData.getChannelIndex(ref1);
    setReference1(index);
  }
}

void PlotInfo::setReference1(int ref1){
  reference1=ref1;
}

void PlotInfo::setReference2(const ChannelName& ref2){
  if(!initialized)
    presetReference2 = ref2;
  else {
    int index = channelData.getChannelIndex(ref2);
    setReference2(index);
  }
}

void PlotInfo::setReference2(int ref2){
  reference2=ref2;
}


const ChannelName& PlotInfo::getReference1Name() const {
  return channelData.getChannelName(reference1);
}

const ChannelName& PlotInfo::getReference2Name() const {
  return channelData.getChannelName(reference2);
}


void PlotInfo::setChannelShow(int index, bool on){
  if(index >=0 && index < channels.size()){
    if(on){
      // make sure it is in the list of visible channels
      if(!visiblechannels.contains(index)){
        // insertation sort
        bool inserted=false;
        FOREACH(QLinkedList<int>, visiblechannels, i){
          if(*i>index){
            visiblechannels.insert(i,index);
            inserted=true;
            break;
          }
        }
        if(!inserted) visiblechannels.push_back(index);
      }
    }else{
      // erase index from list
      QLinkedList<int>::iterator it = qFind(visiblechannels.begin(),
                                            visiblechannels.end(), index);
      if (it != visiblechannels.end()){
        visiblechannels.erase(it);
      }
    }
  }
}

void PlotInfo::setChannelShow(const ChannelName& name, bool on){
  if(!initialized){
    preset[name]=on;
  } else {
    int index = channelData.getChannelIndex(name);
    setChannelShow(index, on);
  }
}

void PlotInfo::setAllChannelShow(bool on){
  visiblechannels.clear();
  if(on){
    for(int i=0; i< channels.size(); i++){
      visiblechannels.push_back(i);
    }
  }
}

bool PlotInfo::getChannelShow(int index) const {
  return visiblechannels.contains(index);
}

void PlotInfo::setStyle(const ChannelName& channel, PlotStyle style){
  int index = channelData.getChannelIndex(channel);
  setStyle(index, style);
}
void PlotInfo::setStyle(int channel, PlotStyle style){
  if(channel >=0 && channel < channels.size()){
    channels[channel].style = style;
  }
};

void PlotInfo::setIsVisible(bool enable){
  isVisisble = enable;
}


void PlotInfo::channelsChanged(){
  if(!initialized){
    channels.resize(channelData.getNumChannels());
    setAllChannelShow(false);
    int i=0;
    initialized = true;
    FOREACHC(QVector<ChannelInfo>, channelData.getInfos(), c){
      channels[i] = ChannelPlotInfo(PS_DEFAULT);
      //
//                   QRegExp re;
//             re.setPattern(qv);
//             if(qv==(cr->getChannelName()) || re.exactMatch(cr->getChannelName()))
      if(preset.contains(c->name)){
        setChannelShow(c->name, preset[c->name]);
      }
      i++;
    }
    preset.clear();
    setReference1(presetReference1);
    setReference2(presetReference2);
  }
}
