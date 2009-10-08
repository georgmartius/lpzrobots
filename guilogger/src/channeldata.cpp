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

#include "channeldata.h"
#include "stl_adds.h"

ChannelData::ChannelData(int buffersize)
  : numchannels(0), buffersize(0), time(0), initialized(false){
  setBufferSize(buffersize);

}
  
int ChannelData::getBuffersize(){
  return buffersize;
}

void ChannelData::setBufferSize(int newBuffersize){
  buffersize=newBuffersize;
  if(initialized){
    data.resize(buffersize);
    for(int i=0; i<buffersize;i++){
      data[i].resize(numchannels);
    }
    time = 0; 
    emit(update());
  }
}
 

/// sets a new information about a channel (also works before initialization)
void ChannelData::setChannelInfo(const ChannelInfo& info){
  if(!initialized){
    preset[info.name]=info;
  } else {
    int index = getChannelIndex(info.name);
    if(index>=0 && index<channels.size())
      channels[index]=info;
    else // if not in known channels then add it to the preset info (not valuable at the moment)
      preset[info.name]=info;
  }  
}

// sets the channels (and initialized the data buffer)
void ChannelData::setChannels(const QStringList& newchannels){
  // if we are allready initialized we do nothing ( not really supported)
  if(initialized){
    if(newchannels.size() != channels.size()){ // this is not allowed so far
      fprintf(stderr,"It is not allowed to reinitialize with new number of channels");      
    }
    return; 
  }else{
    numchannels = newchannels.size();
    channels.resize(numchannels);    
    int i=0;
    QRegExp vectorRE;
    vectorRE.setPatternSyntax(QRegExp::RegExp);
    vectorRE.setPattern("^.+\\[\\d+\\]$"); // regexp for a vector (e.g. v[0])
    QRegExp matrixRE;
    matrixRE.setPatternSyntax(QRegExp::RegExp);
    matrixRE.setPattern("^.+\\[\\d+\\]\\[\\d+\\]$"); // regexp for a matrix (e.g. A[0][2])
    FOREACHC(QStringList, newchannels, n){
      if(preset.contains(*n)){
	channels[i] = preset[*n];
      } else {
	channels[i].name  = *n;
	channels[i].descr = ""; // TODO load descr
	channels[i].type  = AutoDetection;   
      }
      if(channels[i].type == AutoDetection){
	if(vectorRE.exactMatch(*n)){
	  channels[i].type  = VectorElement; 
	} else if(matrixRE.exactMatch(*n)){
	  channels[i].type  = MatrixElement; 
	} else {
	  channels[i].type  = Single;
	}
      }

      channelindex[*n] = i;
      i++;
    }
    // Todo: fill multichannel info
    
    // initialize data buffer
    data.resize(buffersize);
    for(int i=0; i<buffersize;i++){
      data[i].resize(numchannels);
    }    
    initialized = true;
    emit channelsChanged();    
    emit update();
  }
}

/// returns the channel index (-1) if not found
int ChannelData::getChannelIndex(const ChannelName& name) const{
  if(channelindex.contains(name))
    return channelindex[name];
  else
    return -1;
}

const ChannelName& ChannelData::getChannelName(int index) const {
  if(index>=0 && index<channels.size()){
    return channels[index].name;
  }else{
    return emptyChannelName;
  }
  
}


// inserts a new set of data into the ring buffer
void ChannelData::setData(const QVector<double>& newdata){
  if(newdata.size() != numchannels) {
    fprintf(stderr,"Number of data entries (%i) does not match number of channels (%i)",
	    newdata.size(),numchannels);   
  }else{
    time++;
    int index = time%buffersize;
    data[index] = newdata;
  }
}

/* returns the data of the given channels starting from history entries in the past.
    if history=0 then the entire history is given
*/
QVector<ChannelVals> ChannelData::getHistory(const IndexList& channels, int history) const {
  QVector<ChannelVals> rv;
  int start = history ==0 ? time-(buffersize-1) : time-history;
  start = start < 0 ? 0 : start; // not below zero
  rv.resize(time-start);
  for(int i=start; i<time; i++){
    rv[i-start] = getData(channels,i%buffersize);
  }
  return rv;
}

/* returns the data of the given channels starting from history entries in the past.
    if history=0 then the entire history is given
*/
QVector<ChannelVals> ChannelData::getHistory(const QList<ChannelName>& channels, int history) const {
  QVector<ChannelVals> rv;
  int start = history ==0 ? time-buffersize : time-history;
  start = start < 0 ? 0 : start; // not below zero
  rv.resize(time-start);
  for(int i=start; i<time; i++){
    rv[i-start] = getData(channels,i%buffersize);
  }
  return rv;
}

// returns the data of the given channel at the given index
ChannelVals ChannelData::getData(const IndexList& channels, int index) const {
  ChannelVals rv(channels.size());
  int i=0;
  FOREACHC(IndexList, channels, c){
    rv[i] = data[index][*c];
    i++;
  }
  return rv;
}

// returns the data of the given channel at the given index
ChannelVals ChannelData::getData(const QList<ChannelName>& channels, int index) const {
  ChannelVals rv(channels.size());
  int i=0;
  FOREACHC(QList<ChannelName>, channels, c){
    int k = channelindex[*c];
    rv[i] = data[index][k];
    i++;
  }
  return rv;
}


void ChannelData::receiveRawData(QString data){  
  QStringList parsedString = QStringList::split(' ', data);  //parse data string with Space as separator
  QString& first = *(parsedString.begin());
  if(first == "#C")   //Channels einlesen
    {	
      // printf("GuiLogger:: got channels!\n");
      parsedString.erase(parsedString.begin());
      //transmit channels to GNUPlot
      QStringList channels;
      FOREACH(QStringList, parsedString, s){
	channels.push_back((*s).stripWhiteSpace());
      }
      setChannels(channels);
    }
  else if(first.length()>=2 &&  first[0] == '#' && first[1] == 'Q')   //Quit
    {
      printf("Guilogger: Received Quit\n");
      emit quit();
    }	
  else if( first[0] != '#')  // Daten einlesen
    {
      QVector<double> dat(parsedString.size());
      int i=0;
      
      FOREACH(QStringList, parsedString, s){
	dat[i] = (*s).stripWhiteSpace().toFloat();
        i++;
      }
      setData(dat);
    }
}

