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
#include "GroupPlotChannel.h"
#include "tools/stl_adds.h"
#include <cassert>
#include <iostream>

using namespace std;

GroupPlotChannel::GroupPlotChannel(string name) : AbstractPlotChannel(name) {
  // TODO Auto-generated constructor stub

}

GroupPlotChannel::~GroupPlotChannel() {
  // TODO Auto-generated destructor stub
}

void GroupPlotChannel::addPlotChannel(AbstractPlotChannel* channelToAdd) {
  this->channelsOfGroup.push_back(channelToAdd);
}

list<double> GroupPlotChannel::getValues() {
  list<double> valueList;
  FOREACHC(list<AbstractPlotChannel*>, channelsOfGroup, channelIt) {
    //list<double> valueList;
    valueList.push_back((*channelIt)->getValue());
    //return valueList;
  }
  return valueList;
}

list<AbstractPlotChannel*> GroupPlotChannel::getChannelsOfGroup() {
  return channelsOfGroup;
}

AbstractPlotChannel* GroupPlotChannel::at(int pos){
  if(pos>=channelsOfGroup.size()){
    std::cerr << " error: GroupPlotChannel::at " << pos << "/" << channelsOfGroup.size() << std::endl;  
    return 0;                                                                                                
  }

  //  assert(pos < channelsOfGroup.size());

  AbstractPlotChannel* channel = 0;
  int it = 0;
  FOREACHC(list<AbstractPlotChannel*>, channelsOfGroup, channelIt) {
    if(it == pos) channel = *channelIt;
    it++;
  }
//  for( const_iterator i = channelsOfGroup.begin(); i !=  channelsOfGroup.end(); i++){
//    if(it == pos) cannel = *i;
//    it++;
//  }
  return channel;

}


