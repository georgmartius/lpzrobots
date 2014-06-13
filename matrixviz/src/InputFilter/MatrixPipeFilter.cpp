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

#include "MatrixPipeFilter.h"

#include "DefaultPlotChannel.h"
#include <iostream>

using namespace std;

MatrixPipeFilter::MatrixPipeFilter(AbstractPipeReader* apr) :
  AbstractPipeFilter(apr) {
  if (debug) cout << "new MatrixPipeFilter()" << endl;
  QObject::connect(apr,SIGNAL(newData()),this,SLOT(updateChannels()), Qt::DirectConnection);
        // TODO Auto-generated constructor stub

}

MatrixPipeFilter::~MatrixPipeFilter() {
        // TODO Auto-generated destructor stub
}

AbstractPlotChannel* MatrixPipeFilter::createChannel(std::string name)
{
        if (debug) cout << name << endl;
  bool hasBrace = false, hasComma = false;
  int bracePos=0, commaPos=0;
  for(unsigned int i = 0; i < name.size(); i++){
    if(name.at(i) == '['){
      hasBrace = true;
      bracePos = i;
    }else
    if(name.at(i) == ','){
      hasComma = true;
      commaPos = i;
    }
  }
        /*
         * Looking for new vector
         */
        if (hasBrace && !hasComma) {
    // empty vector or new vector (different name)
    if (vectors.size() == 0 || name.substr(0, bracePos) != vectors.back()->getChannelName().substr(0, bracePos)) {
      VectorPlotChannel* vePloChannel = new VectorPlotChannel(name.substr(0, bracePos));
      if (debug) cout << "push_back new vpc" << endl;
      vectors.push_back(vePloChannel);
    }
    if (debug) cout << "new VEPC" << endl;
    VectorElementPlotChannel* elementChannel = new VectorElementPlotChannel(name);
    if (debug) cout << "vectors back" << endl;
    vectors.back()->addElement(elementChannel);

    if (debug) cout << "return VEPC" << endl;
    return elementChannel;
  } else {
    /*
     * Looking for new matrix _A_[x,y]
     */
          if (hasBrace && hasComma /*&& name.at(0) == toupper(name.at(0))*/ ) {
      // empty vector or new matrix
      if (debug)
        cout << "test new matrix: " << name << endl;
      if (matrices.size() == 0 || name.substr(0, bracePos) != matrices.back()->getChannelName().substr(0, bracePos)) {
        MatrixPlotChannel* maPloChannel = new MatrixPlotChannel(name.substr(0, bracePos));
        matrices.push_back(maPloChannel);
      }
      //A[x,0] : new row channel (active + add) new elementplotchannel to active rowchannel
      if (name.substr(commaPos + 1, 1) == "0") {
        if (debug ) cout << "new MPC" << endl;
        MatrixPlotChannel* rowChannel = new MatrixPlotChannel("0");
        matrices.back()->addRow(rowChannel);
      }
      MatrixElementPlotChannel* elementChannel = new MatrixElementPlotChannel(name);

      matrices.back()->getLastRow()->addPlotChannel(elementChannel);

      return elementChannel;
    } else
      // default
      return new DefaultPlotChannel(name);
  }
}

std::vector<MatrixPlotChannel*> MatrixPipeFilter::getMatrixChannels(){
        return matrices;
}

std::vector<VectorPlotChannel*> MatrixPipeFilter::getVectorChannels(){
  return vectors;
}

void MatrixPipeFilter::updateChannels() {
    if (debug) std::cout << "MatrixPipeFilter: updateChannels()" << std::endl;

    const std::list<double>& dataList = (apr->getDataLine());
    std::list<int>::const_iterator index_it=channelIndexList.begin();
    std::list<AbstractPlotChannel*>::const_iterator channel_it=channelList.begin();

    // int tmp_i=0;
    // for(std::list<double>::const_iterator i=dataList.begin(); i != dataList.end(); i++) {
    //   if (debug) printf("[% .1f]",(*i));
    //   if (tmp_i > 5) break;
    //   ++tmp_i;
    // }
    // if (debug) printf("\r\n");

    //int printedIndex = 0;

    for(std::list<double>::const_iterator i=dataList.begin(); i != dataList.end() && index_it!=channelIndexList.end() && channel_it!=channelList.end() ; ++i)
    {
      (*channel_it)->setValue((*i));
      ++channel_it;
//      if (index == (*index_it)){
////         std::cout << "[" << (*channel_it)->getChannelName() << "=" << index << "]";
//        (*channel_it)->setValue((*i));
////       else std::cout << "[  - ]";
//      }
//      index++;
    }
//     std::cout << ")" << std::endl;
    if (debug) printf("\r\n");
}
