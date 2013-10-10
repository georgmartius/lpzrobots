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

#include "MatrixPlotChannel.h"
#include "MatrixElementPlotChannel.h"
#include "cassert"

using namespace std;

MatrixPlotChannel::MatrixPlotChannel(string name) : GroupPlotChannel(name) {
        // TODO Auto-generated constructor stub
}

MatrixPlotChannel::~MatrixPlotChannel() {
        // TODO Auto-generated destructor stub
}


/**
* wieviele Spalten oder Zeilen
* @param dim
* @return
*/
int MatrixPlotChannel::getDimension(int dim)
{
        assert(dim < 2);

        if( dim == 0 ) return channelsOfGroup.size();
        else{
          MatrixPlotChannel* chan = dynamic_cast<MatrixPlotChannel*> (channelsOfGroup.front());
          if(chan == 0) { return 0; }else return chan->getDimension(0);
        }
}

double MatrixPlotChannel::getValue(int row, int column)
{
  return getChannel(row, column)->getValue();
}

MatrixElementPlotChannel* MatrixPlotChannel::getChannel(int row, int column){
  // wenn hauptmatrix, dann suche matrixplotchannel(row) und gib wert von kind-matrixelementplotchannel(clumn)
  // zurück, wenn row- matrixplotchannel, gib nur element von column wieder -> dynamic cast
  MatrixElementPlotChannel* chan;

  MatrixPlotChannel* mRow = dynamic_cast<MatrixPlotChannel*> (at(row));
  if(mRow == 0){ //this is a row
    MatrixElementPlotChannel* element = dynamic_cast<MatrixElementPlotChannel*> (at(column));
    chan = element;
  }else{ //this is a matrix
    MatrixElementPlotChannel* element = dynamic_cast<MatrixElementPlotChannel*> (mRow->at(column));
    /*(val != 0)?: */chan = element;
  }
  //dynamic_cast<MatrixPlotChannel *>(channelsOfGroup).getRow(row)[column]; //sollte gehen NEIN!!

  return chan;
}

GroupPlotChannel* MatrixPlotChannel::getRow(int row){
  GroupPlotChannel* gPlotCh = dynamic_cast<GroupPlotChannel*> (at(row));
  if(gPlotCh == 0){
    return 0;
  }else return gPlotCh;
}

GroupPlotChannel* MatrixPlotChannel::getLastRow(){
  if(channelsOfGroup.back() == 0){
    return 0;
  }
  GroupPlotChannel* gPlotCh = dynamic_cast<GroupPlotChannel*> (channelsOfGroup.back());
  if(gPlotCh == 0){
    return 0;
  }else return gPlotCh;
}

void MatrixPlotChannel::addRow(GroupPlotChannel* gc){
        channelsOfGroup.push_back(gc);
}
