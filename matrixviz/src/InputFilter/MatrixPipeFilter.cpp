/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    mam06fyl@studerv.uni-leipzig.de (robot14)
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.2  2009-10-02 15:25:40  robot14
 *  filters, main app - not finished yet
 *
 *  Revision 1.1  2009/08/13 13:14:05  robot14
 *  first version
 *
 *  Revision 1.1  2009/04/17 14:17:32  guettler
 *  New PlotChannels and Filters for matrices
 *										   *
 *                                                                         *
 **************************************************************************/

#include "MatrixPipeFilter.h"
#include "MatrixPlotChannel.h"
#include "MatrixElementPlotChannel.h"
#include "DefaultPlotChannel.h"


MatrixPipeFilter::MatrixPipeFilter(AbstractPipeReader* apr) :
  AbstractPipeFilter(apr) {
	// TODO Auto-generated constructor stub

}

MatrixPipeFilter::~MatrixPipeFilter() {
	// TODO Auto-generated destructor stub
}

AbstractPlotChannel* MatrixPipeFilter::createChannel(std::string name)
{
    //if (name.find("A[0,1]")==0) return (new MotorSpeedPlotChannel("motorCspeedX"));
	//test
	std::cout << name << std::endl;
	/*
	 * Looking for new matrix _A_[x,y] (and not A[x_]_)
	 */
	if (name.at(0) == toupper(name.at(0)) && name.at(3) != ']' /*ERSTER BUCHSTABE IN name großgeschrieben*/){
	  // empty vector or new matrix
	  if (matrices.size() == 0 || name.at(0) != matrices.back()->getChannelName().at(0)){
	    MatrixPlotChannel* maPloChannel = new MatrixPlotChannel(name.substr(0,1));
	    matrices.push_back(maPloChannel);
	  }
	  //A[x,0] : new row channel (active + add) new elementplotchannel to active rowchannel
	  if (name.substr(4,1) == "0"){
	    MatrixPlotChannel* rowChannel = new MatrixPlotChannel("0");
	    matrices.back()->addRow(rowChannel);
	  }
	  MatrixElementPlotChannel* elementChannel = new MatrixElementPlotChannel( name );

	  matrices.back()->getLastRow()->addPlotChannel(elementChannel);
//	{ // Matrix element found!
//		bool isNewChannel = true;
//		// suche richtigen MatrixPlotChannel
//		MatrixPlotChannel* matrixChannel;
//		matrixChannel = new MatrixPlotChannel(/*Großbuchstabe*/name.substr(0,1));
//		int itM;
//		for( int i = 0; i < matrixPlotChannels.size(); i++ )
//		{// wenn noch nicht in Liste:
//			if( matrixPlotChannels[i] == matrixChannel )
//			{
//				isNewChannel = false;
//				itM = i; //iterator merken
//			}
//		}
//		if ( isNewChannel )
//			{
//			matrixPlotChannels.push_back(matrixChannel);
//			itM = matrixPlotChannels.size() - 1; //index of this element
//			}
//
//		/*
//		 * Looking for row A[_x_,y]
//		 */
//		bool isNewRow = true;
//		int itR;
//		MatrixPlotChannel* matrixRowChannel;
//		// name: "x" von A[x,y]
//		matrixChannel = new MatrixPlotChannel( name.substr(2,1) );
//		for( int j = 0; j < matrixPlotChannels[itM]->getDimension(0); j++ )
//		{
//			if( matrixPlotChannels[itM]->getRow(j) == matrixRowChannel ) //schon drin
//			{
//				isNewRow = false;
//				itR = j;
//			}
//		}
//		if ( isNewRow )
//		{
//			matrixPlotChannels[itM]->addRow(matrixRowChannel);
//			itR = matrixPlotChannels[itM]->size() - 1; //index of this element
//		}
//
//		/*
//		 * Adding matrix element channel A[x,y] (A[x,_y_])
//		 */
//		MatrixElementPlotChannel* elementChannel( name.substr(4,1) );
//		this.matrixPlotChannels[itM]->getRow( itR )->addPlotCannel( elementChannel );


		// eigentlichen Channel hinzufügen
		// überlegen: evtl. Dimension der Matrix in der Hierarchie berücksichtigen (mxn)
		// addRow(GroupChannel*) verwenden usw.

		return elementChannel;
	}
	else // default
		return new DefaultPlotChannel(name);
}

std::vector<MatrixPlotChannel*> MatrixPipeFilter::getMatrixChannels(){
	return matrices;
}


void MatrixPipeFilter::updateChannels() {
//     std::cout << "AbstractPipeFilter: updateChannels()" << std::endl;

//     std::cout << "AbstractPipeFilter: updateChannels(";

    std::list<double> dataList = (apr->getDataLine());
    int index=0;
    std::list<int>::const_iterator index_it=channelIndexList.begin();
    std::list<AbstractPlotChannel*>::const_iterator channel_it=channelList.begin();

    int tmp_i=0;
    for(std::list<double>::iterator i=dataList.begin(); i != dataList.end(); i++) {
      printf("[% .1f]",(*i));
      if (tmp_i > 5) break;
      tmp_i++;
    }
    printf("\r\n");

    int printedIndex = 0;

    for(std::list<double>::iterator i=dataList.begin(); i != dataList.end() && index_it!=channelIndexList.end() && channel_it!=channelList.end() ; i++)
    {
      if (index == (*index_it)){
//         std::cout << "[" << (*channel_it)->getChannelName() << "=" << index << "]";
        (*channel_it)->setValue((*i));
      }else{ //the old value has to be
        printf("[old~]");

        channel_it++;
        index_it++;
      }
//       else std::cout << "[  - ]";

      index++;
    }
//     std::cout << ")" << std::endl;
    printf("\r\n");
}
