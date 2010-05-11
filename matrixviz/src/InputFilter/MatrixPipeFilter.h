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
 *  Revision 1.5  2010-05-11 16:53:03  robot14
 *  *** empty log message ***
 *
 *  Revision 1.4  2010/03/30 13:18:06  robot14
 *  fixed
 *
 *  Revision 1.3  2009/10/22 15:53:08  robot14
 *  first version of texture visualisation
 *
 *  Revision 1.2  2009/10/02 15:25:40  robot14
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
#ifndef MATRIXPIPEFILTER_H_
#define MATRIXPIPEFILTER_H_

/*
 *
 */
#include "MatrixPlotChannel.h"
#include "MatrixElementPlotChannel.h"
#include "VectorPlotChannel.h"
#include "VectorElementPlotChannel.h"
#include "AbstractPipeFilter.h"
//#include "AbstractPipeReader.h"
#include <vector>

/**
 * forward declaration, because not needed directly here
 */


class MatrixPipeFilter: public AbstractPipeFilter {

  Q_OBJECT

public:
	MatrixPipeFilter(AbstractPipeReader* apr);
	virtual ~MatrixPipeFilter();

	virtual AbstractPlotChannel* createChannel(std::string name);

	virtual std::vector<MatrixPlotChannel*> getMatrixChannels();
	virtual std::vector<VectorPlotChannel*> getVectorChannels();

public slots:
	/**
	  * The dataLine from PipeReader will be iterate to set the new channel-value.
	  * The order of the value-input-list (dataList) is important and the index of it
	  * must be equal to the indexnumber of the private channelIndexList that was created by
	  * createChannelList().
	  *
	  * If a new sensor is plugged into hardware-ECB, a new descriptionLine will be created and
	  * the PipeFilter must be reinit to reorder the channelIndexList
	  */
	  void updateChannels();

protected:
	std::vector<MatrixPlotChannel*> matrices; //get
	std::vector<VectorPlotChannel*> vectors;

private:
	static const bool debug = false;
};

#endif /* MATRIXPIPEFILTER_H_ */
