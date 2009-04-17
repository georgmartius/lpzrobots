/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *  Revision 1.1  2009-04-17 14:17:32  guettler
 *  New PlotChannels and Filters for matrices
 *										   *
 *                                                                         *
 **************************************************************************/
#ifndef EXAMPLEMATRIXPIPEFILTER_H_
#define EXAMPLEMATRIXPIPEFILTER_H_

/*
 *
 */
#include "AbstractPipeFilter.h"

/**
 * forward declaration, because not needed directly here
 */
class MatrixPlotChannel;


class ExampleMatrixPipeFilter: public AbstractPipeFilter {
public:
	ExampleMatrixPipeFilter();
	virtual ~ExampleMatrixPipeFilter();

	virtual AbstractPlotChannel* createChannel(std::string name);

	/**
	  * The dataLine from PipeReader will be iterate to set the new channel-value.
	  * The order of the value-input-list (dataList) is important and the index of it
	  * must be equal to the indexnumber of the private channelIndexList that was created by
	  * createChannelList().
	  *
	  * If a new sensor is plugged into hardware-ECB, a new descriptionLine will be created and
	  * the PipeFilter must be reinit to reorder the channelIndexList
	  */
	  virtual void updateChannels();

protected:
	std::list<MatrixPlotChannel*> matrixPlotChannels;
};

#endif /* EXAMPLEMATRIXPIPEFILTER_H_ */
