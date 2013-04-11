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
