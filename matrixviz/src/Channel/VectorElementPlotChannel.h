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
 *  Revision 1.1  2010-03-30 13:18:26  robot14
 *  first version
 *

 *                       *
 *                                                                         *
 **************************************************************************/

#ifndef VECTORELEMENTPLOTCHANNEL_H_
#define VECTORELEMENTPLOTCHANNEL_H_
/*
 *
 */
#include "AbstractPlotChannel.h"
#include <vector>

class VectorElementPlotChannel: public AbstractPlotChannel {
public:
  VectorElementPlotChannel(std::string name);
  virtual ~VectorElementPlotChannel();

  void setValue(double v);
  double getValue();
  double getValue(int time);
  void changeSize(int newSize);
  int getSize();
  int getIndex();

private:
  int bufferSize;
  std::vector<double> ringBuffer;
  int ringBufferIndex;
  static const bool debug = false;
};

#endif /* VECTORELEMENTPLOTCHANNEL_H_ */
