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
#ifndef VECTORPLOTCHANNEL_H_
#define VECTORPLOTCHANNEL_H_


#include "MatrixPlotChannel.h"
#include "VectorElementPlotChannel.h"

/**
 * Beinhaltet zusätzliche Informationen und Methoden, um einen Vector auf Timestamp abgebildet als einheitlichen Channel zu repräsentieren
 */
class VectorPlotChannel: public MatrixPlotChannel {
public:
  VectorPlotChannel(std::string name);
  virtual ~VectorPlotChannel();

  virtual int getDimension(int dim);
  virtual double getValue(int num);
  virtual double getValue(int num, int t);

  VectorElementPlotChannel* getChannel(int num);

  virtual void addElement(VectorElementPlotChannel* vc);

  virtual int getSize();
  virtual int getBufferSize();
  virtual void setBufferSize(int newSize);


protected:

private:
  static const bool debug = false;

};


#endif /* VECTORPLOTCHANNEL_H_ */
