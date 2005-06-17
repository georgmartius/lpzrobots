/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.3  2005-06-17 10:47:34  martius
 *   Logging mode as enum (GuiLogger, NoPlot)
 *
 *   Revision 1.2  2005/06/15 14:02:47  martius
 *   revised and basicly tested
 *                                                                 *
 ***************************************************************************/
#ifndef __ONE2ONEAGENT_H
#define __ONE2ONEAGENT_H

#include "plotagent.h"
class NoiseGenerator;

/// Glue-object between controller and robot which implements a 1 to 1 mapping 
class One2OneAgent : public PlotAgent {
public:
  One2OneAgent(PlotMode plotmode=GuiLogger) : PlotAgent(plotmode) {
    sensors=0; motors=0; 
  }
  
  virtual One2OneAgent::~One2OneAgent();

  virtual bool init(AbstractController* controller, AbstractRobot* robot);

  /// @param noise Noise strength.
  virtual void step(double noise);
private:
  int sensornumber;
  int motornumber;
  sensor *sensors;
  motor *motors;

  NoiseGenerator* noise_gen;
 
};


#endif
