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
 *   Revision 1.2  2005-07-14 16:08:01  fhesse
 *   disabled because agent changed
 *
 *   Revision 1.1  2005/07/06 16:06:28  martius
 *   first attempt to provide pseudosensors. here first and second derivative
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __DERIVATIVEAGENT_H
#define __DERIVATIVEAGENT_H
/*
#include "plotagent.h"
class NoiseGenerator;

/// Glue-object between controller and robot which implements a 1 to 1 mapping 
class DerivativeAgent : public PlotAgent {
public:
  /// created an instance with used derivatives
  //  @param id include zeroth derivative
  //  @param first include first derivative
  //  @param second include second derivative
  //  @param eps updaterate for floating average (0 -> no sensor variation, 1 -> no smoothing)
  //  if all parametes are false, id is set to true (equivalent to One2OneAgent)
  DerivativeAgent(bool id, bool first, bool second, double eps, 
		  NoiseGenerator* noise, PlotMode plotmode=GuiLogger,
		  double derivativeScale=5);
  
  virtual DerivativeAgent::~DerivativeAgent();

  virtual bool init(AbstractController* controller, AbstractRobot* robot);

  /// @param noise Noise strength.
  virtual void step(double noise);
private:

  void calcFirstDerivative();
  void calcSecondDerivative();

 private:
  double eps;
  int robotsensornumber;
  int controllersensornumber;
  int motornumber;
  double derivativeScale;
  static const int buffersize=5;
  int time;
  sensor* controllersensors;   // complete sensors for controller
  sensor* robotsensors;        // real sensor values from robot
  sensor* sensorbuffer[buffersize]; // current and old smoothed sensor values of robot
  sensor* first;               // current first derivative
  sensor* second;              // current second derivative
  motor *motors;
  bool useId, useFirst, useSecond;

  NoiseGenerator* noiseGenerator; 
};

*/
#endif
