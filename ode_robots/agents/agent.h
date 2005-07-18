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
 *   Revision 1.4  2005-07-18 10:13:46  martius
 *   noise moved to wiring
 *
 *   Revision 1.3  2005/07/14 15:57:53  fhesse
 *   now agent contains controller, robot and wiring, plotting ability included, therefore plotagent can be removed; ono2onewiring replaces one2oneagent
 *
 *   Revision 1.2  2005/06/15 14:02:47  martius
 *   revised and basicly tested
 *                                                                 *
 ***************************************************************************/
#ifndef __AGENT_H
#define __AGENT_H

#include "abstractrobot.h"
#include "abstractcontroller.h"
#include "abstractwiring.h"

/// Plot mode for plot agent.
enum PlotMode {NoPlot, GuiLogger, GuiLogger_File};

/// Abstract object containing controller, robot and wiring between them. 
class Agent {
public:
  /// constructor
  Agent(PlotMode plotmode=GuiLogger);

  ///destructor
  virtual ~Agent();  

  /// initializes the object with the given controller, robot and wiring
  //  and initializes pipe to guilogger
  virtual bool init(AbstractController* controller, AbstractRobot* robot, AbstractWiring* wiring);


  /// Performs an step of the agent, including sensor reading, pushing sensor values through wiring, 
  //  controller step, pushing controller steps back through wiring and sent resulting motorcommands to robot.
  //  @param noise Noise strength.
  virtual void step(double noise);

  /// Returns a pointer to the controller.
  AbstractController* getController() { return controller;}

  /// Returns a pointer to the robot.
  AbstractRobot* getRobot() { return robot;}

  /// Returns a pointer to the wiring.
  AbstractWiring* getWiring() { return wiring;}


protected:

  /**
   * Plots controller sensor- and motorvalues and internal controller parameters.
   * @param x actual sensorvalues (used for generation of motorcommand in actual timestep)
   * @param y actual motorcommand (generated in the actual timestep)
   */
  virtual void plot(const sensor* x, int sensornumber, const motor* y, int motornumber);
  
  bool OpenGui();
  void CloseGui();


  AbstractController* controller;
  AbstractRobot* robot;
  AbstractWiring* wiring;


  int rsensornumber;
  int rmotornumber;
  int csensornumber;
  int cmotornumber;

  sensor *rsensors;
  motor  *rmotors;
  sensor *csensors;
  motor  *cmotors;


private:
  FILE* pipe;
  int numberInternalParameters;
  PlotMode plotmode;



};

#endif
