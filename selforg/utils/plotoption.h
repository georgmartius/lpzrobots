/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    joergweide84@aol.com (robot12)                                       *
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
 *  Revision 1.2  2009-07-21 08:50:16  robot12
 *  finish of the split
 *  to do: add some comments....
 *
 *  Revision 1.1  2009/06/02 09:55:24  robot12
 *  Splitting of WiredController and PlotOption into WiredController : public PlotOptionEngine and
 *  PlotOption (used by ga_tools). Further refactorings needed.
 *										   *
 *                                                                         *
 **************************************************************************/


#ifndef PLOTOPTION_H_
#define PLOTOPTION_H_

#include <stdio.h>
#include <list>
#include <utility>
#include <string>

class Configurable;
class Inspectable;

/** Output mode for agent.
 */
enum PlotMode {
  /// dummy (does nothing) is there for compatibility, might be removed later
  NoPlot,
  /// write into file
  File,
  /// plotting with guilogger (gnuplot)
  GuiLogger,
  /// plotting with guiscreen (gnuplot) in file logging mode
  GuiLogger_File,
  /// net visualiser
  NeuronViz,

  /// Acustic output of robotic values via external SoundMan
  SoundMan,

  /// gui for ECBRobots (see lpzrobots/ecbrobots), should be usable with OdeRobots, too
  ECBRobotGUI,

  /// dummy used for upper bound of plotmode type
  LastPlot
};

/** Output either sensors from robot or from controller
    (there can be a difference depending on the used wiring)
 */
enum PlotSensors {Robot, Controller};

/** This class contains options for the use of an external plot utility like guilogger or neuronviz
    or just simply file output
 */
class PlotOption {
public:
  friend class WiredController;
  friend class PlotOptionEngine;

  PlotOption(){ mode=NoPlot; whichSensors=Controller; interval=1; pipe=0; parameter="";}
  PlotOption( PlotMode mode, PlotSensors whichSensors = Controller,
              int interval = 1,
              std::list<const Configurable*> confs = std::list<const Configurable*>(),
              std::string parameter="")
    : interval(interval), mode(mode), whichSensors(whichSensors),
    configureables(confs), parameter(parameter), namesPlotted(false)
    { pipe=0; }

  virtual ~PlotOption(){}

  virtual PlotMode getPlotOptionMode() const { return mode; }
  // flushes pipe (depending on mode)
  virtual void flush(long step);

  /// nice predicate function for finding by mode
  struct matchMode : public std::unary_function<const PlotOption&, bool> {
    matchMode(PlotMode mode) : mode(mode) {}
    int mode;
    bool operator()(const PlotOption& m) { return (m.mode == mode); }

  };

  void addConfigurable(const Configurable*);
  void setName(const std::string& name) { this->name = name;}

  bool open(); ///< opens the connections to the plot tool
  void close();///< closes the connections to the plot tool

  FILE* pipe;
  long t;
  int interval;
  std::string name;

private:

  PlotMode mode;
  PlotSensors whichSensors;
  std::list< const Configurable* > configureables;
  std::string parameter; ///< additional parameter for external command
  bool namesPlotted;
};

#endif /* PLOTOPTION_H_ */
