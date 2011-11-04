/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
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
  /// plotting with matrixVisualizer
  MatrixViz,
  /// net visualiser
  NeuronViz,

  /// Acustic output of robotic values via external SoundMan
  SoundMan,

  /// gui for ECBRobots (see lpzrobots/ecbrobots), should be usable with OdeRobots, too
  ECBRobotGUI,

  /// dummy used for upper bound of plotmode type
  LastPlot
};


/** This class contains options for the use of an external plot utility like guilogger or neuronviz
    or just simply file output
 */
class PlotOption {
public:
  friend class WiredController;
  friend class PlotOptionEngine;

  PlotOption(){ mode=NoPlot; interval=1; pipe=0; parameter="";}
  /**
     creates a new plotting object
     @param mode output type @see PlotMode
     @param interval every i-th step is plotted
     @param parameter free parameters for plotting tool
     Note: the argument whichSensor is removed. You can adjust this in the wirings now.
   */
  PlotOption( PlotMode mode, int interval = 1, std::string parameter="")
    : interval(interval), mode(mode), parameter(parameter)
  {
    pipe=0;
  }

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
  const std::string& getName() const { return name; }

  bool open(); ///< opens the connections to the plot tool
  void close();///< closes the connections to the plot tool

  FILE* pipe;
  long t;
  int interval;
  std::string name;

private:

  PlotMode mode;
  std::string parameter; ///< additional parameter for external command
};

#endif /* PLOTOPTION_H_ */
