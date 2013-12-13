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
#include <vector>

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

  PlotOption()
    : pipe(0), interval(1), mode(NoPlot),  parameter("")
  {
    mask.resize(256);
  }

  /**
     creates a new plotting object
     @param mode output type @see PlotMode
     @param interval every i-th step is plotted
     @param parameter free parameters for plotting tool
     &param filter filter for channels, @see setFilter(const std::string)

     Note: the argument whichSensor is removed. You can adjust this in the wirings now.
   */
  PlotOption( PlotMode mode, int interval = 1, std::string parameter=std::string(), std::string filter=std::string())
    : pipe(0), interval(interval), mode(mode), parameter(parameter)
  {
    if(!filter.empty()){
      setFilter(filter);
    }
    mask.resize(256);
  }

  virtual ~PlotOption(){}

  virtual PlotMode getPlotOptionMode() const { return mode; }

  /// sets a filter to this plotoption: To export only selected channels
  virtual void setFilter(const std::list<std::string>& accept, const std::list<std::string>& ignore);
  /// sets a filter to this plotoption: syntax: +accept_substr -ignore_substr ...
  virtual void setFilter(const std::string filter);

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

  virtual bool useChannel(const std::string& name);

  virtual int printInspectables(const std::list<const Inspectable*>& inspectables, int cnt=0);

  virtual int printInspectableNames(const std::list<const Inspectable*>& inspectables, int cnt=0);

  virtual void printInspectableInfoLines( const std::list<const Inspectable*>& inspectables);

  /** prints a network description of the structure given by the inspectable object. (mostly unused now)
    The network description syntax is as follow
    \code
    #N neural_net NETWORKNAME
    #N layer LAYERNAME1 RANK?
    #N neuron N0 BIASN0?
    #N neuron N1 BIASN1?
    #N layer LAYERNAME2 RANK?
    #N neuron K0 BIASK0?
    #N neuron K1 BIASK1?
    ...
    #N connection C00 N0 K0
    #N connection C10 N0 K1
    #N connection C01 N1 K0
    #N connection C11 N1 K1
    ...
    #N nn_end
    \endcode
    All identifiers are alphanumeric without spaces.
  */
  void printNetworkDescription(const std::string& name, const Inspectable* inspectable);

  FILE* pipe;
  long t;
  int interval;
  std::string name;

private:
  PlotMode mode;
  std::string parameter; ///< additional parameter for external command
  std::list<std::string> accept; ///< channels to accept (use) (empty means all)
  std::list<std::string> ignore; ///< channels not ignore      (empty means ignore non)
  std::vector<bool> mask; ///< mask for accepting channels (calculated from accept and ignore)
};

#endif /* PLOTOPTION_H_ */
