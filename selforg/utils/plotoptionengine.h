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
 *  Revision 1.7  2010-07-02 15:57:25  martius
 *  wirings have new initIntern signature -> less errors can be made
 *  abstractwiring generates the noise of given length
 *
 *  Revision 1.6  2009/08/10 15:36:19  der
 *  plotoptions can again be added and initialized later
 *  ctrl-g and -f are working again
 *  ctrl-n added for neuronviz
 *
 *  Revision 1.5  2009/08/10 07:37:48  guettler
 *  -Inspectable interface now supports to add infoLines itself.
 *   These lines are then outprinted line by line to the PlotOption once,
 *   preceded by a #I.
 *  -Restart functionality of PlotOptionEngine added (e.g. closePipes(), reInit()).
 *
 *  Revision 1.4  2009/08/05 22:53:02  martius
 *  redesigned
 *   works as a stand alone object now
 *   added init function
 *   configurables are now in engine and not in plotoptions
 *   works with wiredcontroller
 *
 *  Revision 1.3  2009/07/21 08:50:16  robot12
 *  finish of the split
 *  to do: add some comments....
 *
 *  Revision 1.2  2009/06/30 14:20:49  robot12
 *  finishing the gen API and add some comments
 *
 *  Revision 1.1  2009/06/02 09:55:24  robot12
 *  Splitting of WiredController and PlotOption into WiredController : public PlotOptionEngine and
 *  PlotOption (used by ga_tools). Further refactorings needed.
 *										   *
 *                                                                         *
 **************************************************************************/

#ifndef PLOTOPTIONENGINE_H_
#define PLOTOPTIONENGINE_H_

#include <list>

#include "plotoption.h"
#include <selforg/abstractcontroller.h>

class Inspectable;

/*
 *
 */
class PlotOptionEngine
{
public:
  PlotOptionEngine(const PlotOption& plotOption);
  PlotOptionEngine(const std::list<PlotOption>& plotOptions);

  virtual ~PlotOptionEngine();

  /** initializes PlotOptionEngine and opens all pipes and stuff.
      The optional controller is used to print structure information
   */
  virtual bool init(AbstractController* maybe_controller =0);

  /**
   * Reinitialises the PlotOptionEngine.
   * This means it closes all open pipes by calling closePipes()
   * and restarts them by calling init().
   * @return true if succeeded, otherwise false.
   */
  virtual bool reInit();

  /**
   * Closes all open pipes of the current used PlotOptions.
   */
  virtual void closePipes();

  /**
     sets the name of all plotoptions (call before init, but after options are added)
   */
  virtual void setName(const std::string& name);

  /** adds the PlotOptions to the list of plotoptions
      If a plotoption with the same Mode exists, then the old one is deleted first
   */
  virtual PlotOption& addPlotOption(PlotOption& plotoption);

  /** adds the PlotOptions to the list of plotoptions
      If a plotoption with the same Mode exists, then the old one is deleted first
      The plotting is also initialized
   */
  virtual bool addAndInitPlotOption(PlotOption& plotoption);

  /** removes the PlotOptions with the given type
      @return true if sucessful, false otherwise
   */
  virtual bool removePlotOption(PlotMode mode);


  /** adds an inspectable object for logging. Must be called before init!
   */
  virtual void addInspectable(const Inspectable* inspectable, bool front = false);

  /** adds an configureable object for logging. Must be called before init!
   */
  virtual void addConfigurable(const Configurable* c);

  /**
     write comment to output streams (PlotOptions). For instance changes in parameters.
  */
  virtual void writePlotComment(const char* cmt);

  virtual void plot(double time);

protected:

  bool initPlotOption(PlotOption& po);

  std::list<PlotOption> plotOptions;
  std::list<const Inspectable* > inspectables;
  std::list< const Configurable* > configureables;
  long int t;
  
  bool initialised;

  // old artefact, should be removed in future releases
  AbstractController* maybe_controller;
};

#endif /* PLOTOPTIONENGINE_H_ */
