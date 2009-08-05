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
 *  Revision 1.5  2009-08-05 22:53:02  martius
 *  redesigned
 *   works as a stand alone object now
 *   added init function
 *   configurables are now in engine and not in plotoptions
 *   works with wiredcontroller
 *
 *  Revision 1.4  2009/07/29 14:19:49  jhoffmann
 *  Various bugfixing, remove memory leaks (with valgrind->memcheck / alleyoop)
 *
 *  Revision 1.3  2009/07/21 08:50:16  robot12
 *  finish of the split
 *  to do: add some comments....
 *
 *  Revision 1.2  2009/06/29 13:11:32  robot14
 *  corrected includes (now gcc4.3 compatible)
 *
 *  Revision 1.1  2009/06/02 09:55:24  robot12
 *  Splitting of WiredController and PlotOption into WiredController : public PlotOptionEngine and
 *  PlotOption (used by ga_tools). Further refactorings needed.
 *                                                                                   *
 *                                                                         *
 **************************************************************************/

#include "plotoptionengine.h"
#include "plotoption.h"
#include "inspectable.h"
#include <signal.h>
#include "printInternals.h"
#include <string>
#include <assert.h>
#include <string.h>
#include <algorithm>

PlotOptionEngine::PlotOptionEngine(const PlotOption& plotOption) {
  if(plotOption.mode!=NoPlot) 
    plotOptions.push_back(plotOption);
  initialised = false;  
  t=1;
}

PlotOptionEngine::PlotOptionEngine(const std::list<PlotOption>& plotOptions) 
  : plotOptions(plotOptions) {
  initialised = false;  
  t=1;
}


PlotOptionEngine::~PlotOptionEngine()
{
  // closes all pipes of the agents due to pause mode or so
  FOREACH(std::list<PlotOption>, plotOptions, po){
    po->close();        
  }
}


bool PlotOptionEngine::init(AbstractController* maybe_controller){
  // this prevents the simulation to terminate if the child  closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN);

  FOREACH(std::list<PlotOption>, plotOptions, po){
    po->open();
    if(po->pipe){
      // print start
      time_t t = time(0);
      fprintf(po->pipe,"# Start %s", ctime(&t));
      // print network description given by the structural information of the controller
      if(maybe_controller){
	printNetworkDescription(po->pipe, maybe_controller->getName(), maybe_controller);
      }
      // print interval
      fprintf(po->pipe, "# Recording every %dth dataset\n", po->interval);
      // print all configureables
      FOREACHC(std::list<const Configurable*>, configureables, i){
	(*i)->print(po->pipe, "# ");
      }
      
      // print head line with all parameter names
      fprintf(po->pipe,"#C t");
      printInspectableNames(po->pipe, inspectables);
    }
    else
      printf("Opening of pipe for PlotOption failed!\n");
  }
  initialised = true;
  return true;
}

void PlotOptionEngine::setName(const std::string& name){
  FOREACH(std::list<PlotOption>, plotOptions, po){
    po->setName(name);
  }
}



PlotOption PlotOptionEngine::addPlotOption(PlotOption& plotOption) {
  PlotOption po = plotOption;
  // if plotoption with the same mode exists -> delete it
  removePlotOption(po.mode);
  
  plotOptions.push_back(po);
  
  return po;
}

bool PlotOptionEngine::removePlotOption(PlotMode mode) {
  // if plotoption with the same mode exists -> delete it
  std::list<PlotOption>::iterator po
    = find_if(plotOptions.begin(), plotOptions.end(), PlotOption::matchMode(mode));
  if(po != plotOptions.end()){
    (*po).close();
    plotOptions.erase(po);
    return true;
  }
  return false;
}

void PlotOptionEngine::addInspectable(const Inspectable* inspectable){  
  assert(!initialised);
  inspectables.push_back(inspectable);
}

void PlotOptionEngine::addConfigurable(const Configurable* c){
  assert(!initialised);
  configureables.push_back(c);
}

void PlotOptionEngine::writePlotComment(const char* cmt){
  assert(initialised);
  for(std::list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    if( ((*i).pipe) && (t % (*i).interval == 0) && (strlen(cmt)>0)){ // for the guilogger pipe
      char last = cmt[strlen(cmt)-1];
      fprintf((*i).pipe, "# %s", cmt);
      if(last!=10 && last!=13) // print with or without new line
        fprintf((*i).pipe, "\n");
    }
  }
}

// Plots controller sensor- and motorvalues and internal controller parameters.
void PlotOptionEngine::plot(double time)
{
  assert(initialised);

  for(std::list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++)
  {
    if ( ((*i).pipe) && ((*i).interval>0) && (t % (*i).interval == 0) )
    {
      fprintf((*i).pipe, "%f", time);
      printInspectables((*i).pipe, inspectables);
      (*i).flush(t);
    }
  }
  t++;
}


// GEORG: it is better to plot it at initialization time!
// void PlotOptionEngine::plotNames()
// {
// 	  for(std::list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++)
// 	  {
// 	    if( ((*i).pipe) && ((*i).interval>0) && (t % (*i).interval == 0) )
// 	    {
// 	    	if (!(*i).namesPlotted)
// 	    	{
// 	    		fprintf((*i).pipe,"#C t");
// 	    		printInspectableNames((*i).pipe,inspectables);
// 	    		(*i).namesPlotted = true;
// 	    	}
// 	    }
// 	  }
// }
