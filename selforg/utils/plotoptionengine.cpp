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

#include "plotoptionengine.h"
#include "plotoption.h"
#include "inspectable.h"
#include <signal.h>
#include <string>
#include <assert.h>
#include <string.h>
#include <algorithm>
#include <locale.h> // need to set LC_NUMERIC to have a '.' in the numbers written or piped to gnuplot

using namespace std;

PlotOptionEngine::PlotOptionEngine(const PlotOption& plotOption) : maybe_controller(0), name("") {
  if(plotOption.mode!=NoPlot)
    plotOptions.push_back(plotOption);
  initialised = false;
  t=1;
}

PlotOptionEngine::PlotOptionEngine(const list<PlotOption>& plotOptions)
  : plotOptions(plotOptions), maybe_controller(0), name("") {
  initialised = false;
  t=1;
}


PlotOptionEngine::~PlotOptionEngine() {
  closePipes();
}


bool PlotOptionEngine::init(AbstractController* maybe_controller){
  setlocale(LC_NUMERIC,"en_US"); // set us type output

  this->maybe_controller = maybe_controller;
#ifdef SIGPIPE // is not defined on windows
  // this prevents the simulation to terminate if the child  closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN);
#endif
  FOREACH(list<PlotOption>, plotOptions, po){
    initPlotOption(*po);
  }
  initialised = true;
  return true;
}

bool PlotOptionEngine::initPlotOption(PlotOption& po){
  po.open();
  if(po.pipe){
    // print start
    time_t t = time(0);
    fprintf(po.pipe,"# Start %s", ctime(&t));
    if (po.getName().size()>0) // print name of PlotOption
      fprintf(po.pipe, "#IN %s\n", po.getName().c_str());
    // print network description given by the structural information of the controller
    if(maybe_controller){
      po.printNetworkDescription(maybe_controller->getName(), maybe_controller);
    }
    // print interval
    if(po.interval==1)
      fprintf(po.pipe, "# Recording every dataset\n");
    else
      fprintf(po.pipe, "# Recording every %dth dataset\n", po.interval);
    // print all configureables
    FOREACHC(list<const Configurable*>, configureables, i){
      (*i)->print(po.pipe, "# ", 1000); // do not wrap descriptions (up to len 1000)
    }
    // print infolines of all inspectables
    fprintf(po.pipe,"#I D t time (s)\n"); // add description for time
    po.printInspectableInfoLines(inspectables);

    fprintf(po.pipe,"#######\n");
    // print head line with all parameter names
    fprintf(po.pipe,"#C t");
    po.printInspectableNames(inspectables,0);
    fprintf(po.pipe,"\n"); // terminate line
    return true;
  } else {
    fprintf(stderr,"Opening of pipe for PlotOption failed!\n");
    return false;
  }
}

bool PlotOptionEngine::reInit() {
  closePipes();
  return init(maybe_controller);
}

void PlotOptionEngine::closePipes() {
  if (initialised) {
    // closes all pipes of the agents due to pause mode or so
    FOREACH(list<PlotOption>, plotOptions, po){
      po->close();
    }
    initialised = false;
  }
}

void PlotOptionEngine::setName(const string& _name){
  name = _name;
  FOREACH(list<PlotOption>, plotOptions, po){
    po->setName(_name);
  }
}

PlotOption& PlotOptionEngine::addPlotOption(const PlotOption& plotOption) {
  PlotOption po = plotOption;
  // set name of PlotOption if not existent
  if (po.getName().empty())
    po.setName(name);
  // Georg: no we do not delete it anymore. Allowing for more multiple
  //  (e.g. file logging with different interval)
  // if plotoption with the same mode exists -> delete it
  // removePlotOption(po.mode);

  plotOptions.push_back(po);

  return plotOptions.back();
}

bool PlotOptionEngine::addAndInitPlotOption(const PlotOption& plotOption, bool forceInit) {
  PlotOption& po = addPlotOption(plotOption);
  if (initialised || forceInit)
    return initPlotOption(po);
  else
    return true;
}

bool PlotOptionEngine::removePlotOption(PlotMode mode) {
  // if plotoption with the same mode exists -> delete it
  auto po = find_if(plotOptions.rbegin(), plotOptions.rend(), PlotOption::matchMode(mode));
  if(po != plotOptions.rend()){
    (*po).close();
    plotOptions.erase(--po.base());
    return true;
  }
  return false;
}

void PlotOptionEngine::addInspectable(const Inspectable* inspectable, bool front){
  assert(!initialised);
  if(front)
    inspectables.push_front(inspectable);
  else
    inspectables.push_back(inspectable);
}

void PlotOptionEngine::addConfigurable(const Configurable* c){
  assert(!initialised);
  configureables.push_back(c);
}

void PlotOptionEngine::writePlotComment(const char* cmt, bool addSpace){
  assert(initialised);
  for(auto &po : plotOptions){
    if( (po.pipe) && (strlen(cmt)>0)){ // for the guilogger pipe
      char last = cmt[strlen(cmt)-1];
      if(addSpace)
        fprintf(po.pipe, "# %s", cmt);
      else
        fprintf(po.pipe, "#%s", cmt);
      if(last!=10 && last!=13) // print with or without new line
        fprintf(po.pipe, "\n");
    }
  }
}

// Plots controller sensor- and motorvalues and internal controller parameters.
void PlotOptionEngine::plot(double time)
{
  assert(initialised);

  for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++)
  {
    if ( ((*i).pipe) && ((*i).interval>0) && (t % (*i).interval == 0) )
    {
      fprintf((*i).pipe, "%f", time);
      i->printInspectables(inspectables,0);
      fprintf((*i).pipe,"\n"); // terminate line
      (*i).flush(t);
    }
  }
  t++;
}




// GEORG: it is better to plot it at initialization time!
// void PlotOptionEngine::plotNames()
// {
//           for(list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++)
//           {
//             if( ((*i).pipe) && ((*i).interval>0) && (t % (*i).interval == 0) )
//             {
//                     if (!(*i).namesPlotted)
//                     {
//                             fprintf((*i).pipe,"#C t");
//                             printInspectableNames((*i).pipe,inspectables);
//                             (*i).namesPlotted = true;
//                     }
//             }
//           }
// }
