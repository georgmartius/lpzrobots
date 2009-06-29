/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *  Revision 1.2  2009-06-29 13:11:32  robot14
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

PlotOptionEngine::PlotOptionEngine(const PlotOption& plotOption)
{
  if(plotOption.mode!=NoPlot) plotOptions.push_back(plotOption);
  t=1;
}

PlotOptionEngine::PlotOptionEngine(const std::list<PlotOption>& plotOptions) : plotOptions(plotOptions)
{
	t=1;
}


PlotOptionEngine::~PlotOptionEngine()
{
  // closes all pipes of the agents due to pause mode or so
  for (int i = NoPlot; i < LastPlot; i++){
    removePlotOption((PlotMode)i);
  }
}

PlotOption PlotOptionEngine::addPlotOption(PlotOption& plotOption) {
  PlotOption po = plotOption;
  // if plotoption with the same mode exists -> delete it
  removePlotOption(po.mode);

  // this prevents the simulation to terminate if the child  closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN);
  po.open();
  if(po.pipe){
	// print start
	time_t t = time(0);
	fprintf(po.pipe,"# Start %s", ctime(&t));
	// print network description given by the structural information of the controller
	//printNetworkDescription(po.pipe, "Selforg"/*controller->getName()*/, controller);

	fprintf(po.pipe,"#N neural_net Selforg\n");
	for(std::list<const Inspectable*>::const_iterator i = inspectables.begin(); i!=inspectables.end(); i++) {
	  const Inspectable* inspectable = (*i);
	  std::list< Inspectable::ILayer> layers      = inspectable->getStructuralLayers();
	  std::list< Inspectable::IConnection> conns  = inspectable->getStructuralConnections();
	  // print layers with neurons
	  for(std::list<Inspectable::ILayer>::iterator i = layers.begin(); i != layers.end(); i++){
		Inspectable::ILayer& l = (*i);
		fprintf(po.pipe, "#N layer %s %i\n", l.layername.c_str(), l.rank);
		for(int n = 0; n < l.dimension; n++){
		  if(l.biasname.empty()){
		fprintf(po.pipe, "#N neuron %s[%i]\n", l.vectorname.c_str(), n);
		  }else {
		fprintf(po.pipe, "#N neuron %s[%i] %s[%i]\n", l.vectorname.c_str(), n, l.biasname.c_str(), n);
		  }
		}
	  }

	  // print connections
	  for(std::list<Inspectable::IConnection>::iterator i = conns.begin(); i != conns.end(); i++){
		Inspectable::IConnection& c = (*i);
		// find the layers refered in the connection description
		std::list<Inspectable::ILayer>::iterator l1it
		  = find_if(layers.begin(), layers.end(), Inspectable::matchName(c.vector1) );
		std::list<Inspectable::ILayer>::iterator l2it
		  = find_if(layers.begin(), layers.end(), Inspectable::matchName(c.vector2) );
		assert(l1it != layers.end()); // we need to find them otherwise
		assert(l2it != layers.end());

		Inspectable::ILayer& l1 = (*l1it);
		Inspectable::ILayer& l2 = (*l2it);
		for(int j=0; j < l1.dimension; j++){
		  for(int k=0; k < l2.dimension; k++){
		fprintf(po.pipe, "#N connection %s[%i,%i] %s[%i] %s[%i]\n",
			c.matrixname.c_str(), k, j, l1.vectorname.c_str(), j, l2.vectorname.c_str(), k);
		  }
		}
	  }
	}
	fprintf(po.pipe,"#N nn_end\n");

	// print interval
	fprintf(po.pipe, "# Recording every %dth dataset\n", po.interval);

	fflush(po.pipe);
  }
  else
	printf("Opening of pipe for PlotOption failed!\n");

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
  //if(!initialised){
    inspectables.push_back(inspectable);
  //} else {
    //std::cerr << "WiredController::addInspectable(const Inspectable* inspectable); failed, because WiredController was already initialised! " << std::endl;
  //}
}


void PlotOptionEngine::writePlotComment(const char* cmt){
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
void PlotOptionEngine::plot(double time){
  for(std::list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++){
    if( ((*i).pipe) && ((*i).interval>0) && (t % (*i).interval == 0) ){

      fprintf((*i).pipe, "%f", time);
      printInspectables((*i).pipe, inspectables);
      (*i).flush(t);
    }
  }
}

void PlotOptionEngine::plotNames()
{
	  for(std::list<PlotOption>::iterator i=plotOptions.begin(); i != plotOptions.end(); i++)
	  {
	    if( ((*i).pipe) && ((*i).interval>0) && (t % (*i).interval == 0) )
	    {
	    	if (!(*i).namesPlotted)
	    	{
	    		fprintf((*i).pipe,"#C t");
	    		printInspectableNames((*i).pipe,inspectables);
	    		(*i).namesPlotted = true;
	    	}
	    }
	  }
}

//  Performs an step of the engine:
void PlotOptionEngine::step(double time) {
	plotNames();
	plot(time);
  // do a callback for all registered Callbackable classes
  //FOREACH(list<Callbackable*>, callbackables, i){
//    (*i)->doOnCallBack();
  //}
  t++;
}
