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
 *                                                                         *
 *   $Log$
 *   Revision 1.16  2011-03-22 16:49:29  guettler
 *   - adpaptions to enhanced configurable and inspectable interface
 *
 *   Revision 1.15  2011/03/21 17:48:13  guettler
 *   adapted to enhanced Inspectable interface:
 *   - has now a name shown also in GuiLogger
 *   - supports plotting of inspectable childs of an inspectable
 *   - inspectable names are plotted out in description line additionally
 *
 *   Revision 1.14  2010/06/03 09:52:18  martius
 *   using const list references as it should be
 *
 *   Revision 1.13  2009/08/05 20:25:29  martius
 *   Bug in printInternalParameters fixed (removed if(*i) which was nonsense)
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#include <stdio.h>
#include <iostream>
#include <algorithm>
using namespace std;
#include <stdlib.h>
#include <assert.h>
#include "printInternals.h"
#include "inspectable.h"

void printNetworkDescription(FILE* f, const string& name, const Inspectable* inspectable){
  assert(inspectable);
  fprintf(f,"#N neural_net %s\n", name.c_str());
  list< Inspectable::ILayer> layers      = inspectable->getStructuralLayers();
  list< Inspectable::IConnection> conns  = inspectable->getStructuralConnections();
  // print layers with neurons
  for(list<Inspectable::ILayer>::iterator i = layers.begin(); i != layers.end(); i++){
    Inspectable::ILayer& l = (*i);
    fprintf(f, "#N layer %s %i\n", l.layername.c_str(), l.rank);
    for(int n = 0; n < l.dimension; n++){
      if(l.biasname.empty()){
	fprintf(f, "#N neuron %s[%i]\n", l.vectorname.c_str(), n);
      }else {
	fprintf(f, "#N neuron %s[%i] %s[%i]\n", l.vectorname.c_str(), n, l.biasname.c_str(), n);
      }
    }
  }

  // print connections
  for(list<Inspectable::IConnection>::iterator i = conns.begin(); i != conns.end(); i++){
    Inspectable::IConnection& c = (*i);
    // find the layers refered in the connection description
    list<Inspectable::ILayer>::iterator l1it
      = find_if(layers.begin(), layers.end(), Inspectable::matchName(c.vector1) );
    list<Inspectable::ILayer>::iterator l2it
      = find_if(layers.begin(), layers.end(), Inspectable::matchName(c.vector2) );
    assert(l1it != layers.end()); // we need to find them otherwise
    assert(l2it != layers.end());

    Inspectable::ILayer& l1 = (*l1it);
    Inspectable::ILayer& l2 = (*l2it);
    for(int j=0; j < l1.dimension; j++){
      for(int k=0; k < l2.dimension; k++){
	fprintf(f, "#N connection %s[%i,%i] %s[%i] %s[%i]\n",
		c.matrixname.c_str(), k, j, l1.vectorname.c_str(), j, l2.vectorname.c_str(), k);
      }
    }
  }
  fprintf(f,"#N nn_end\n");


}

void printInspectableNames(FILE* f, const list<const Inspectable*>& inspectables) 
{
  if (!f)
    return;

  FOREACHC(list<const Inspectable*>, inspectables, insp){
    if(*insp){
      // then the internal parameters
      list<Inspectable::iparamkey> l = (*insp)->getInternalParamNames();
      for(list<Inspectable::iparamkey>::iterator i = l.begin(); i != l.end(); i++){
	fprintf(f, " %s", (*i).c_str());
      }
      printInspectableNames(f, (*insp)->getInspectables());
    }
  }
}

void printInspectableInfoLines(FILE* f, const list<const Inspectable*>& inspectables) {
  if (!f)
    return;
  FOREACHC(list<const Inspectable*>, inspectables, insp) {
    const list<string>& infoLines = (*insp)->getInfoLines();
    FOREACHC(list<string>, infoLines, infoLine) {
      fprintf(f,"%s", string("#I [").append((*insp)->getNameOfInspectable()).append("] ").append(*infoLine).append("\n").c_str());
    }
    printInspectableInfoLines(f, (*insp)->getInspectables());
  }
}

void printInternalParameterNames(FILE* f,
				int sensornumber, int motornumber,
				const list<const Inspectable*>& inspectables) {
  fprintf(f,"#C t");
  for(int i = 0; i < sensornumber; i++){
    fprintf(f, " x[%i]", i);
  }
  for(int i = 0; i < motornumber; i++){
    fprintf(f, " y[%i]", i);
  }
  printInspectableNames(f,inspectables);
}

void printInspectables(FILE* f, const std::list<const Inspectable*>& inspectables)
{
  if (!f)
    return;

  // internal parameters ( we allocate one place more to be able to realise when the number raises)
  Inspectable::iparamvallist l;
  FOREACHC(list<const Inspectable*>, inspectables, insp)
  {
    if(*insp)
    {
      l = (*insp)->getInternalParams();
      FOREACHC(Inspectable::iparamvallist, l, i )
      {
          fprintf(f, " %f", (*i));
      }
      printInspectables(f, (*insp)->getInspectables());
    }
  }
}

void printInternalParameters(FILE* f, double time,
			     const sensor* x, int sensornumber,
			     const motor* y,  int motornumber,
			     const list<const Inspectable*>& inspectables)
{
  if (!f)
    return;

  fprintf(f, "%f", time);
  for(int i = 0; i < sensornumber; i++){
    fprintf(f, " %f", x[i]);
  }
  for(int i = 0; i < motornumber; i++){
    fprintf(f, " %f", y[i]);
  }
  printInspectables(f,inspectables);
  fprintf(f,"\n"); // terminate line
}

