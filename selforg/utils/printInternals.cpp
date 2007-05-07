#include <stdio.h>
#include <iostream>
using namespace std;
#include <stdlib.h>
#include <assert.h>
#include "printInternals.h"
#include "inspectable.h"

void printNetworkDescription(FILE* f, const string& name, const Inspectable* inspectable){
  assert(inspectable);
  time_t t = time(0);
  fprintf(f,"# Start %s", ctime(&t));
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

void printInternalParameterNames(FILE* f,
				int sensornumber, int motornumber,
				list<const Inspectable*> inspectables) {
  fprintf(f,"#C");
  for(int i = 0; i < sensornumber; i++){
    fprintf(f, " x[%i]", i);
  }
  for(int i = 0; i < motornumber; i++){
    fprintf(f, " y[%i]", i);
  }
  FOREACHC(list<const Inspectable*>, inspectables, insp){
    if(*insp){
      // then the internal parameters
      list<Inspectable::iparamkey> l = (*insp)->getInternalParamNames();
	    for(list<Inspectable::iparamkey>::iterator i = l.begin(); i != l.end(); i++){
		    fprintf(f, " %s", (*i).c_str());
    	}
    }
  }
  fprintf(f,"\n"); // terminate line
}

void printInternalParameters(FILE* f,
			     const sensor* x, int sensornumber,
			     const motor* y,  int motornumber,
			     list<const Inspectable*> inspectables){

  for(int i = 0; i < sensornumber; i++){
    fprintf(f, " %f", x[i]);
  }
  for(int i = 0; i < motornumber; i++){
    fprintf(f, " %f", y[i]);
  }
  // internal parameters ( we allocate one place more to be able to realise when the number raises)
  list<Inspectable::iparamval> l;
  FOREACHC(list<const Inspectable*>, inspectables, insp){
    if(*insp){
      l = (*insp)->getInternalParams();
      FOREACHC(list<Inspectable::iparamval>, l, i ){
	fprintf(f, " %f", (*i));
      }
    }
  }
  fprintf(f,"\n"); // terminate line

}
