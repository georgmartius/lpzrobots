#ifndef _PRINT_INTERNALS_H
#define _PRINT_INTERNALS_H

#include "inspectable.h"
#include "types.h"

/** prints the names of the sensor and motor channels 
    and the names of the internal parameters of all inspectable object to the given file (stream)
    (Headline)
*/
void printInternalParameterNames(FILE* f, 
				int sensornumber, int motornumber, 
				std::list<const Inspectable*> inspectables);

/** prints the values of the motors and sensors
    and the internal parameters of the inspectable objects to a file or stream
*/
void printInternalParameters(FILE* f, double time,
			     const sensor* x, int sensornumber, 
			     const motor* y,  int motornumber, 
			     std::list<const Inspectable*> inspectables);

/** prints a network description of the structure given by the inspectable object.
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
void printNetworkDescription(FILE* f, const std::string& name, const Inspectable* inspectable);


#endif
