/*
 * XMLSimulation.h
 *
 *  Created on: 07.03.2010
 *      Author: guettler
 */

#ifndef XMLSIMULATION_H_
#define XMLSIMULATION_H_

#include <ode_robots/simulation.h>

class XMLParserEngine;

namespace lpzrobots {
  
  /**
   * Wrapper class to get access to protected areas without manipulating the class Simulation
   */
  class XMLSimulation : public Simulation {
    public:
      using Simulation::setCameraHomePos;
      using Simulation::plotoptions;
      using Simulation::globalData;
      using Base::osgHandle;

      XMLSimulation();
      virtual ~XMLSimulation();
  };

}

#endif /* XMLSIMULATION_H_ */
