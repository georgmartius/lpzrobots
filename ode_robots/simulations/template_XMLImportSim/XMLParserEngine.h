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
 *
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.3  2010-06-15 15:02:19  guettler
 *  using now "XercescForwardDecl.h" to avoid namespace problems (3_0, 3_1)
 *
 *  Revision 1.2  2010/03/10 13:54:59  guettler
 *  further developments for xmlimport
 *
 *  Revision 1.1  2010/03/07 22:50:38  guettler
 *  first development state for feature XMLImport
 *                                                                                   *
 *                                                                         *
 **************************************************************************/
#ifndef __XMLPARSERENGINE_H_
#define __XMLPARSERENGINE_H_

#include <ode_robots/globaldata.h>
#include <ode_robots/odehandle.h>
#include <ode_robots/osghandle.h>

#include "XercescForwardDecl.h"
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>

#include "XMLPrimitiveFactory.h"

namespace lpzrobots {
  class XMLSimulation;
}

/*
 *
 */
class XMLParserEngine {
  public:
    XMLParserEngine(lpzrobots::GlobalData& globalData, const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle, lpzrobots::XMLSimulation* simulation);
    virtual ~XMLParserEngine();

    /**
     * Loads the given XML file into the simulation.
     * @param XMLFile name of file to load
     * @return true if successful loaded, otherwise false
     */
    bool loadXMLFile(std::string XMLFile);

    void setValidateXML(bool validate);

    /**
     * Tells if the DOMParser is validating the xml file. If yes,
     * he does do this only if a grammar is given.
     * @return
     */
    bool isValidateXML();

    lpzrobots::GlobalData& getGlobalData() { return globalData; }

    const lpzrobots::OdeHandle& getOdeHandle() { return odeHandle; }

    const lpzrobots::OsgHandle& getOsgHandle() { return osgHandle; }

    const lpzrobots::XMLSimulation* getSimulation() { return simulation; }

    XMLPrimitiveFactory* getPrimitiveFactory() { return primitiveFactory; }


  protected:
    xercesc::XercesDOMParser* parser;
    lpzrobots::GlobalData& globalData;
    const lpzrobots::OdeHandle& odeHandle;
    const lpzrobots::OsgHandle& osgHandle;
    lpzrobots::XMLSimulation* simulation;
    bool validateXML;

    XMLPrimitiveFactory* primitiveFactory;


    void parseGlobalVariables(XERCESC::DOMNode* globalVariablesNode);
};

#endif /* __XMLPARSERENGINE_H_ */
