/*
 * XMLObject.h
 *
 *  Created on: 03.02.2010
 *      Author: robot3
 */

#ifndef XMLOBJECT_H_
#define XMLOBJECT_H_

#include "XMLParserEngine.h"

class XMLObject {
public:
        XMLObject(XMLParserEngine& xmlEngine) : xmlEngine(xmlEngine) {}

        virtual ~XMLObject();

protected:
        XMLParserEngine& xmlEngine;
};

#endif /* XMLOBJECT_H_ */
