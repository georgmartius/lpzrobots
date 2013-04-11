/*
 * XMLPassiveObject.h
 *
 *  Created on: 03.02.2010
 *      Author: robot3
 */

#ifndef XMLPASSIVEOBJECT_H_
#define XMLPASSIVEOBJECT_H_

#include <XercescForwardDecl.h>
#include <ode_robots/abstractobstacle.h>
#include "XMLObject.h"



class XMLPassiveObject: public lpzrobots::AbstractObstacle, public XMLObject {
public:
        XMLPassiveObject(XERCESC::DOMNode* passiveObjectNode, XMLParserEngine& xmlEngine);
        virtual ~XMLPassiveObject();

          /**
           * sets position of the obstacle and creates/recreates obstacle if necessary
           */
          virtual void setPose(const osg::Matrix& pose);

          /// return the "main" primitive of the obtactle. The meaning of "main" is arbitrary
          virtual lpzrobots::Primitive* getMainPrimitive() const;

          /// overload this function to create the obstactle. All primitives should go into the list "obst"
          virtual void create();

protected:
          XERCESC::DOMNode* passiveObjectNode;
};

#endif /* XMLPASSIVEOBJECT_H_ */
