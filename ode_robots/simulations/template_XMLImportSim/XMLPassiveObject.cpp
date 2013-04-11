/*
 * XMLPassiveObject.cpp
 *
 *  Created on: 03.02.2010
 *      Author: robot3
 */

#include "XMLPassiveObject.h"
#include <xercesc/dom/DOMNode.hpp>
#include <ode_robots/playground.h>
#include <ode_robots/osgprimitive.h>

#include "XMLHelper.h"

using namespace XERCESC;
using namespace std;
using namespace osg;
using namespace lpzrobots;

XMLPassiveObject::XMLPassiveObject(DOMNode* passiveObjectNode, XMLParserEngine& xmlEngine)
: AbstractObstacle(xmlEngine.getOdeHandle(), xmlEngine.getOsgHandle()), XMLObject(xmlEngine), passiveObjectNode(passiveObjectNode) {

}

XMLPassiveObject::~XMLPassiveObject() { /* obstacle is destroyed by AbstractObstacle */ }

void XMLPassiveObject::setPose(const osg::Matrix & pose)
{
  if (!obstacle_exists)
    create();
        this->obst[0]->setPose(pose);
}



void XMLPassiveObject::create()
{
        if (obstacle_exists)
                destroy();
        this->obst.push_back(this->xmlEngine.getPrimitiveFactory()->createPrimitive(passiveObjectNode));
        obstacle_exists = true;
}



Primitive* XMLPassiveObject::getMainPrimitive() const
{
  assert(obstacle_exists);
        return obst[0];
}


