/*
 * XMLPassiveObject.cpp
 *
 *  Created on: 03.02.2010
 *      Author: robot3
 */

#include "XMLPassiveObject.h"
#include <xercesc/dom/DOMNode.hpp>
#include <ode_robots/playground.h>

#include "XMLHelper.h"

using namespace xercesc_3_1;
using namespace std;
using namespace osg;
using namespace lpzrobots;

XMLPassiveObject::XMLPassiveObject(DOMNode* passiveObjectNode, XMLParserEngine& xmlEngine)
: AbstractObstacle(xmlEngine.getOdeHandle(), xmlEngine.getOsgHandle()), XMLObject(xmlEngine) {
	if (XMLHelper::matchesName(passiveObjectNode,"Playground"))
	{
		cout << "Playground found!" << endl;

		//XMLHelper::getColor(passiveObjectNode);
		//XMLHelper::getPosition(passiveObjectNode);
/*
		for EACHCHILDNODE(passiveObjectNode,nodeOfPlayground)
		{
			cout << "   found: " << C(nodeOfPlayground->getNodeName()) << endl;


			if(XMLHelper::matchesName(nodeOfPlayground,"Color"))
			{
				cout << "Color found 1" << endl;
				XMLHelper::getColor(nodeOfPlayground);
				cout << XMLHelper::getChildNodeValue(nodeOfPlayground,"red")<< endl;
				cout << XMLHelper::getChildNodeValue(nodeOfPlayground,"alpha")<< endl;
				cout << XMLHelper::getChildNodeValue(nodeOfPlayground,"green")<< endl;
				cout << XMLHelper::getChildNodeValue(nodeOfPlayground,"blue")<< endl;
			}
			//cout << "   found: " << C(nodeOfPlayground->getAttributes()->getNamedItem(X("red"))->getNodeValue());
		}

		cout << passiveObjectNode << endl;*/
		Playground* playground = new Playground(odeHandle, osgHandle.changeColor(XMLHelper::getColor(passiveObjectNode)), XMLHelper::getGeometry(passiveObjectNode));
		playground->setPosition(XMLHelper::getPosition(passiveObjectNode));
		xmlEngine.getGlobalData().obstacles.push_back(playground);
	} else{
		//cout << "   Passifobject found: " << C(nodeOfPassiveObject->getNodeName()) << endl;
		this->obst.push_back(this->xmlEngine.getPrimitiveFactory()->createPrimitive(passiveObjectNode));

		//return;
	}
}

XMLPassiveObject::~XMLPassiveObject() {
	// TODO Auto-generated destructor stub
}

void XMLPassiveObject::setPose(const osg::Matrix & pose)
{
}



void XMLPassiveObject::create()
{
}



Primitive *XMLPassiveObject::getMainPrimitive() const
{
}


