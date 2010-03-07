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
 *  Revision 1.1  2010-03-07 22:50:38  guettler
 *  first development state for feature XMLImport
 *										   *
 *                                                                         *
 **************************************************************************/

#include "XMLBoundingShape.h"
#include "XMLHelper.h"
#include "XMLDefinitions.h"
#include <xercesc/dom/DOMNode.hpp>

#include <ode_robots/mathutils.h>

using namespace xercesc_3_1;

using namespace lpzrobots;
using namespace osg;

XMLBoundingShape::XMLBoundingShape(const xercesc_3_1::DOMNode* boundingBoxNode, XMLParserEngine& engine, lpzrobots::Primitive* parent)
: BoundingShape("", parent), XMLObject(xmlEngine), boundingBoxNode(boundingBoxNode) {
  // TODO Auto-generated constructor stub

}

XMLBoundingShape::~XMLBoundingShape() {
  // TODO Auto-generated destructor stub
}


bool XMLBoundingShape::init(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle,
      double scale, char mode) {
  for EACHCHILDNODE(boundingBoxNode, node) {
    if (node->getNodeType() == DOMNode::ELEMENT_NODE) {
      Primitive* primitive = 0;
      if (XMLHelper::matchesName(node,XMLDefinitions::BoxNode))
        primitive = new Box(VALOFNODE(node, XMLDefinitions::lengthAtt) * scale, VALOFNODE(node, XMLDefinitions::widthAtt) * scale, VALOFNODE(node, XMLDefinitions::heightAtt) * scale);
      else if(XMLHelper::matchesName(node,XMLDefinitions::SphereNode))
        primitive = new Sphere(VALOFNODE(node,XMLDefinitions::radiusAtt));
      else if(XMLHelper::matchesName(node,XMLDefinitions::CylinderNode))
        primitive = new Cylinder(VALOFNODE(node,XMLDefinitions::radiusAtt) * scale, VALOFNODE(node,XMLDefinitions::heightAtt) * scale);
      else if(XMLHelper::matchesName(node,XMLDefinitions::CapsuleNode))
        primitive = new Capsule(VALOFNODE(node,XMLDefinitions::radiusAtt) * scale, VALOFNODE(node,XMLDefinitions::heightAtt) * scale);
      if (primitive!=0) {
        const Vec3 rot = XMLHelper::getRotation(node);
        const Vec3 pos = XMLHelper::getPosition(node);
        Primitive* Trans = new lpzrobots::Transform(parent, primitive, osgRotate(rot[0]*M_PI/180.0f,rot[1]*M_PI/180.0f,rot[2]*M_PI/180.0f)
                 *osg::Matrix::translate(scale*pos[0],scale*pos[1],scale*pos[2]));
        Trans->init(odeHandle, 0, osgHandle.changeColor(Color(1.0,0,0,0.3)),mode);
        active = true;
        std::cout << "Primitive for BoundingShape created!" << std::endl;
      }
    }
  }
  return active;
}
