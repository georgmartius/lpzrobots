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
 *  Revision 1.5  2010-06-15 15:02:19  guettler
 *  using now "XercescForwardDecl.h" to avoid namespace problems (3_0, 3_1)
 *
 *  Revision 1.4  2010/05/20 10:38:20  guettler
 *  - setMaterial for BoundingShape now allowed
 *  - static Mesh (mass=0) should work
 *
 *  Revision 1.3  2010/03/11 15:18:06  guettler
 *  -BoundingShape can now be set from outside (see XMLBoundingShape)
 *  -Mesh can be created without Body and Geom.
 *  -various bugfixes
 *
 *  Revision 1.2  2010/03/10 13:54:59  guettler
 *  further developments for xmlimport
 *
 *  Revision 1.1  2010/03/07 22:50:38  guettler
 *  first development state for feature XMLImport
 *                                                                                   *
 *                                                                         *
 **************************************************************************/

#include "XMLBoundingShape.h"
#include "XMLHelper.h"
#include "XMLDefinitions.h"
#include <xercesc/dom/DOMNode.hpp>

#include <ode_robots/mathutils.h>

using namespace XERCESC;

using namespace lpzrobots;
using namespace osg;

XMLBoundingShape::XMLBoundingShape(const DOMNode* boundingBoxNode, XMLParserEngine& engine, lpzrobots::Mesh* parent)
: BoundingShape("", parent), XMLObject(xmlEngine), boundingBoxNode(boundingBoxNode) {
  // TODO Auto-generated constructor stub

}

XMLBoundingShape::~XMLBoundingShape() {
  // TODO Auto-generated destructor stub
}


bool XMLBoundingShape::init(const lpzrobots::OdeHandle& _odeHandle, const lpzrobots::OsgHandle& osgHandle,
      double scale, char mode) {
  odeHandle = OdeHandle(_odeHandle);
  parentSpace = odeHandle.space;
  odeHandle.createNewSimpleSpace(parentSpace,true);
  char primitiveMode = mode & ~Primitive::Body; // never create any body for the primitives (TODO: compound body)
  if (!(mode & Primitive::Body)) {
    attachedToParentBody = false;
  }
  for EACHCHILDNODE(boundingBoxNode, node) {
    if (node->getNodeType() == DOMNode::ELEMENT_NODE) {
      Primitive* primitive = 0;
      if (XMLHelper::matchesName(node,XMLDefinitions::boxNode))
        primitive = new Box(VALOFNODE(node, XMLDefinitions::lengthAtt) * scale, VALOFNODE(node, XMLDefinitions::widthAtt) * scale, VALOFNODE(node, XMLDefinitions::heightAtt) * scale);
      else if(XMLHelper::matchesName(node,XMLDefinitions::sphereNode))
        primitive = new Sphere(VALOFNODE(node,XMLDefinitions::radiusAtt));
      else if(XMLHelper::matchesName(node,XMLDefinitions::cylinderNode))
        primitive = new Cylinder(VALOFNODE(node,XMLDefinitions::radiusAtt) * scale, VALOFNODE(node,XMLDefinitions::heightAtt) * scale);
      else if(XMLHelper::matchesName(node,XMLDefinitions::capsuleNode))
        primitive = new Capsule(VALOFNODE(node,XMLDefinitions::radiusAtt) * scale, VALOFNODE(node,XMLDefinitions::heightAtt) * scale);
      if (primitive!=0) {
        XMLPrimitiveFactory::setMaterial(boundingBoxNode, primitive);
        XMLPrimitiveFactory::setMaterial(node,primitive);
        const Vec3 rot = XMLHelper::getRotation(node);
        const Vec3 pos = XMLHelper::getPosition(node);
        if (mode & Primitive::Body) {  // use Transforms to attach the Primitives to the body
          std::cout << "BoundingShape body mode!" << std::endl;
          Primitive* Trans = new lpzrobots::Transform(parent, primitive, osgRotate(rot[0]*M_PI/180.0f,rot[1]*M_PI/180.0f,rot[2]*M_PI/180.0f)
                   *osg::Matrix::translate(scale*pos[0],scale*pos[1],scale*pos[2]));
          Trans->init(odeHandle, 0, osgHandle.changeColor(Color(1.0,0,0,0.3)),primitiveMode);
        }
        else {
          std::cout << "BoundingShape geom only mode!" << std::endl;
          primitive->init(odeHandle, 0, osgHandle.changeColor(Color(1.0,0,0,0.3)), primitiveMode);
          boundingPrimitiveList.push_back(primitive);
          boundingPrimitivePoseList.push_back(osgRotate(rot[0]*M_PI/180.0f,rot[1]*M_PI/180.0f,rot[2]*M_PI/180.0f)
                   *osg::Matrix::translate(scale*pos[0],scale*pos[1],scale*pos[2]));
        }
        active = true;
        std::cout << "Primitive for BoundingShape created!" << std::endl;
      }
    }
  }
  return active;
}
