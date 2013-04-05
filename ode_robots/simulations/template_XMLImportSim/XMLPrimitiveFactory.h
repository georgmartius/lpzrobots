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
 *  Factory which generates a lpzRobots::Primitive from a xercesc::DOMNode *
 *  holding all necessary information.                                     *
 *  This is a simple version of an concrete factory without an abstract    *
 *  factory.                                                               *
 *                                                                         *
 *  $Log$
 *  Revision 1.3  2010-06-15 15:02:19  guettler
 *  using now "XercescForwardDecl.h" to avoid namespace problems (3_0, 3_1)
 *
 *  Revision 1.2  2010/05/20 10:38:20  guettler
 *  - setMaterial for BoundingShape now allowed
 *  - static Mesh (mass=0) should work
 *
 *  Revision 1.1  2010/03/07 22:50:38  guettler
 *  first development state for feature XMLImport
 *                                                                                   *
 *                                                                         *
 **************************************************************************/
#ifndef __XMLPRIMITIVEFACTORY_H_
#define __XMLPRIMITIVEFACTORY_H_

//#include <xercesc/dom/DOMNode.hpp>
#include <ode_robots/abstractobstacle.h>
#include "XercescForwardDecl.h"




class XMLParserEngine;
namespace lpzrobots {
  class Primitive;
  class Box;
  class Capsule;
  class Cylinder;
  class DummyPrimitive;
  class HeightField;
  class Mesh;
  class Plane;
  class Ray;
  class Sphere;
  class Transform;
  class GlobalData;
  class OdeHandle;
  class OsgHandle;
}


/**
 * Factory which generates a lpzRobots::Primitive from a xercesc::DOMNode holding all
 * necessary information.
 *
 * Furthermore the created Primitives are stored in a map with key=id and value = Primitive
 * to obtain the Primitive by id
 */
class XMLPrimitiveFactory {
  public:
    XMLPrimitiveFactory(XMLParserEngine* engine, lpzrobots::GlobalData& globalData,
        const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle);
    virtual ~XMLPrimitiveFactory();

    /**
     * This method delegates the call to the specialized methods according to
     * the name of the primitiveNode
     * @param primitiveNode the node which describes the primitive to be generated
     * @return A Primitive described by the given DOMNode
     * @see lpzrobots::Primitive
     */
    lpzrobots::Primitive* createPrimitive(XERCESC::DOMNode* primitiveNode);

    /**
     *
     * @param boxNode the node which describes the Box to be generated
     * @return A Box described by the given DOMNode
     * @see lpzrobots::Box
     */
    lpzrobots::Box* createBox(XERCESC::DOMNode* boxNode);

    /**
     *
     * @param capsuleNode the node which describes the Capsule to be generated
     * @return A Capsule described by the given DOMNode
     * @see lpzrobots::Capsule
     */
    lpzrobots::Capsule* createCapsule(XERCESC::DOMNode* capsuleNode);

    /**
     *
     * @param cylinderNode the node which describes the Cylinder to be generated
     * @return A Cylinder described by the given DOMNode
     * @see lpzrobots::Cylinder
     */
    lpzrobots::Cylinder* createCylinder(XERCESC::DOMNode* cylinderNode);

    /**
     *
     * @param dummyPrimitiveNode the node which describes the DummyPrimitive to be generated
     * @return A DummyPrimitive described by the given DOMNode
     * @see lpzrobots::DummyPrimitive
     */
    lpzrobots::DummyPrimitive* createDummyPrimitive(XERCESC::DOMNode* dummyPrimitiveNode);

    /**
     *
     * @param meshNode the node which describes the Mesh to be generated
     * @return A Mesh described by the given DOMNode
     * @see lpzrobots::Mesh
     */
    lpzrobots::Mesh* createMesh(XERCESC::DOMNode* meshNode);

    /**
     *
     * @param planeNode the node which describes the Plane to be generated
     * @return A Plane described by the given DOMNode
     * @see lpzrobots::Plane
     */
    lpzrobots::Plane* createPlane(XERCESC::DOMNode* planeNode);

    /**
     *
     * @param rayNode the node which describes the Ray to be generated
     * @return A Ray described by the given DOMNode
     * @see lpzrobots::Ray
     */
    lpzrobots::Ray* createRay(XERCESC::DOMNode* rayNode);

    /**
     *
     * @param sphereNode the node which describes the Sphere to be generated
     * @return A Sphere described by the given DOMNode
     * @see lpzrobots::Sphere
     */
    lpzrobots::Sphere* createSphere(XERCESC::DOMNode* sphereNode);

    /**
     *
     * @param transformNode the node which describes the Transform to be generated
     * @return A Transform described by the given DOMNode
     * @see lpzrobots::Transform
     */
    lpzrobots::Transform* createTransform(XERCESC::DOMNode* transformNode);

    /**
     *
     * @param heightFieldNode the node which describes the HeightField to be generated
     * @return A HeightField described by the given DOMNode
     * @see lpzrobots::HeightField
     */
    lpzrobots::HeightField* createHeightField(XERCESC::DOMNode* heightFieldNode);

    static void setMaterial(const XERCESC::DOMNode* node, lpzrobots::Primitive* primitive);

  private:
    XMLParserEngine* engine;
    lpzrobots::GlobalData& globalData;
    const lpzrobots::OdeHandle& odeHandle;
    const lpzrobots::OsgHandle& osgHandle;

    /**
     * Sets the texture at the primitive if the corresponding textureNode is present.
     * The texture info is encoded by:
     * <texture file="filename.jpg" repeatOnR="2" repeatOnS="1" surface="1"/>
     * Assigns a texture to the x-th surface of the primitive,
     * you can choose how often to repeat negative values of repeat correspond
     * to length units. E.g. assume a rectangle of size 5 in x direction:
     * with repeatOnX = 2 the texture would be two times repeated.
     * With repeatOnX = -1 the texture would be 5 times repeated
     * because the texture is made to have the size 1.
     * @param node parent node which contains the textureNode
     */
    static void setTextureIfPresent(XERCESC::DOMNode* node, lpzrobots::Primitive* primitive);
    static char getPrimitiveMode(XERCESC::DOMNode* node);
};

#endif /* __XMLPRIMITIVEFACTORY_H_ */
