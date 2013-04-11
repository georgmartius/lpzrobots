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
 *  Revision 1.6  2011-05-31 10:21:48  martius
 *  make xml stuff work again
 *  moved obsolete stuff
 *
 *  Revision 1.5  2010/06/15 15:02:19  guettler
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

#include "XMLPrimitiveFactory.h"
#include "XMLHelper.h"
#include "XMLErrorHelper.h"
#include "XMLDefinitions.h"
#include "XMLBoundingShape.h"

#include "XMLParserEngine.h"

#include <ode_robots/primitive.h>
#include <ode_robots/heightfieldprimitive.h>
#include <ode_robots/globaldata.h>
#include <ode_robots/odehandle.h>
#include <ode_robots/osghandle.h>
#include <ode_robots/passivemesh.h>
#include <ode_robots/osgprimitive.h>

using namespace lpzrobots;
using namespace XERCESC;
using namespace std;

XMLPrimitiveFactory::XMLPrimitiveFactory(XMLParserEngine* engine, GlobalData& globalData, const OdeHandle& odeHandle,
    const OsgHandle& osgHandle) :
  engine(engine), globalData(globalData), odeHandle(odeHandle), osgHandle(osgHandle) {
}

XMLPrimitiveFactory::~XMLPrimitiveFactory() {
}

Primitive* XMLPrimitiveFactory::createPrimitive(DOMNode* primitiveNode) {
  if (XMLHelper::matchesName(primitiveNode, XMLDefinitions::boxNode))
    return createBox(primitiveNode);
  else if (XMLHelper::matchesName(primitiveNode, XMLDefinitions::capsuleNode))
    return createCapsule(primitiveNode);
  else if (XMLHelper::matchesName(primitiveNode, XMLDefinitions::cylinderNode))
    return createCylinder(primitiveNode);
  else if (XMLHelper::matchesName(primitiveNode, XMLDefinitions::dummyPrimitiveNode))
    return createDummyPrimitive(primitiveNode);
  else if (XMLHelper::matchesName(primitiveNode, XMLDefinitions::meshNode))
    return createMesh(primitiveNode);
  else if (XMLHelper::matchesName(primitiveNode, XMLDefinitions::planeNode))
    return createPlane(primitiveNode);
  else if (XMLHelper::matchesName(primitiveNode, XMLDefinitions::rayNode))
    return createRay(primitiveNode);
  else if (XMLHelper::matchesName(primitiveNode, XMLDefinitions::sphereNode))
    return createSphere(primitiveNode);
  else if (XMLHelper::matchesName(primitiveNode, XMLDefinitions::transformNode))
    return createTransform(primitiveNode);
  else
    XMLErrorHelper::printError(string("createPrimitive failed - no Implementation found for \n").append(
        C(primitiveNode->getNodeName())));
  return new DummyPrimitive();
}

Box* XMLPrimitiveFactory::createBox(DOMNode* boxNode) {
  // don't check if node matches "Box"
  /* <box length="3" width="3" height="3" weight="2" scale="1.0">
   <position x="24.54" y="3.23" z="2.342"/>
   <rotation alpha="44.345534" beta="90.354544" gamma="-135.366342"/>
   </box> */
  cout << "Box found!" << endl;
  cout << "  Length found " << XMLHelper::getNodeAtt(boxNode, XMLDefinitions::lengthAtt) << endl;
  cout << "  Heigth found " << XMLHelper::getNodeAtt(boxNode, XMLDefinitions::heightAtt) << endl;
  cout << "  Width found " << XMLHelper::getNodeAtt(boxNode, XMLDefinitions::widthAtt) << endl;
  cout << "  Mass found " << XMLHelper::getNodeAtt(boxNode, XMLDefinitions::massAtt) << endl;

  Box* box = new Box(XMLHelper::getNodeAtt(boxNode, XMLDefinitions::lengthAtt), XMLHelper::getNodeAtt(boxNode,
      XMLDefinitions::widthAtt), XMLHelper::getNodeAtt(boxNode, XMLDefinitions::heightAtt));
  box->init(odeHandle, VALOFNODE(boxNode, XMLDefinitions::massAtt), osgHandle.changeColor(XMLHelper::getColor(boxNode)), getPrimitiveMode(boxNode));// the mass of the mesh
  setTextureIfPresent(boxNode, box);
  setMaterial(boxNode, box);
  box->setPose(XMLHelper::getPose(boxNode));

  return box;
}

Capsule* XMLPrimitiveFactory::createCapsule(DOMNode* capsuleNode) {
  cout << "Capsule found!" << endl;
  cout << "  Radius found " << XMLHelper::getNodeAtt(capsuleNode, XMLDefinitions::radiusAtt) << endl;
  cout << "  Heigth found " << XMLHelper::getNodeAtt(capsuleNode, XMLDefinitions::heightAtt) << endl;
  cout << "  Mass found " << XMLHelper::getNodeAtt(capsuleNode, XMLDefinitions::massAtt) << endl;

  Capsule* capsule = new Capsule(XMLHelper::getNodeAtt(capsuleNode, XMLDefinitions::radiusAtt), XMLHelper::getNodeAtt(
      capsuleNode, XMLDefinitions::heightAtt));
  capsule->init(odeHandle, XMLHelper::getNodeAtt(capsuleNode, XMLDefinitions::massAtt), osgHandle.changeColor(XMLHelper::getColor(capsuleNode)), getPrimitiveMode(
      capsuleNode));// the mass of the mesh
  setTextureIfPresent(capsuleNode, capsule);
  setMaterial(capsuleNode, capsule);
  capsule->setPose(XMLHelper::getPose(capsuleNode));
  return capsule;
}

Cylinder* XMLPrimitiveFactory::createCylinder(DOMNode* cylinderNode) {
  cout << "Cylinder found!" << endl;
  cout << "  Radius found " << XMLHelper::getNodeAtt(cylinderNode, XMLDefinitions::radiusAtt) << endl;
  cout << "  Height found " << XMLHelper::getNodeAtt(cylinderNode, XMLDefinitions::heightAtt) << endl;
  cout << "  Mass found " << XMLHelper::getNodeAtt(cylinderNode, XMLDefinitions::massAtt) << endl;

  Cylinder* cylinder = new Cylinder(XMLHelper::getNodeAtt(cylinderNode, XMLDefinitions::radiusAtt),
      XMLHelper::getNodeAtt(cylinderNode, XMLDefinitions::heightAtt));
  setTextureIfPresent(cylinderNode, cylinder);
  setMaterial(cylinderNode, cylinder);
  cylinder->init(odeHandle, XMLHelper::getNodeAtt(cylinderNode, XMLDefinitions::massAtt), osgHandle.changeColor(XMLHelper::getColor(cylinderNode)), getPrimitiveMode(cylinderNode));// the mass of the mesh
  cylinder->setPose(XMLHelper::getPose(cylinderNode));
  return cylinder;

  /*Cylinder* cylinder = new Cylinder(VALOFCHILD(cylinderNode, XMLDefinitions::radiusAtt),
   VALOFCHILD(cylinderNode, XMLDefinitions::heightAtt));
   cylinder->setPosition(XMLHelper::getPosition(cylinderNode));
   cylinder->init(odeHandle, VALOFCHILD(cylinderNode, XMLDefinitions::weightAtt), osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
   cylinder->setPose(XMLHelper::getPose(cylinderNode));
   setTextureIfPresent(cylinderNode, cylinder);
   return cylinder;*/
}

DummyPrimitive* XMLPrimitiveFactory::createDummyPrimitive(DOMNode* dummyPrimitiveNode) {
  cout << "Dummy Primitive found!" << endl;
  /*DummyPrimitive* dummyPrimitive = new DummyPrimitive();
   dummyPrimitive->setPosition(XMLHelper::getPosition(dummyPrimitiveNode));
   dummyPrimitive->init(odeHandle, VALOFCHILD(dummyPrimitiveNode, XMLDefinitions::weightAtt), osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
   dummyPrimitive->setPose(XMLHelper::getPose(dummyPrimitiveNode));
   setTextureIfPresent(dummyPrimitiveNode, dummyPrimitive);
   return dummyPrimitive;*/
}

Mesh* XMLPrimitiveFactory::createMesh(DOMNode* meshNode) {
  cout << "Mesh found!" << endl;
  cout << "  Filename found "
      << VALOFCHILDASSTRING(meshNode,XMLDefinitions::graphicalRepresentationNode, XMLDefinitions::fileAtt) << endl;
  cout << "  Scale found " << XMLHelper::getChildNodeValue(meshNode, XMLDefinitions::graphicalRepresentationNode,
      XMLDefinitions::scaleAtt, 1.0) << endl;
  cout << "  Mass found " << VALOFCHILD(meshNode,XMLDefinitions::boundingShapeNode, XMLDefinitions::massAtt) << endl;

  Mesh* mesh =
      new Mesh(VALOFCHILDASSTRING(meshNode,XMLDefinitions::graphicalRepresentationNode, XMLDefinitions::fileAtt), // the filename of the mesh
          XMLHelper::getChildNodeValue(meshNode, XMLDefinitions::graphicalRepresentationNode, XMLDefinitions::scaleAtt,
              1.0));// the scale factor to be used
  // always do not let create the BoundingShape (by .bbox) (so don't use mode Primitive::Geom)
  char primitiveMode = 0;
  double mass = XMLHelper::getChildNodeValue(meshNode, XMLDefinitions::boundingShapeNode, XMLDefinitions::massAtt, 1.0);
  double visible = XMLHelper::getChildNodeValue(meshNode, XMLDefinitions::graphicalRepresentationNode, XMLDefinitions::visibleAtt, 1.0);
  if (visible)
    primitiveMode |= Primitive::Draw;
  // if mass == 0, don't create a body.
  // Then the BoundingShape must not use Transforms to attach the Primitives to the body,
  // the Primitives have to be generated static (without body) and grouped into one space.
  if (mass>=XMLDefinitions::compareEPS)
    primitiveMode|=Primitive::Body;
  // Geom is "created" by the XMLBoundingShape
  mesh->init(odeHandle, mass, osgHandle.changeColor(XMLHelper::getColor(XMLHelper::getChildNode(meshNode, XMLDefinitions::graphicalRepresentationNode))), primitiveMode);
  //setTextureIfPresent(meshNode, mesh);
  //setMaterial(meshNode, mesh);
  // create BoundingShape
  primitiveMode = Primitive::Geom;
  if (osgHandle.drawBoundings)
    primitiveMode|= Primitive::Draw;
  if (mass >= XMLDefinitions::compareEPS)
    primitiveMode|= Primitive::Body; // if mesh has a body, the geoms need to have be attached to the body
  XMLBoundingShape* boundingShape = new XMLBoundingShape(XMLHelper::getChildNode(meshNode, XMLDefinitions::boundingShapeNode),*(this->engine),mesh);
  boundingShape->init(odeHandle, osgHandle, XMLHelper::getChildNodeValue(meshNode, XMLDefinitions::graphicalRepresentationNode, XMLDefinitions::scaleAtt, 1.0), primitiveMode);
  mesh->setPose(XMLHelper::getPose(meshNode));
  return mesh;

  /*PassiveMesh* mesh = new PassiveMesh(odeHandle,osgHandle,
   VALOFCHILDASSTRING(meshNode,XMLDefinitions::GraphicalRepresentationNode, XMLDefinitions::fileAtt),
   XMLHelper::getChildNodeValue(meshNode, XMLDefinitions::GraphicalRepresentationNode, XMLDefinitions::scaleAtt,1.0),
   VALOFCHILDASSTRING(meshNode,XMLDefinitions::BoundingShapeNode, XMLDefinitions::mass));
   mesh->setPosition(osg::Vec3(1.0,0.2,1.0f));*/
   //mesh->setPosition(XMLHelper::getPosition(meshNode);
   //mesh->init(odeHandle, VALOFCHILD(meshNode, XMLDefinitions::weightAtt), osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
   //mesh->setPose(XMLHelper::getPose(meshNode));
   // setTextureIfPresent(meshNode, mesh);
   // return mesh;
   /* example
   <GraphicalRepresentation>
   <file>cow.osg</file>
   <color>
   <red>155</red>
   <green>099</green>
   <blue>255</blue>
   <alpha>0</alpha>
   </color>
   <texture>
   <file>myTexture.osg</file>
   <wrapTexture>yes</wrapTexture>
   </texture>
   </GraphicalRepresentation>
   <BoundingShape>
   <sphere>
   <position>
   <x>24.234235</x>
   <y>3.234234</y>
   <z>2.2323525</z>
   </position>
   <rotation>
   <alpha>44.345534</alpha>
   <beta>90.354544</beta>
   <gamma>-135.366342</gamma>
   </rotation>
   <radius>3</radius>
   </sphere>
   </BoundingBox>
   <position>
   <x>24.234235</x>
   <y>3.234234</y>
   <z>2.2323525</z>
   </position>
   <rotation>
   <x>24.234235</x>
   <y>3.234234</y>
   <z>2.2323525</z>
   </rotation>
   <scale>1.50000</scale>
   */

  // a Mesh consists of a graphical and a physical representation.
  // The physical representation is normally parsed by the class
  // lpzrobots::BoundingShape. Here the same stuff is done.

  // create graphical representation
  for EACHCHILDNODE(meshNode, childNode) {

  }
}

Plane* XMLPrimitiveFactory::createPlane(DOMNode* planeNode) {
  cout << "Plane found!" << endl;
  /*Plane* plane = new Plane();
   plane->setPosition(XMLHelper::getPosition(planeNode));
   plane->init(odeHandle, VALOFCHILD(planeNode, XMLDefinitions::weightAtt), osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
   plane->setPose(XMLHelper::getPose(planeNode));
   setTextureIfPresent(planeNode, plane);
   return plane;*/
}

Ray* XMLPrimitiveFactory::createRay(DOMNode* rayNode) {
  cout << "Ray found!" << endl;
  /*Ray* ray = new Ray(VALOFCHILD(rayNode, XMLDefinitions::rangeAtt),
   VALOFCHILD(rayNode, XMLDefinitions::thicknessAtt),VALOFCHILD(rayNode, XMLDefinitions::rangeAtt));
   ray->setPosition(XMLHelper::getPosition(rayNode));
   ray->init(odeHandle, VALOFCHILD(rayNode, XMLDefinitions::weightAtt), osgHandle, Primitive::Geom | Primitive::Draw);
   ray->setPose(XMLHelper::getPose(rayNode));
   setTextureIfPresent(rayNode, ray);
   return ray;*/
}

Sphere* XMLPrimitiveFactory::createSphere(DOMNode* sphereNode) {
  cout << "Sphere found!" << endl;
  cout << "  Radius found " << XMLHelper::getNodeAtt(sphereNode, XMLDefinitions::radiusAtt) << endl;
  cout << "  Mass found " << XMLHelper::getNodeAtt(sphereNode, XMLDefinitions::massAtt) << endl;

  Sphere* sphere = new Sphere(XMLHelper::getNodeAtt(sphereNode, XMLDefinitions::radiusAtt));
  sphere->init(odeHandle, XMLHelper::getNodeAtt(sphereNode, XMLDefinitions::massAtt), osgHandle.changeColor(XMLHelper::getColor(sphereNode)), getPrimitiveMode(sphereNode));// the mass of the mesh
  setTextureIfPresent(sphereNode, sphere);
  setMaterial(sphereNode, sphere);
  //sphere->setTexture(VALOFCHILDASSTRING(sphereNode,XMLDefinitions::texture,XMLDefinitions::fileAtt));
  sphere->setPose(XMLHelper::getPose(sphereNode));
  return sphere;
  /*Sphere* sphere = new Sphere(VALOFCHILD(sphereNode, XMLDefinitions::radiusAtt));
   sphere->setPosition(XMLHelper::getPosition(sphereNode));
   sphere->init(odeHandle, VALOFCHILD(sphereNode, XMLDefinitions::weightAtt), osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
   sphere->setPose(XMLHelper::getPose(sphereNode));
   setTextureIfPresent(sphereNode, sphere);
   return sphere;*/
}

Transform* XMLPrimitiveFactory::createTransform(DOMNode* transformNode) {
  cout << "Transform found!" << endl;
  /*Transform* transform = new Transform(VALOFCHILD(transformNode, XMLDefinitions::parentPrimitive),
   VALOFCHILD(transformNode, XMLDefinitions::childPrimitive),VALOFCHILD(transformNode, XMLDefinitions::scaleAtt));  //Richtig??
   transform->setPosition(XMLHelper::getPosition(transformNode));
   transform->init(odeHandle, VALOFCHILD(transformNode, XMLDefinitions::weightAtt), osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
   transform->setPose(XMLHelper::getPose(transformNode));
   setTextureIfPresent(transformNode, transform);
   return transform;*/
  /*return new Transform(new DummyPrimitive(), new DummyPrimitive(), osg::Matrix::translate(osg::Vec3f(0,0,0)));
   */}

HeightField* XMLPrimitiveFactory::createHeightField(DOMNode* heightFieldNode) {
  cout << "Heightfield found!" << endl;
  /*HeightField* heightField = new HeightField(VALOFCHILDASSTRING(heightFieldNode, XMLDefinitions::fileAtt),
   VALOFCHILD(heightFieldNode, XMLDefinitions::xAtt),VALOFCHILD(heightFieldNode, XMLDefinitions::yAtt),
   VALOFCHILD(heightFieldNode, XMLDefinitions::heightAtt));
   heightField->setPosition(XMLHelper::getPosition(heightFieldNode));
   heightField->init(odeHandle, VALOFCHILD(heightFieldNode, XMLDefinitions::weightAtt), osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
   heightField->setPose(XMLHelper::getPose(heightFieldNode));
   setTextureIfPresent(heightFieldNode, heightField);
   return heightField;*/
}

void XMLPrimitiveFactory::setTextureIfPresent(DOMNode* node, Primitive* primitive) {

  if (CHILDOFNODE(node,XMLDefinitions::texture) != 0) {
    cout << "Textureset found!" << endl;
    string fileName = VALOFCHILDASSTRING(node,XMLDefinitions::texture,XMLDefinitions::fileAtt);
    double repeatOnR = VALOFCHILD(node,XMLDefinitions::texture,XMLDefinitions::repeatOnRAtt);
    double repeatOnS = VALOFCHILD(node,XMLDefinitions::texture,XMLDefinitions::repeatOnSAtt);
    double surface = VALOFCHILD(node,XMLDefinitions::texture,XMLDefinitions::surfaceAtt);
    cout << "   Surface " << surface << endl;
    cout << "   Filename " << fileName << endl;
    cout << "   RepeatOnR " << repeatOnR << endl;
    cout << "   RepeatOnS " << repeatOnS << endl;
    primitive->setTexture((int) surface, TextureDescr(fileName, repeatOnR, repeatOnS));
  }
  // else do nothing
}

void XMLPrimitiveFactory::setMaterial(const DOMNode* node, Primitive* primitive) {

  if (CHILDOFNODE(node,XMLDefinitions::materialNode) != 0) {
    cout << "Material found!" << endl;
    cout << "  Elasticity " << VALOFCHILDASSTRING(node,XMLDefinitions::materialNode,XMLDefinitions::elasticityAtt)
        << endl;
    cout << "  Hardness " << VALOFCHILDASSTRING(node,XMLDefinitions::materialNode,XMLDefinitions::hardnessAtt) << endl;
    cout << "  Roughness " << VALOFCHILDASSTRING(node,XMLDefinitions::materialNode,XMLDefinitions::roughnessAtt)
        << endl;
    cout << "  Slip " << VALOFCHILDASSTRING(node,XMLDefinitions::materialNode,XMLDefinitions::slipAtt) << endl;
    // TODO: get correct substance from xml and set!
    primitive->setSubstance(Substance::getDefaultSubstance());
    primitive->substance.elasticity = VALOFCHILD(node,XMLDefinitions::materialNode,XMLDefinitions::elasticityAtt);
    primitive->substance.hardness = VALOFCHILD(node,XMLDefinitions::materialNode,XMLDefinitions::hardnessAtt);
    primitive->substance.roughness = VALOFCHILD(node,XMLDefinitions::materialNode,XMLDefinitions::roughnessAtt);
    primitive->substance.slip = VALOFCHILD(node,XMLDefinitions::materialNode,XMLDefinitions::slipAtt);
  }

}

/**
 * Returns char mode = Body | Geom | Draw
 * @param node
 */
char XMLPrimitiveFactory::getPrimitiveMode(DOMNode* node) {
  char mode = 0;
  double mass = XMLHelper::getNodeAtt(node, XMLDefinitions::massAtt, 1.0);
  double visible = XMLHelper::getNodeAtt(node, XMLDefinitions::visibleAtt, 1.0);
  double permeable = XMLHelper::getNodeAtt(node, XMLDefinitions::permeableAtt, 0.0);
  std::cout << "body=" << (mass>0?"yes":"no") << ", draw=" << (visible?"yes":"no") << ", geom=" << (permeable?"no":"yes") << endl;
  if (mass>=XMLDefinitions::compareEPS)
    mode |= Primitive::Body;
  if (visible>0)
    mode |= Primitive::Draw;
  if (permeable==0)
    mode |= Primitive::Geom;
  return mode;
}
