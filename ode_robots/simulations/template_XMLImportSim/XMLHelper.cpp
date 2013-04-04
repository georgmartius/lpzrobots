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
 *  Revision 1.4  2010-06-15 15:02:19  guettler
 *  using now "XercescForwardDecl.h" to avoid namespace problems (3_0, 3_1)
 *
 *  Revision 1.3  2010/03/12 09:11:58  guettler
 *  debug cout color improved
 *
 *  Revision 1.2  2010/03/08 07:20:00  guettler
 *  - remove const return from some methods
 *  - fixed setPose
 *
 *  Revision 1.1  2010/03/07 22:50:38  guettler
 *  first development state for feature XMLImport
 *                       *
 *                                                                         *
 **************************************************************************/
#include "XMLHelper.h"

#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOMNode.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/dom/DOMNamedNodeMap.hpp>

#include "XMLErrorHelper.h"
#include "XMLDefinitions.h"

#include <string>
#include <stdlib.h>

#include <ode_robots/mathutils.h>

using namespace XERCESC;
using namespace std;
using namespace osg;
using namespace lpzrobots;

XString::XString(const char* const toTranscode) {
  unicodeChars = XMLString::transcode(toTranscode);
  cChars = XMLString::transcode(unicodeChars);
}

XString::XString(const std::string toTranscode) {
  unicodeChars = XMLString::transcode(toTranscode.c_str());
  cChars = XMLString::transcode(unicodeChars);
}

XString::XString(const XMLCh* toTranscode) {
  cChars = XMLString::transcode(toTranscode);
  unicodeChars = XMLString::transcode(cChars);
}


XString::~XString() {
  XMLString::release(&unicodeChars);
  XMLString::release(&cChars);
}

const XMLCh* XString::unicodeForm() const {
  return unicodeChars;
}

const char* XString::charForm() const {
  return cChars;
}

const string XMLHelper::getNodeType(const xercesc::DOMNode* node) {
  switch (node->getNodeType()) {
    case 1:
      return "ELEMENT_NODE";
      break;
    case 2:
      return "ATTRIBUTE_NODE";
      break;
    case 3:
      return "TEXT_NODE";
      break;
    case 4:
      return "CDATA_SECTION_NODE";
      break;
    case 5:
      return "ENTITY_REFERENCE_NODE";
      break;
    case 6:
      return "ENTITY_NODE";
      break;
    case 7:
      return "PROCESSING_INSTRUCTION_NODE";
      break;
    case 8:
      return "COMMENT_NODE";
      break;
    case 9:
      return "DOCUMENT_NODE";
      break;
    case 10:
      return "DOCUMENT_TYPE_NODE";
      break;
    case 11:
      return "DOCUMENT_FRAGMENT_NODE";
      break;
    case 12:
      return "NOTATION_NODE";
      break;
    default:
      return "UNKNOWN_NODE";
      break;
  }
}

const bool XMLHelper::matchesName(const DOMNode* childNode, const string childNodeName) {
  if (childNode!=0 && strcmp(C(childNode->getNodeName()),childNodeName.c_str())==0)
    return true;
  return false;
}

double XMLHelper::getNodeValue(const DOMNode* node, const double defaultValue /* = 0.0 */)
{
  if (node!=0) {
    try {
      return atof(C(node->getNodeValue()));
    } catch (DOMException e) {
      XMLErrorHelper::printError(e);
    }
  }
  return defaultValue;
}

double XMLHelper::getNodeAtt(const DOMNode* node, const string value, const double defaultValue /* = 0.0 */)
{
          if (node!=0) {
                const DOMNode* attributeNode = node->getAttributes()->getNamedItem(X(value));
            if (attributeNode!=0)
              return getNodeValue(attributeNode, defaultValue);
            //return getNodeValue(getNode(node, value), defaultValue);
          }
          return defaultValue;
}

string XMLHelper::getNodeAttAsString(const DOMNode* node, const string value, const string defaultValue /* = "" */)
{
  if (node!=0) {
        const DOMNode* attributeNode = node->getAttributes()->getNamedItem(X(value));
    if (attributeNode!=0)
      return getNodeValueAsString(attributeNode, defaultValue);
    //return getNodeValueAsString(getNode(node, value), defaultValue);
  }
  return defaultValue;
}


double XMLHelper::getChildNodeValue(const DOMNode* node, const string childNodeName, const string childValue, const double defaultValue /* = 0.0 */)
{
          if (node!=0) {
                const DOMNode* childNode = getChildNode(node,childNodeName);
                const DOMNode* attributeNode = childNode->getAttributes()->getNamedItem(X(childValue));
            if (attributeNode!=0)
              return getNodeValue(attributeNode, defaultValue);
            return getNodeValue(getChildNode(childNode, childValue), defaultValue);
          }
          return defaultValue;
}

string XMLHelper::getNodeValueAsString(const DOMNode* node, const string defaultValue /* = "" */)
{
  if (node!=0) {
    try {
      return C(node->getNodeValue());
    } catch (DOMException e) {
      XMLErrorHelper::printError(e);
    }
  }
  return defaultValue;
}

string XMLHelper::getChildNodeValueAsString(const DOMNode* node, const string childNodeName, const string childValue, const string defaultValue /* = "" */)
{
  if (node!=0) {
        const DOMNode* childNode = getChildNode(node,childNodeName);
        const DOMNode* attributeNode = childNode->getAttributes()->getNamedItem(X(childValue));
    if (attributeNode!=0)
      return getNodeValueAsString(attributeNode, defaultValue);
    return getNodeValueAsString(getChildNode(childNode, childValue), defaultValue);
  }
  return defaultValue;
}



const DOMNode* XMLHelper::getChildNode(const DOMNode* node, const string childNodeName) {
  if (node!=0) {
    for EACHCHILDNODE(node,childNode) {
      if (matchesName(childNode, childNodeName))
        return childNode;
    }
  }
  return 0;
}

const Vec3 XMLHelper::getPosition(const DOMNode* node) {
  // <position x="24.54" y="3.23" z="2.342"/>
  if (node!=0) {
    const DOMNode* posNode = getChildNode(node, XMLDefinitions::positionNode);
    if (posNode!=0) {
    cout << "  Position found " << getNodeAtt(posNode, XMLDefinitions::xAtt, 0) << " "<< getNodeAtt(posNode, XMLDefinitions::yAtt, 0) << " " << getNodeAtt(posNode, XMLDefinitions::zAtt, 0) << endl;
    return Vec3(getNodeAtt(posNode, XMLDefinitions::xAtt, 0),
                    getNodeAtt(posNode, XMLDefinitions::yAtt, 0),
                    getNodeAtt(posNode, XMLDefinitions::zAtt, 0));
    }
  }
  return Vec3(0,0,0);
}

const Vec3 XMLHelper::getViewPosition(const DOMNode* node) {
  // <position x="24.54" y="3.23" z="2.342"/>
  if (node!=0) {
    const DOMNode* posNode = getChildNode(node, XMLDefinitions::viewPositionNode);
    if (posNode!=0) {
    cout << "  ViewPosition found" << endl;
    return Vec3(getNodeAtt(posNode, XMLDefinitions::xAtt, 0),
                    getNodeAtt(posNode, XMLDefinitions::yAtt, 0),
                    getNodeAtt(posNode, XMLDefinitions::zAtt, 0));
    }
  }
  return Vec3(0,0,0);
}

const Vec3 XMLHelper::getRotation(const DOMNode* node) {
  // <rotation alpha="44.345534" beta="90.354544" gamma="-135.366342"/>
  if (node!=0) {
    const DOMNode* rotNode = getChildNode(node, XMLDefinitions::rotationNode);
    if (rotNode!=0) {
    return Vec3(getNodeAtt(rotNode, XMLDefinitions::alphaAtt),
                    getNodeAtt(rotNode, XMLDefinitions::betaAtt),
                    getNodeAtt(rotNode, XMLDefinitions::gammaAtt));
      }
  }
  return Vec3(0,0,0);
}

const Matrix XMLHelper::getPose(const DOMNode* node, double forcedScale /* = 0 */) {
  const Vec3 rot = getRotation(node);
  double scale = forcedScale==0 ? getNodeAtt(node, XMLDefinitions::scaleAtt, 1.0) : forcedScale;
  const Vec3 pos = getPosition(node);
  return osgRotate(rot[0]*M_PI/180.0f,rot[1]*M_PI/180.0f,rot[2]*M_PI/180.0f)
                   *osg::Matrix::translate(scale*pos[0],scale*pos[1],scale*pos[2]);
}

const lpzrobots::Color XMLHelper::getColor(const DOMNode* node) {
        const DOMNode* colorNode = getChildNode(node,XMLDefinitions::colorNode);
        if (colorNode!=0) {
                double redValue = getNodeAtt(colorNode,XMLDefinitions::redAtt,255);
                double greenValue = getNodeAtt(colorNode,XMLDefinitions::greenAtt,255);
                double blueValue = getNodeAtt(colorNode,XMLDefinitions::blueAtt,255);
                double alphaValue = getNodeAtt(colorNode,XMLDefinitions::alphacolorAtt,255);
                cout << "  Color found: red=" << redValue << ", green="<< greenValue << ", blue="<< blueValue << ", alpha="<< alphaValue << endl;
                // calculate values between 0...1 (XML: 0...255)
                return Color(redValue/255.0,greenValue/255.0,blueValue/255.0,alphaValue/255.0);
        }
        else
                return Color(1,1,1,0);
}

const Vec3 XMLHelper::getGeometry(const DOMNode* node) {
  //<geometry length="24.54" width="3.23" height="2.342"/>
  if (getChildNode(node, XMLDefinitions::geometryNode)!=0) {
    const DOMNode* posNode = getChildNode(node, XMLDefinitions::geometryNode);
    if (posNode!=0) {
    cout << "  Geometry found " << getNodeAtt(posNode, XMLDefinitions::lengthAtt, 0) << " "<< getNodeAtt(posNode, XMLDefinitions::widthAtt, 0) << " " << getNodeAtt(posNode, XMLDefinitions::heightAtt, 0) << endl;

    return Vec3(getNodeAtt(posNode, XMLDefinitions::lengthAtt, 0),
                    getNodeAtt(posNode, XMLDefinitions::widthAtt, 0),
                    getNodeAtt(posNode, XMLDefinitions::heightAtt, 0));
    }
  }
  //cout << "geometrynode not found" << endl;
  return Vec3(0,0,0);
}


