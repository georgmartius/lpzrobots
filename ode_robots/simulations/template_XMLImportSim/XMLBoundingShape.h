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
 *  Revision 1.2  2010/03/11 15:18:06  guettler
 *  -BoundingShape can now be set from outside (see XMLBoundingShape)
 *  -Mesh can be created without Body and Geom.
 *  -various bugfixes
 *
 *  Revision 1.1  2010/03/07 22:50:38  guettler
 *  first development state for feature XMLImport
 *                                                                                   *
 *                                                                         *
 **************************************************************************/
#ifndef __XMLBOUNDINGSHAPE_H_
#define __XMLBOUNDINGSHAPE_H_

#include <ode_robots/boundingshape.h>
#include "XMLObject.h"

/**
 * Class which builds up the BoundingShape from a DOMNode instead
 * of an .bbox-file.
 */
class XMLBoundingShape : public lpzrobots::BoundingShape, public XMLObject {
  public:
  /**
   * @param boundingBoxNode which contains necessary information about the
   *        used Primitives for the BoundingShape.
   * @param parent primitive to which the BoundingShape is associated
   */
    XMLBoundingShape(const XERCESC::DOMNode* boundingBoxNode, XMLParserEngine& engine, lpzrobots::Mesh* parent);
    virtual ~XMLBoundingShape();

     /// tries to open the bbox file and greates all geoms
     virtual bool init(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle,
           double scale, char mode);

  protected:
     const XERCESC::DOMNode* boundingBoxNode;
   };



#endif /* __XMLBOUNDINGSHAPE_H_ */
