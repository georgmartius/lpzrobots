/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2006-07-14 12:23:33  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.8  2006/06/29 16:35:56  robot3
 *   includes cleared up
 *
 *   Revision 1.1.2.7  2006/06/27 14:14:29  robot3
 *   -optimized mesh and boundingshape code
 *   -other changes
 *
 *   Revision 1.1.2.6  2006/06/26 21:52:58  robot3
 *   Mesh works now with bbox file
 *
 *   Revision 1.1.2.5  2006/06/23 08:54:40  robot3
 *   made some changes on primitive Mesh (including boundingshape)
 *
 *   Revision 1.1.2.4  2006/06/22 11:33:43  robot3
 *   moved boundingshape implementation to .cpp-file
 *
 *   Revision 1.1.2.3  2006/05/29 22:22:07  martius
 *   added std includes
 *
 *   Revision 1.1.2.2  2006/03/29 15:03:19  martius
 *   format documented
 *
 *   Revision 1.1.2.1  2006/03/29 14:51:45  martius
 *   class for reading bounding shape description files and creates the appropriate geoms
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __BOUNDINGSHAPE_H
#define __BOUNDINGSHAPE_H

#include "primitive.h"

#include <list>
#include <string>


namespace lpzrobots {
  
  /**
     class for reading bounding shape description files (.bbox) and to create appropriate geoms
     File Format: Lines wise, every line stands for one primitive. Possible lines are:
     sphere radius (x,y,z)
     cylinder radius height (x,y,z) (alpha, beta, gamma)
     capsule radius height (x,y,z) (alpha, beta, gamma)
     box length width height (x,y,z) (alpha, beta, gamma)
     (x,y,z) is the position vector and (alpha, beta, gamma) are 
     the rotation angles about x,y,z axis respectively        
  */

  class BoundingShape{

  public:
    BoundingShape(const std::string& filename, Primitive* parent);

    virtual ~BoundingShape();

    bool readBBoxFile(std::string& filename, const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
		      double scale, char mode);
      
    virtual bool init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      double scale, char mode);
    
    virtual bool isActive();

  private:
    std::string filename;
    bool active;
    Primitive* parent;
  };

}

#endif
