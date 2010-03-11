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
 *   Revision 1.5  2010-03-11 15:17:19  guettler
 *   -BoundingShape can now be set from outside (see XMLBoundingShape)
 *   -Mesh can be created without Body and Geom.
 *
 *   Revision 1.4  2010/03/07 22:39:45  guettler
 *   variables are now protected instead of private for inheritance issues
 *
 *   Revision 1.3  2006/08/11 15:41:40  martius
 *   osgDB used to find path
 *
 *   Revision 1.2  2006/07/14 12:23:33  martius
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
#include "odehandle.h"

#include <string>
#include <vector>


namespace lpzrobots {
  
  /**
     class for reading bounding shape description files (.bbox) and to create appropriate geoms

     File Format: Lines wise, every line stands for one primitive. 

     Possible lines are:
     - sphere radius (x,y,z)
     - cylinder radius height (x,y,z) (alpha, beta, gamma)
     - capsule radius height (x,y,z) (alpha, beta, gamma)
     - box length width height (x,y,z) (alpha, beta, gamma)
     
     (x,y,z) is the position vector and (alpha, beta, gamma) are 
     the rotation angles about x,y,z axis respectively        

     Example:
     \code
cylinder 6.5 50 (0,0,25) (0,0,0)
cylinder 50 15 (0,0,28) (0,0,0)
cylinder 40 30 (0,0,50) (0,0,0)
cylinder 30 20 (0,0,75) (0,0,0)
cylinder 20 30 (0,0,100) (0,0,0)
cylinder 13 30 (0,0,125) (0,0,0)
cylinder 8 30 (0,0,150) (0,0,0)
cylinder 5 30 (0,0,175) (0,0,0)
     \endcode
  */

  class BoundingShape{

  public:
    /**
       @param filename path and name of bbox file. It is located using OsgDB search path
       @param parent primitive to which the bbox is assoziated
    */
    BoundingShape(const std::string& filename, Mesh* parent);

    virtual ~BoundingShape();

    /// tries to open the bbox file and greates all geoms
    virtual bool init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      double scale, char mode);
    
    virtual bool isActive();

    /**
     * updates all Primitives of the BoundingShape if only in geom mode (no Body)
     * @param pose
     */
    virtual void setPose(const osg::Matrix& pose);

  private:
    bool readBBoxFile(std::string& filename, const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
		      double scale, char mode);

  protected:
    std::string filename;
    bool active;
    Primitive* parent;
    bool attachedToParentBody; // true as default, false not yet implemented by BoundingShape
    std::vector<Primitive*> boundingPrimitiveList; // used if not attached to a body
    std::vector<osg::Matrix> boundingPrimitivePoseList; // stores the relative pose of each primitive
    OdeHandle odeHandle;
    dSpaceID parentSpace;
  };

}

#endif
