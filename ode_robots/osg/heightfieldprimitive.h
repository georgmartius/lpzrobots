/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   Revision 1.4  2010-03-09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.3  2007/07/17 07:18:19  martius
 *   setMass added (dummy)
 *
 *   Revision 1.2  2006/07/14 12:23:34  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2006/05/28 22:14:56  martius
 *   heightfield included
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __HEIGHTFIELDPRIMITIVES_H
#define __HEIGHTFIELDPRIMITIVES_H

#include "primitive.h"
#include "osgheightfield.h"
#include <ode-dbl/ode.h>

namespace lpzrobots {


  /** Height field primitive */
  class HeightField : public Primitive {
  public:
    HeightField(const std::string& filename, float x_size, float y_size, float height);
    HeightField(osg::HeightField* heightfield, float x_size, float y_size);
    
    virtual ~HeightField();
    virtual void init(const OdeHandle& odeHandle, double mass,
		      const OsgHandle& osgHandle,
		      char mode = Body | Geom | Draw) ;
    
    
    virtual void setPose(const osg::Matrix& pose);
    
    virtual void update();
    virtual OSGPrimitive* getOSGPrimitive() { return osgheightfield; }

    virtual void setMass(double mass) {}
  
  protected:
    OSGHeightField* osgheightfield;
    dTriMeshDataID data;
    
    OSGPrimitive* test[20];
  };


}

#endif
