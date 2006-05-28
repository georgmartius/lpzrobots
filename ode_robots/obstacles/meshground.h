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
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.1  2006-05-28 22:14:56  martius
 *   heightfield included
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __MESHGROUND_H
#define __MESHGROUND_H

#include <stdio.h>
#include <math.h>

#include "heightfieldprimitive.h"
#include "abstractobstacle.h"
 
namespace lpzrobots {

  class MeshGround : public AbstractObstacle {
  protected:

    std::string filename;
    std::string texture;
    HeightField* heightfield;
    double x_size;
    double y_size;
    double height;
    OSGHeightField::CodingMode coding;

  public:
  
    MeshGround(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
	       const std::string& filename, const std::string& texture, 
	       double x_size, double y_size, double height,
	       OSGHeightField::CodingMode coding = OSGHeightField::Red);

    /**
     * updates the position of the geoms  ( not nessary for static objects)
     */
    virtual void update(){
    };
  
  
    virtual void setPose(const osg::Matrix& pose);


  protected:
    virtual void create();


    virtual void destroy();    

  };
}

#endif
