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
 *   Revision 1.6  2007-07-03 13:06:41  martius
 *   groundplane thick
 *
 *   Revision 1.5  2007/04/05 15:10:14  martius
 *   extra ground substance
 *
 *   Revision 1.4  2007/03/16 11:01:37  martius
 *   abstractobstacle gets mor functionallity
 *   setSubstance
 *
 *   Revision 1.3  2006/08/11 15:41:04  martius
 *   playgrounds handle non-quadratic ground planes
 *
 *   Revision 1.2  2006/07/14 12:23:32  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.9  2006/07/14 11:36:32  martius
 *   revert to older revision of robot3
 *
 *   Revision 1.1.2.8  2006/07/13 12:11:26  robot5
 *   Using overhauled primitives Plane and Box.
 *   Repeat texturing is supported by them now.
 *
 *   Revision 1.1.2.7  2006/06/29 16:39:55  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.1.2.6  2006/06/26 08:25:03  robot3
 *   fixed ground texture bug
 *
 *   Revision 1.1.2.5  2006/06/22 12:25:45  der
 *   added setGroundTexture and setGroundColor
 *
 *   Revision 1.1.2.4  2006/06/16 22:27:26  martius
 *   getMainPrimtive
 *
 *   Revision 1.1.2.3  2006/05/23 14:13:41  der
 *   fixed initialization bug
 *
 *   Revision 1.1.2.2  2006/05/23 13:37:45  robot3
 *   -fixed some creating bugs
 *   -setColor,setTexture and createGround must be
 *    called before setPosition now
 *
 *   Revision 1.1.2.1  2006/05/19 08:41:27  robot3
 *   Class AbstractGround contains now basic routines like
 *   creating the groundPlane, setPose and so on
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __ABSTRACTGROUND_H
#define __ABSTRACTGROUND_H

#include <vector>
#include "abstractobstacle.h"

namespace lpzrobots {

  class Primitive; 

  // abstract class for any playground
  class AbstractGround : public AbstractObstacle {

public:

    AbstractGround(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
		   bool createGround, double groundLength, double groundWidth, double wallThickness);

    virtual ~AbstractGround();


    virtual void setPose(const osg::Matrix& pose);

    virtual void createGround(bool create);

    virtual void setTexture(const std::string& filename);

    virtual Primitive* getMainPrimitive() const;

    /**
     * assigns the texture to the object
     */
    virtual void setGroundTexture(const std::string& filename);

    /**
     * sets the ground color
     * should be called before setPosition()
     * @param color values in RGBA
     */
    virtual void setGroundColor(const Color& color);

    /**
     * sets the substance of the ground. 
     * @param substance description of the substance
     */
    virtual void setGroundSubstance(const Substance& substance);


  protected:

    Primitive* groundPlane; // the groundplane
    bool creategroundPlane;
    double groundLength;
    double groundWidth;
    double wallThickness;
    std::string wallTextureFileName;
    Color groundColor;
    std::string groundTextureFileName;

    virtual void createGround();

  };

}

#endif
