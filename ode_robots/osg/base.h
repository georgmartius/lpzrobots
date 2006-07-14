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
 *  base.h provides osg stuff for basic environment with sky and so on.    *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2006-07-14 12:23:33  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.7  2006/06/29 16:35:56  robot3
 *   includes cleared up
 *
 *   Revision 1.1.2.6  2006/05/28 22:14:56  martius
 *   heightfield included
 *
 *   Revision 1.1.2.5  2006/05/18 11:45:51  robot3
 *   -shadowing the normal scene integrated (first version)
 *   -note that there is a bug that the shadow disappears
 *    after some time (e.g. 60 minutes)
 *
 *   Revision 1.1.2.4  2006/01/31 15:45:02  martius
 *   virtual destructor
 *
 *   Revision 1.1.2.3  2006/01/12 14:21:00  martius
 *   drawmode, material
 *
 *   Revision 1.1.2.2  2005/12/09 16:54:16  martius
 *   camera is woring now
 *
 *   Revision 1.1.2.1  2005/12/06 17:40:59  martius
 *   base class for simulation
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __BASE_H
#define __BASE_H

#include<ode/ode.h>
#include<osg/Transform>

#include "osghandle.h"
#include "odehandle.h"

class osg::Node;

namespace lpzrobots {

  class MoveEarthySkyWithEyePointTransform : public osg::Transform {
    public:
      /** Get the transformation matrix which moves from local coords to world coords.*/
      virtual bool computeLocalToWorldMatrix(osg::Matrix& matrix,osg::NodeVisitor* nv) const;

      /** Get the transformation matrix which moves from world coords to local coords.*/
      virtual bool computeWorldToLocalMatrix(osg::Matrix& matrix,osg::NodeVisitor* nv) const;
  };

  class Base {
  public:
    virtual osg::Group* makeScene(bool useShadow);
    virtual osg::Node* makeSky();
    virtual osg::Node* makeGround();
    virtual osg::LightSource* makeLights(osg::StateSet* stateset);  
    virtual osg::Group* createShadowedScene(osg::Node* shadowed,osg::Vec3 posOfLight, unsigned int unit);


    virtual ~Base() {}

  protected:
    dGeomID ground;

    osg::Group* root;

    OsgHandle osgHandle;
    // ODE globals
    OdeHandle odeHandle;
  };
}

#endif
