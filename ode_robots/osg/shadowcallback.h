/***************************************************************************
 *  callback class for a shadowed scene                                    *
 *                                                                         *
 ***************************************************************************/
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
 *   Revision 1.1.2.1  2006-05-18 11:32:58  robot3
 *   -first open version
 *   -note that there is probably a precision problem that causes
 *    the disappearing of the shadow after some time (after ca. 60
 *    minutes simulation time)
 *   -imports optimized
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __SHADOWCALLBACK_H
#define __SHADOWCALLBACK_H


#include <osg/CameraNode>
#include <osg/TexGenNode>


class ShadowDrawCallback : public osg::NodeCallback
{
 public:

  /**
   * creates the shadow draw callback for the shadowed scene
   * note that the sky and the ground is not shadowed,
   * use any playground for having shadows on the roof
   */    
  ShadowDrawCallback(osg::Vec3 posOfLight, osg::CameraNode* cameraNode, osg::TexGenNode* texgenNode);
  
  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
  
 protected:
    
  virtual ~ShadowDrawCallback();
        
  osg::Vec3 posOfLight;
  osg::ref_ptr<osg::CameraNode> _cameraNode;
  osg::ref_ptr<osg::TexGenNode> _texgenNode;
  
};

#endif
