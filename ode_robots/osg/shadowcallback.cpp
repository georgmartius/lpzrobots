/***************************************************************************
 *  callback class for a shadowed scene                                    *
 *                                                                         *
 *  // originally from osgdepthshadow example                              *
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
 *   Revision 1.1.2.2  2006-05-18 11:33:17  robot3
 *   imports optimized
 *
 *   Revision 1.1.2.1  2006/05/18 11:25:59  robot3
 *   -first open version
 *   -note that there is probably a precision problem that causes
 *    the disappearing of the shadow after some time (after ca. 60
 *    minutes simulation time)
 *
 *                                                                         *
 ***************************************************************************/
#include <osgProducer/Viewer>

#include "shadowcallback.h"

ShadowDrawCallback::ShadowDrawCallback(osg::Vec3 posOfLight, osg::CameraNode* cameraNode,
				       osg::TexGenNode* texgenNode):
  posOfLight(posOfLight), _cameraNode(cameraNode), _texgenNode(texgenNode) {}

       
void ShadowDrawCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
  // first update subgraph to make sure objects are all moved into postion
  traverse(node,nv);
            
  // now compute the camera's view and projection matrix to point at the shadower (the camera's children)
  osg::BoundingSphere bs;
  for(unsigned int i=0; i<_cameraNode->getNumChildren(); ++i)
    {
      bs.expandBy(_cameraNode->getChild(i)->getBound());
    }
            
  if (!bs.valid())
    {
      std::cout << "bb invalid!" << std::endl;
      osg::notify(osg::WARN) << "bb invalid"<<_cameraNode.get()<<std::endl;
      return;
    }
            
  float centerDistance = (posOfLight-bs.center()).length();

  float znear = centerDistance-bs.radius();
  float zfar  = centerDistance+bs.radius();
  float zNearRatio = 0.001f;
  if (znear<zfar*zNearRatio) znear = zfar*zNearRatio;

#if 0
  // hack to illustrate the precision problems of excessive gap between near far range.
  znear = 0.00001*zfar;
#endif
  float top   = (bs.radius()/centerDistance)*znear;
  float right = top;

  _cameraNode->setReferenceFrame(osg::CameraNode::ABSOLUTE_RF);
  _cameraNode->setProjectionMatrixAsFrustum(-right,right,-top,top,znear,zfar);
  _cameraNode->setViewMatrixAsLookAt(posOfLight,bs.center(),osg::Vec3(0.0f,1.0f,0.0f));

  // compute the matrix which takes a vertex from local coords into tex coords
  // will use this later to specify osg::TexGen..
  osg::Matrix MVPT = _cameraNode->getViewMatrix() * 
    _cameraNode->getProjectionMatrix() *
    osg::Matrix::translate(1.0,1.0,1.0) *
    osg::Matrix::scale(0.5f,0.5f,0.5f);
                               
  _texgenNode->getTexGen()->setMode(osg::TexGen::EYE_LINEAR);
  _texgenNode->getTexGen()->setPlanesFromMatrix(MVPT);

}
        
  
ShadowDrawCallback::~ShadowDrawCallback() {}
        

