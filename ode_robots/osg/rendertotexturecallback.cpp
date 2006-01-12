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
 *   Revision 1.1.2.1  2006-01-12 14:22:37  martius
 *   deep shadow implementation
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/

#include "rendertotexturecallback.h"
#include <osg/Node>
#include <osg/PolygonOffset>
#include <osg/CullFace>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/TexGen>
#include <osg/Depth>
#include <osg/StateSet>
#include <osg/ClearNode>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/ShapeDrawable>
#include <osgUtil/RenderToTextureStage>


namespace lpzrobots {

  using namespace osg;

  const int depth_texture_height = 512;
  const int depth_texture_width  = 512;
  ref_ptr<RefMatrix> bias = new RefMatrix(0.5f, 0.0f, 0.0f, 0.0f,
					  0.0f, 0.5f, 0.0f, 0.0f,
					  0.0f, 0.0f, 0.5f, 0.0f,
					  0.5f, 0.5f, 0.5f, 1.0f);


  RenderToTextureCallback::RenderToTextureCallback(osg::Node* subgraph, 
						   osg::Texture2D* texture, 
						   osg::LightSource* light_source,
						   osg::TexGen* tex_gen):
    _subgraph(subgraph),
    _texture(texture),
    _local_stateset(new StateSet),
    _viewport(new Viewport),
    _light_projection(new RefMatrix),
    _light_source(light_source),
    _tex_gen(tex_gen)
  {
    _local_stateset->setAttribute(_viewport.get());
    _local_stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    
    ref_ptr<PolygonOffset> polygon_offset = new PolygonOffset;
    polygon_offset->setFactor(1.1f);
    polygon_offset->setUnits(4.0f);
    _local_stateset->setAttribute(polygon_offset.get(), StateAttribute::ON | StateAttribute::OVERRIDE);
    _local_stateset->setMode(GL_POLYGON_OFFSET_FILL, StateAttribute::ON | StateAttribute::OVERRIDE);
	  
    ref_ptr<CullFace> cull_face = new CullFace;
    cull_face->setMode(CullFace::FRONT);
    _local_stateset->setAttribute(cull_face.get(), StateAttribute::ON | StateAttribute::OVERRIDE);
    _local_stateset->setMode(GL_CULL_FACE, StateAttribute::ON | StateAttribute::OVERRIDE);
	  
    _viewport->setViewport(0, 0, depth_texture_width, depth_texture_height);
	  
    float znear = 1.0f * _subgraph->getBound().radius();
    float zfar  = 3.0f * _subgraph->getBound().radius();
    float top   = 0.5f * _subgraph->getBound().radius();
    float right = 0.5f * _subgraph->getBound().radius();
    znear *= 0.8f;
    zfar *= 1.2f;
    _light_projection->makeFrustum(-right, right, -top, top, znear, zfar);
  }


  void RenderToTextureCallback::_request_render_to_depth_texture(osg::Node&, osgUtil::CullVisitor& cv)
  {   
    // create the render to texture stage.
    osg::ref_ptr<osgUtil::RenderToTextureStage> rtts = new osgUtil::RenderToTextureStage;

    // set up lighting.
    // currently ignore lights in the scene graph itself..
    // will do later.
    osgUtil::RenderStage* previous_stage = cv.getCurrentRenderBin()->getStage();

    // set up the background color and clear mask.
    rtts->setClearMask(GL_DEPTH_BUFFER_BIT);
    rtts->setColorMask(new ColorMask(false, false, false, false));

    // set up to charge the same RenderStageLighting is the parent previous stage.
    rtts->setRenderStageLighting(previous_stage->getRenderStageLighting());


    // record the render bin, to be restored after creation
    // of the render to text
    osgUtil::RenderBin* previousRenderBin = cv.getCurrentRenderBin();

    osgUtil::CullVisitor::ComputeNearFarMode saved_compute_near_far_mode = cv.getComputeNearFarMode();
    cv.setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);

    // set the current renderbin to be the newly created stage.
    cv.setCurrentRenderBin(rtts.get());

    ref_ptr<RefMatrix> light_view = new RefMatrix;
    Vec4 lpos = _light_source->getLight()->getPosition();    
    light_view->makeLookAt(Vec3(lpos.x(), lpos.y(), lpos.z()), Vec3(0, 0, 0), Z_AXIS);
    Matrix texture_matrix = (*light_view.get()) * (*_light_projection.get()) * (*bias.get());
    _tex_gen->setPlane(TexGen::S, Vec4(texture_matrix(0, 0), 
				       texture_matrix(1, 0), 
				       texture_matrix(2, 0), 
				       texture_matrix(3, 0))); 
    _tex_gen->setPlane(TexGen::T, Vec4(texture_matrix(0, 1), 
				       texture_matrix(1, 1), 
				       texture_matrix(2, 1), 
				       texture_matrix(3, 1))); 
    _tex_gen->setPlane(TexGen::R, Vec4(texture_matrix(0, 2), 
				       texture_matrix(1, 2), 
				       texture_matrix(2, 2), 
				       texture_matrix(3, 2))); 
    _tex_gen->setPlane(TexGen::Q, Vec4(texture_matrix(0, 3), 
				       texture_matrix(1, 3), 
				       texture_matrix(2, 3), 
				       texture_matrix(3, 3))); 

    cv.pushProjectionMatrix(_light_projection.get());
    cv.pushModelViewMatrix(light_view.get());
    cv.pushStateSet(_local_stateset.get());

    // traverse the subgraph
    _subgraph->accept(cv);

    cv.popStateSet();
    cv.popModelViewMatrix();
    cv.popProjectionMatrix();

    cv.setComputeNearFarMode(saved_compute_near_far_mode);

    // restore the previous renderbin.
    cv.setCurrentRenderBin(previousRenderBin);

    if (rtts->getRenderGraphList().size()==0 && rtts->getRenderBinList().size()==0)
      {
	// getting to this point means that all the subgraph has been
	// culled by small feature culling or is beyond LOD ranges.
	return;
      }

    rtts->setViewport(_viewport.get());
    
    // and the render to texture stage to the current stages
    // dependancy list.
    cv.getCurrentRenderBin()->getStage()->addToDependencyList(rtts.get());

    // if one exist attach texture to the RenderToTextureStage.
    rtts->setTexture(_texture.get());
  }

}
