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
#ifndef __RENDERTOTEXTURECALLBACK_H
#define __RENDERTOTEXTURECALLBACK_H

#include <osg/ref_ptr>
#include "osgforwarddecl.h"
//#include <osg/Node>
#include <osg/NodeCallback>
#include <osgUtil/CullVisitor>
#include <osg/Texture2D>
//#include <osg/StateSet> 
#include <osg/Viewport>
//#include <osg/LightSource>
#include <osg/TexGen>

namespace lpzrobots {

  class RenderToTextureCallback: public osg::NodeCallback {
  public:
    RenderToTextureCallback(osg::Node* subgraph, 
			    osg::Texture2D* texture, 
			    osg::LightSource* light_source,
			    osg::TexGen* tex_gen); 
    
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
      
      osgUtil::CullVisitor* cullVisitor = dynamic_cast<osgUtil::CullVisitor*>(nv);
      if (cullVisitor && _texture.valid() && _subgraph.valid())
	{            
	  _request_render_to_depth_texture(*node, *cullVisitor);
	}
      
      // must traverse the subgraph            
      traverse(node,nv);
    }
    
    void _request_render_to_depth_texture(osg::Node& node, osgUtil::CullVisitor& cv);
    
    
    osg::ref_ptr<osg::Node>                     _subgraph;
    osg::ref_ptr<osg::Texture2D>                _texture;
    osg::ref_ptr<osg::StateSet>                 _local_stateset;
    osg::ref_ptr<osg::Viewport>                 _viewport;
    osg::ref_ptr<osg::RefMatrix>                _light_projection;
    osg::ref_ptr<osg::LightSource>              _light_source;
    osg::ref_ptr<osg::TexGen>                   _tex_gen;
  };

}

#endif
