/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
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
 *                                                                         *
 *
 *******************************************`********************************/

#include <osgViewer/Viewer>
#include <osg/Group>

namespace lpzrobots {

  /** Viewer holds a single view on to a single scene 
      that supports the rendering of offscreen RRT (render to texture) cameras
      at any time (without sync)
  */
  class LPZViewer : public osgViewer::Viewer {
  public:
    
    LPZViewer();

    LPZViewer(osg::ArgumentParser& arguments);

    LPZViewer(const osgViewer::Viewer& viewer, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);

    virtual ~LPZViewer();

    /** call this function to render the off screen scene.
        If no off screen nodes are supplied than nothing is done      
    */
    virtual void renderOffScreen();

    /** set the group that contains the offscreen scene (usually RTT cameras)
        If no group is set or the group is empty than nothing
        is done in renderOffscreen();
     */
    virtual void setOffScreenData(osg::Group* offscreen);
    
  protected:
    virtual void offScreenRenderingTraversals();
    osg::Group* offScreenGroup;

    void lpzviewerConstructorInit();

  };

}
