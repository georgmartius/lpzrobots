/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/

#include "lpzviewer.h"

#include <osgViewer/ViewerBase>
#include <osgViewer/View>
#include <osgViewer/Renderer>

#include <osgUtil/Statistics>

namespace lpzrobots {

  using namespace osgViewer;

  LPZViewer::LPZViewer(){
    lpzviewerConstructorInit();
  }

  LPZViewer::LPZViewer(osg::ArgumentParser& arguments)
    : osgViewer::Viewer(arguments) {
    lpzviewerConstructorInit();
  }

  LPZViewer::LPZViewer(const osgViewer::Viewer& viewer, const osg::CopyOp& copyop)
    : osgViewer::Viewer(viewer,copyop) {
    lpzviewerConstructorInit();
  }

  LPZViewer::~LPZViewer(){}


  void LPZViewer::setOffScreenData(osg::Group* offscreen){
    offScreenGroup=offscreen;
  }


  void LPZViewer::lpzviewerConstructorInit(){
    offScreenGroup = 0;
  }

  void LPZViewer::setUpThreading(){
    Viewer::setUpThreading();
    OpenThreads::SetProcessorAffinityOfCurrentThread(0xFFFF);
  }
  void LPZViewer::startThreading(){
    Viewer::startThreading();
    OpenThreads::SetProcessorAffinityOfCurrentThread(0xFFFF);
  }


  bool LPZViewer::needForOffScreenRendering(){
    return !_done && offScreenGroup && offScreenGroup->getNumChildren() != 0;
  }


  void LPZViewer::renderOffScreen()
  {
    if(!needForOffScreenRendering()) return;
    osg::Node* origNode = _camera->getChild(0);
    _camera->setChild(0,offScreenGroup);
    osg::Camera::DrawCallback* origFDC = _camera->getFinalDrawCallback();
    _camera->setFinalDrawCallback(0);

    // printf("offscreen rendering \n");
    updateTraversal(); // we don't need that (simulationTime of OSG is anyway to updated)
    offScreenRenderingTraversals();
    _camera->setChild(0,origNode);
    _camera->setFinalDrawCallback(origFDC);

  }


  void LPZViewer::offScreenRenderingTraversals()
  {

    /*** This is copied from ViewerBase::renderingTraversals() and
         statistics and swapbuffer and so on is removed.
    */

    if (_done) return;

    // might not need it because our nodes are also in the main scenegraph
    //  (however, this might not be rendered very often)
    for(unsigned int i=0; i< _camera->getNumChildren(); i++){
      _camera->getChild(i)->getBound();
    }

    // osg::notify(osg::NOTICE)<<std::endl<<"Start frame"<<std::endl;

    Contexts contexts;
    getContexts(contexts);

    Cameras cameras;
    getCameras(cameras);

    Contexts::iterator itr;

    bool doneMakeCurrentInThisThread = false;

    if (_endDynamicDrawBlock.valid())
      {
        _endDynamicDrawBlock->reset();
      }

    // dispatch the rendering threads
    if (_startRenderingBarrier.valid()) _startRenderingBarrier->block();

    // reset any double buffer graphics objects
    for(Cameras::iterator camItr = cameras.begin();
        camItr != cameras.end();
        ++camItr)
      {
        osg::Camera* camera = *camItr;
        Renderer* renderer = dynamic_cast<Renderer*>(camera->getRenderer());
        if (renderer)
          {
            if (!renderer->getGraphicsThreadDoesCull() && !(camera->getCameraThread()))
              {
                renderer->cull();
              }
          }

      }

    for(itr = contexts.begin();
        itr != contexts.end();
        ++itr)
      {
        if (_done) return;
        if (!((*itr)->getGraphicsThread()) && (*itr)->valid())
          {
            doneMakeCurrentInThisThread = true;
            makeCurrent(*itr);
            (*itr)->runOperations();
          }
      }


    // wait till the rendering dispatch is done.
    if (_endRenderingDispatchBarrier.valid()) _endRenderingDispatchBarrier->block();

    // wait till the dynamic draw is complete.
    if (_endDynamicDrawBlock.valid())
      {
        _endDynamicDrawBlock->block();
      }

    if (_releaseContextAtEndOfFrameHint && doneMakeCurrentInThisThread)
      {
        //osg::notify(osg::NOTICE)<<"Doing release context"<<std::endl;
        releaseContext();
      }

  }

}
